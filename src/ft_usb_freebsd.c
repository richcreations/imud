/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * ft_usb_freebsd.c — include/ft_usb.h over libusb20.
 *
 * libusb20 is FreeBSD's own USB access layer, in base beside ugen(4) and
 * headed by <libusb20.h>, so this adds no dependency the tree does not
 * already assume — it stands to ugen as src/ft_usb_linux.c stands to usbfs.
 *
 * It needs no privilege beyond rw on the /dev/ugenX.Y node.
 *
 * Device identity is read from the kernel's cached descriptors
 * (libusb20_dev_get_info, libusb20_dev_get_device_desc) rather than by
 * fetching string descriptors from the part.  That is one fewer control
 * transfer per candidate during enumeration, and it is the only form that
 * works through a hypervisor's USB passthrough, where an on-demand string
 * read can fail while the cached copy is intact.
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <dev/usb/usb_ioctl.h>
#include <libusb20.h>
#include <libusb20_desc.h>

#include "ft_usb.h"
#include "log.h"

/* Generous enough for any MPSSE batch src/bus_ft232h.c builds, and the bound
 * libusb20_tr_bulk_intr_sync() enforces on a single transfer. */
#define FT_XFER_MAX 65536

struct ft_usb {
    struct libusb20_device   *dev;
    struct libusb20_transfer *xin, *xout;
    unsigned ifno;
    uint8_t  ep_in, ep_out;
    unsigned pktsz;
    int      opened;
    char     name[352];
};

/* The ugen node name for a device, "ugen0.2" — FreeBSD's equivalent of the
 * Linux port path, and what usbconfig(8) calls it. */
static void ugen_name(struct libusb20_device *d, char *out, size_t n)
{
    snprintf(out, n, "ugen%u.%u", libusb20_dev_get_bus_number(d),
             libusb20_dev_get_address(d));
}

/* Does this device match what the operator asked for? */
static int matches(struct libusb20_device *d, const char *want)
{
    if (!want || want[0] == '\0') return 1;

    char node[32];
    ugen_name(d, node, sizeof node);
    if (strcmp(node, want) == 0) return 1;

    char topo[32];
    snprintf(topo, sizeof topo, "%u.%u", libusb20_dev_get_bus_number(d),
             libusb20_dev_get_address(d));
    if (strcmp(topo, want) == 0) return 1;

    struct usb_device_info info;
    if (libusb20_dev_get_info(d, &info) == 0 &&
        strcmp(info.udi_serial, want) == 0)
        return 1;

    return 0;
}

/*
 * The IN endpoint's wMaxPacketSize, from the active configuration descriptor.
 * Falls back to the high-speed 512 rather than failing: a wrong bound here
 * costs a split read, not a wrong one.
 */
static unsigned ep_packet_size(struct libusb20_device *d, unsigned ifno,
                               uint8_t ep)
{
    unsigned sz = 512;

    struct libusb20_config *cfg =
        libusb20_dev_alloc_config(d, libusb20_dev_get_config_index(d));
    if (!cfg) return sz;

    if (ifno < cfg->num_interface) {
        const struct libusb20_interface *itf = &cfg->interface[ifno];
        for (uint8_t i = 0; i < itf->num_endpoints; i++) {
            const struct LIBUSB20_ENDPOINT_DESC_DECODED *e =
                &itf->endpoints[i].desc;
            if (e->bEndpointAddress == ep && e->wMaxPacketSize)
                sz = e->wMaxPacketSize;
        }
    }

    free(cfg);
    return sz;
}

ft_usb_t *ft_usb_open(const char *want, uint16_t vid, uint16_t pid,
                      unsigned ifno)
{
    struct libusb20_backend *be = libusb20_be_alloc_default();
    if (!be) {
        errno = ENOMEM;
        return NULL;
    }

    struct libusb20_device *d = NULL, *found = NULL;
    while (!found && (d = libusb20_be_device_foreach(be, d)) != NULL) {
        struct LIBUSB20_DEVICE_DESC_DECODED *dd = libusb20_dev_get_device_desc(d);
        if (!dd || dd->idVendor != vid || dd->idProduct != pid) continue;
        if (!matches(d, want)) continue;
        found = d;
    }

    if (!found) {
        LOG_E("ft232h: no %04x:%04x%s%s found\n", vid, pid,
              (want && *want) ? " matching " : "", (want && *want) ? want : "");
        libusb20_be_free(be);
        errno = ENODEV;
        return NULL;
    }

    /* Take the device out of the backend's list so freeing the backend does
     * not free it with everything else it enumerated. */
    libusb20_be_dequeue_device(be, found);
    libusb20_be_free(be);

    ft_usb_t *u = calloc(1, sizeof *u);
    if (!u) {
        libusb20_dev_free(found);
        return NULL;
    }
    u->dev = found;

    char node[32];
    ugen_name(found, node, sizeof node);

    /* Two transfers: one per endpoint, indexed below by ft_usb_read/write. */
    if (libusb20_dev_open(found, 2) != 0) {
        LOG_E("ft232h: cannot open /dev/%s: %s\n", node, strerror(errno));
        libusb20_dev_free(found);
        free(u);
        errno = EACCES;
        return NULL;
    }
    u->opened = 1;

    /* An FT232H arrives claimed by uftdi(4), and MPSSE is unreachable until it
     * lets go.  Unlike usbfs there is no call to hand the interface back, so
     * ft_usb_close() cannot restore it and a replug is what returns the serial
     * port — libusb's own attach_kernel_driver is unimplemented here for the
     * same reason. */
    (void)libusb20_dev_detach_kernel_driver(found, (uint8_t)ifno);

    u->ifno   = ifno;
    /* FTDI numbers its interfaces' endpoints in pairs from 0x02/0x81; the
     * FT232H has only interface A. */
    u->ep_out = (uint8_t)(0x02 + ifno * 2);
    u->ep_in  = (uint8_t)(0x81 + ifno * 2);
    u->pktsz  = ep_packet_size(found, ifno, u->ep_in);

    u->xin  = libusb20_tr_get_pointer(found, 0);
    u->xout = libusb20_tr_get_pointer(found, 1);
    if (!u->xin || !u->xout ||
        libusb20_tr_open(u->xin,  FT_XFER_MAX, 1, u->ep_in)  != 0 ||
        libusb20_tr_open(u->xout, FT_XFER_MAX, 1, u->ep_out) != 0) {
        LOG_E("ft232h: cannot open endpoints on /dev/%s\n", node);
        ft_usb_close(u);
        errno = EIO;
        return NULL;
    }

    struct usb_device_info info;
    snprintf(u->name, sizeof u->name, "/dev/%s (%s)", node,
             libusb20_dev_get_info(found, &info) == 0 && info.udi_serial[0]
                 ? info.udi_serial : node);

    LOG_I("ft232h: %s, %u-byte packets\n", u->name, u->pktsz);
    return u;
}

void ft_usb_close(ft_usb_t *u)
{
    if (!u) return;

    if (u->xin)  (void)libusb20_tr_close(u->xin);
    if (u->xout) (void)libusb20_tr_close(u->xout);
    if (u->opened) (void)libusb20_dev_close(u->dev);
    if (u->dev) libusb20_dev_free(u->dev);
    free(u);
}

int ft_usb_control(ft_usb_t *u, uint8_t request, uint16_t value)
{
    struct LIBUSB20_CONTROL_SETUP_DECODED setup;
    LIBUSB20_INIT(LIBUSB20_CONTROL_SETUP, &setup);

    setup.bmRequestType = 0x40;                     /* vendor, host to device */
    setup.bRequest      = request;
    setup.wValue        = value;
    setup.wIndex        = (uint16_t)(u->ifno + 1);  /* FTDI indexes A as 1 */
    setup.wLength       = 0;

    uint16_t actlen = 0;
    if (libusb20_dev_request_sync(u->dev, &setup, NULL, &actlen, 1000, 0) != 0) {
        errno = EIO;
        return -1;
    }
    return 0;
}

static int bulk(struct libusb20_transfer *xfer, void *buf, unsigned len,
                unsigned timeout_ms)
{
    uint32_t act = 0;
    uint8_t st = libusb20_tr_bulk_intr_sync(xfer, buf, len, &act, timeout_ms);

    if (st == LIBUSB20_TRANSFER_TIMED_OUT) return 0;
    if (st != LIBUSB20_TRANSFER_COMPLETED) {
        errno = EIO;
        return -1;
    }
    return (int)act;
}

int ft_usb_write(ft_usb_t *u, const uint8_t *buf, unsigned len,
                 unsigned timeout_ms)
{
    /* libusb20 writes nothing through this pointer on an OUT transfer. */
    return bulk(u->xout, (void *)(uintptr_t)buf, len, timeout_ms);
}

int ft_usb_read(ft_usb_t *u, uint8_t *buf, unsigned len, unsigned timeout_ms)
{
    if (len > u->pktsz) len = u->pktsz;
    return bulk(u->xin, buf, len, timeout_ms);
}

unsigned ft_usb_packet_size(const ft_usb_t *u)
{
    return u->pktsz;
}

const char *ft_usb_name(const ft_usb_t *u)
{
    return u->name;
}
