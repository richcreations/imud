/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * ft_usb_linux.c — include/ft_usb.h over usbfs.
 *
 * The second file in the tree that includes a Linux kernel header, and for the
 * same reason as src/bus_linux.c: the transfers are ioctls.  No libusb and no
 * libftdi, which keeps the FT232H backend free of a dependency the rest of the
 * tree does not have — usbfs is what libusb itself drives on Linux.
 *
 * It needs no privilege.  A seat-local FT232H carries an ACL granting the
 * logged-in user rw on its /dev/bus/usb node (systemd's uaccess), and every
 * operation here — including detaching the kernel's serial driver — is
 * governed by that file mode rather than by a capability.
 */

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <linux/usbdevice_fs.h>
#include <sys/ioctl.h>

#include "ft_usb.h"
#include "log.h"

#define SYSFS_USB "/sys/bus/usb/devices"

struct ft_usb {
    int      fd;
    unsigned ifno;
    uint8_t  ep_in, ep_out;
    unsigned pktsz;
    int      claimed;
    char     name[352];
};

/* One small sysfs attribute, newline stripped.  Returns 0 or -1. */
static int sysfs_str(const char *dir, const char *attr, char *out, size_t n)
{
    char path[768];
    snprintf(path, sizeof path, "%s/%s/%s", SYSFS_USB, dir, attr);

    FILE *f = fopen(path, "r");
    if (!f) return -1;
    char *got = fgets(out, (int)n, f);
    fclose(f);
    if (!got) return -1;

    out[strcspn(out, "\r\n")] = '\0';
    return 0;
}

static long sysfs_num(const char *dir, const char *attr, int base)
{
    char buf[64];
    if (sysfs_str(dir, attr, buf, sizeof buf) < 0) return -1;
    return strtol(buf, NULL, base);
}

/*
 * The IN endpoint's wMaxPacketSize, read from the interface's endpoint
 * directory.  Falls back to the high-speed 512 rather than failing: a wrong
 * bound here costs a split read, not a wrong one.
 */
static unsigned ep_packet_size(const char *dir, unsigned ifno, uint8_t ep)
{
    char path[640];
    snprintf(path, sizeof path, "%s/%s:1.%u/ep_%02x/wMaxPacketSize",
             SYSFS_USB, dir, ifno, ep);

    FILE *f = fopen(path, "r");
    if (!f) return 512;
    unsigned v = 0;
    if (fscanf(f, "%x", &v) != 1 || v == 0) v = 512;
    fclose(f);
    return v;
}

/* Does this sysfs device match what the operator asked for? */
static int matches(const char *dir, const char *want, long busnum, long devnum)
{
    if (!want || want[0] == '\0') return 1;

    char serial[128];
    if (sysfs_str(dir, "serial", serial, sizeof serial) == 0 &&
        strcmp(serial, want) == 0)
        return 1;

    char topo[48];
    snprintf(topo, sizeof topo, "%ld.%ld", busnum, devnum);
    if (strcmp(topo, want) == 0) return 1;

    /* The sysfs port path ("1-2"), which survives a replug into the same
     * socket where bus.dev does not. */
    return strcmp(dir, want) == 0;
}

/*
 * Find the node for the first matching device.  Interface directories carry a
 * ':' in their name and are skipped; only the device directories have
 * idVendor.
 */
static int find_node(const char *want, uint16_t vid, uint16_t pid,
                     char *node, size_t nodesz, char *dir, size_t dirsz)
{
    DIR *d = opendir(SYSFS_USB);
    if (!d) return -1;

    int found = 0;
    struct dirent *e;
    while (!found && (e = readdir(d)) != NULL) {
        if (e->d_name[0] == '.' || strchr(e->d_name, ':')) continue;

        if (sysfs_num(e->d_name, "idVendor", 16)  != (long)vid) continue;
        if (sysfs_num(e->d_name, "idProduct", 16) != (long)pid) continue;

        long bus = sysfs_num(e->d_name, "busnum", 10);
        long dev = sysfs_num(e->d_name, "devnum", 10);
        if (bus < 0 || dev < 0) continue;

        if (!matches(e->d_name, want, bus, dev)) continue;

        snprintf(node, nodesz, "/dev/bus/usb/%03ld/%03ld", bus, dev);
        snprintf(dir,  dirsz,  "%s", e->d_name);
        found = 1;
    }

    closedir(d);
    if (!found) errno = ENODEV;
    return found ? 0 : -1;
}

/*
 * Take the interface from whatever kernel driver holds it and claim it.
 * DISCONNECT_CLAIM does both in one step; the two-ioctl form is the fallback
 * for a kernel too old to have it.
 */
static int claim(int fd, unsigned ifno)
{
    struct usbdevfs_disconnect_claim dc;
    memset(&dc, 0, sizeof dc);
    dc.interface = ifno;
    dc.flags     = 0;              /* disconnect whatever is bound, then claim */
    if (ioctl(fd, USBDEVFS_DISCONNECT_CLAIM, &dc) == 0) return 0;
    if (errno != ENOTTY && errno != EINVAL) return -1;

    struct usbdevfs_ioctl c = {
        .ifno = (int)ifno, .ioctl_code = USBDEVFS_DISCONNECT, .data = NULL,
    };
    if (ioctl(fd, USBDEVFS_IOCTL, &c) < 0 && errno != ENODATA) return -1;

    unsigned i = ifno;
    return ioctl(fd, USBDEVFS_CLAIMINTERFACE, &i);
}

ft_usb_t *ft_usb_open(const char *want, uint16_t vid, uint16_t pid,
                      unsigned ifno)
{
    char node[64], dir[256];
    if (find_node(want ? want : "", vid, pid, node, sizeof node,
                  dir, sizeof dir) < 0) {
        LOG_E("ft232h: no %04x:%04x%s%s found\n", vid, pid,
              (want && *want) ? " matching " : "", (want && *want) ? want : "");
        return NULL;
    }

    ft_usb_t *u = calloc(1, sizeof *u);
    if (!u) return NULL;

    u->fd = open(node, O_RDWR | O_CLOEXEC);
    if (u->fd < 0) {
        LOG_E("ft232h: cannot open %s: %s\n", node, strerror(errno));
        free(u);
        return NULL;
    }

    if (claim(u->fd, ifno) < 0) {
        LOG_E("ft232h: cannot claim %s interface %u: %s\n",
              node, ifno, strerror(errno));
        close(u->fd);
        free(u);
        return NULL;
    }

    u->ifno    = ifno;
    u->claimed = 1;
    /* FTDI numbers its interfaces' endpoints in pairs from 0x02/0x81; the
     * FT232H has only interface A. */
    u->ep_out  = (uint8_t)(0x02 + ifno * 2);
    u->ep_in   = (uint8_t)(0x81 + ifno * 2);
    u->pktsz   = ep_packet_size(dir, ifno, u->ep_in);
    snprintf(u->name, sizeof u->name, "%s (%s)", node, dir);

    LOG_I("ft232h: %s, %u-byte packets\n", u->name, u->pktsz);
    return u;
}

void ft_usb_close(ft_usb_t *u)
{
    if (!u) return;

    if (u->claimed) {
        unsigned i = u->ifno;
        (void)ioctl(u->fd, USBDEVFS_RELEASEINTERFACE, &i);

        /* Hand the interface back, so unplugging is not the only way to get a
         * serial port again. */
        struct usbdevfs_ioctl c = {
            .ifno = (int)u->ifno, .ioctl_code = USBDEVFS_CONNECT, .data = NULL,
        };
        (void)ioctl(u->fd, USBDEVFS_IOCTL, &c);
    }

    if (u->fd >= 0) close(u->fd);
    free(u);
}

int ft_usb_control(ft_usb_t *u, uint8_t request, uint16_t value)
{
    struct usbdevfs_ctrltransfer c;
    memset(&c, 0, sizeof c);
    c.bRequestType = 0x40;                  /* vendor, host to device */
    c.bRequest     = request;
    c.wValue       = value;
    c.wIndex       = (uint16_t)(u->ifno + 1);  /* FTDI indexes A as 1 */
    c.wLength      = 0;
    c.timeout      = 1000;
    c.data         = NULL;

    return ioctl(u->fd, USBDEVFS_CONTROL, &c) < 0 ? -1 : 0;
}

static int bulk(ft_usb_t *u, uint8_t ep, void *buf, unsigned len,
                unsigned timeout_ms)
{
    struct usbdevfs_bulktransfer b;
    memset(&b, 0, sizeof b);
    b.ep      = ep;
    b.len     = len;
    b.timeout = timeout_ms;
    b.data    = buf;

    int n = ioctl(u->fd, USBDEVFS_BULK, &b);
    if (n < 0 && errno == ETIMEDOUT) return 0;
    return n;
}

int ft_usb_write(ft_usb_t *u, const uint8_t *buf, unsigned len,
                 unsigned timeout_ms)
{
    /* usbfs writes nothing through this pointer on an OUT transfer. */
    return bulk(u, u->ep_out, (void *)(uintptr_t)buf, len, timeout_ms);
}

int ft_usb_read(ft_usb_t *u, uint8_t *buf, unsigned len, unsigned timeout_ms)
{
    if (len > u->pktsz) len = u->pktsz;
    return bulk(u, u->ep_in, buf, len, timeout_ms);
}

unsigned ft_usb_packet_size(const ft_usb_t *u)
{
    return u->pktsz;
}

const char *ft_usb_name(const ft_usb_t *u)
{
    return u->name;
}
