/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * ft_usb_darwin.c — include/ft_usb.h over IOKit.
 *
 * The macOS answer to what src/ft_usb_linux.c does with usbfs, and chosen the
 * same way: IOUSBLib is a system framework, so the FT232H backend stays free
 * of a dependency the rest of the tree does not have.  libusb would be less
 * code and would serve the BSDs too, but it is not on a stock Mac, and a
 * backend nobody can build without Homebrew first is not much of a backend.
 *
 * It needs no privilege and no kext unloading.  A stock macOS binds
 * AppleUSBFTDI to the part and publishes /dev/cu.usbserial-*, but that driver
 * does not hold the interface exclusively while nothing has the port open, so
 * USBDeviceOpen and USBInterfaceOpen both succeed as the logged-in user
 * (measured on macOS 14.8.9 with the serial node present).  The Seize
 * fallbacks below are for the case where something does hold it — another
 * process with the port open, or a macOS that binds it harder.
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOCFPlugIn.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/usb/IOUSBLib.h>

#include "ft_usb.h"
#include "log.h"

struct ft_usb {
    IOUSBDeviceInterface500    **dev;
    IOUSBInterfaceInterface500 **intf;
    unsigned ifno;
    UInt8    pipe_in, pipe_out;   /* pipe REFS, not endpoint addresses */
    unsigned pktsz;
    char     name[128];
};

/* A CFString property off the device, as UTF-8.  Empty when absent. */
static void io_string(io_service_t svc, CFStringRef key, char *out, size_t n)
{
    out[0] = '\0';
    CFTypeRef v = IORegistryEntryCreateCFProperty(svc, key, NULL, 0);
    if (!v) return;
    if (CFGetTypeID(v) == CFStringGetTypeID())
        CFStringGetCString(v, out, (CFIndex)n, kCFStringEncodingUTF8);
    CFRelease(v);
}

static uint32_t io_number(io_service_t svc, CFStringRef key)
{
    uint32_t got = 0;
    CFTypeRef v = IORegistryEntryCreateCFProperty(svc, key, NULL, 0);
    if (!v) return 0;
    if (CFGetTypeID(v) == CFNumberGetTypeID())
        CFNumberGetValue(v, kCFNumberSInt32Type, &got);
    CFRelease(v);
    return got;
}

/*
 * Does this device match what the operator asked for?  The serial string is
 * the portable half; the location ID stands in for the Linux port path, being
 * the identifier that follows the physical socket rather than the plug order
 * — and it is what macOS itself puts in /dev/cu.usbserial-NNNN.
 */
static int matches(io_service_t svc, const char *want)
{
    if (!want || want[0] == '\0') return 1;

    char serial[128];
    io_string(svc, CFSTR(kUSBSerialNumberString), serial, sizeof serial);
    if (serial[0] && strcmp(serial, want) == 0) return 1;

    uint32_t loc = io_number(svc, CFSTR(kUSBDevicePropertyLocationID));
    char locstr[16];
    snprintf(locstr, sizeof locstr, "%x", loc);
    return strcmp(locstr, want) == 0;
}

/* The first device matching vid/pid and `want`.  Caller releases. */
static io_service_t find_device(const char *want, uint16_t vid, uint16_t pid,
                                char *name, size_t namesz)
{
    CFMutableDictionaryRef m = IOServiceMatching(kIOUSBDeviceClassName);
    if (!m) return IO_OBJECT_NULL;

    int v = vid, p = pid;
    CFNumberRef nv = CFNumberCreate(NULL, kCFNumberIntType, &v);
    CFNumberRef np = CFNumberCreate(NULL, kCFNumberIntType, &p);
    CFDictionarySetValue(m, CFSTR(kUSBVendorID), nv);
    CFDictionarySetValue(m, CFSTR(kUSBProductID), np);
    CFRelease(nv);
    CFRelease(np);

    io_iterator_t it;
    if (IOServiceGetMatchingServices(kIOMainPortDefault, m, &it) != KERN_SUCCESS)
        return IO_OBJECT_NULL;

    io_service_t svc, found = IO_OBJECT_NULL;
    while ((svc = IOIteratorNext(it)) != IO_OBJECT_NULL) {
        if (!found && matches(svc, want)) {
            found = svc;
            snprintf(name, namesz, "%04x:%04x @ %x", vid, pid,
                     io_number(svc, CFSTR(kUSBDevicePropertyLocationID)));
            continue;   /* keep it; release the rest */
        }
        IOObjectRelease(svc);
    }
    IOObjectRelease(it);
    return found;
}

/* An IOKit COM interface off a service.  Returns NULL on failure. */
static void *plugin_interface(io_service_t svc, CFUUIDRef type, CFUUIDRef iface)
{
    IOCFPlugInInterface **plug = NULL;
    SInt32 score = 0;
    if (IOCreatePlugInInterfaceForService(svc, type, kIOCFPlugInInterfaceID,
                                          &plug, &score) != KERN_SUCCESS || !plug)
        return NULL;

    void *out = NULL;
    (*plug)->QueryInterface(plug, CFUUIDGetUUIDBytes(iface), (LPVOID *)&out);
    (*plug)->Release(plug);
    return out;
}

static int open_interface(ft_usb_t *u, unsigned ifno)
{
    IOUSBFindInterfaceRequest req = {
        .bInterfaceClass    = kIOUSBFindInterfaceDontCare,
        .bInterfaceSubClass = kIOUSBFindInterfaceDontCare,
        .bInterfaceProtocol = kIOUSBFindInterfaceDontCare,
        .bAlternateSetting  = kIOUSBFindInterfaceDontCare,
    };
    io_iterator_t it;
    if ((*u->dev)->CreateInterfaceIterator(u->dev, &req, &it) != kIOReturnSuccess)
        return -1;

    /* FTDI numbers its interfaces from zero in enumeration order; the FT232H
     * has only interface A. */
    io_service_t svc = IO_OBJECT_NULL;
    for (unsigned i = 0; i <= ifno; i++) {
        if (svc) IOObjectRelease(svc);
        svc = IOIteratorNext(it);
        if (!svc) break;
    }
    IOObjectRelease(it);
    if (!svc) { errno = ENODEV; return -1; }

    u->intf = plugin_interface(svc, kIOUSBInterfaceUserClientTypeID,
                               kIOUSBInterfaceInterfaceID500);
    IOObjectRelease(svc);
    if (!u->intf) { errno = ENODEV; return -1; }

    kern_return_t kr = (*u->intf)->USBInterfaceOpen(u->intf);
    if (kr == kIOReturnExclusiveAccess)
        kr = (*u->intf)->USBInterfaceOpenSeize(u->intf);
    if (kr != kIOReturnSuccess) {
        LOG_E("ft232h: cannot claim interface %u (0x%x) — something else has "
              "the port open\n", ifno, kr);
        errno = EBUSY;
        return -1;
    }

    /* Map endpoint addresses to pipe refs: IOKit addresses pipes by index
     * within the interface, not by the endpoint number on the wire. */
    UInt8 n = 0;
    (*u->intf)->GetNumEndpoints(u->intf, &n);
    for (UInt8 i = 1; i <= n; i++) {
        UInt8 dir, num, tt, iv;
        UInt16 mps;
        if ((*u->intf)->GetPipeProperties(u->intf, i, &dir, &num, &tt, &mps, &iv)
            != kIOReturnSuccess)
            continue;
        if (tt != kUSBBulk) continue;
        if (dir == kUSBIn  && !u->pipe_in)  { u->pipe_in  = i; u->pktsz = mps; }
        if (dir == kUSBOut && !u->pipe_out) { u->pipe_out = i; }
    }
    if (!u->pipe_in || !u->pipe_out) {
        LOG_E("ft232h: interface %u has no bulk in/out pair\n", ifno);
        errno = ENODEV;
        return -1;
    }
    if (!u->pktsz) u->pktsz = 512;
    u->ifno = ifno;
    return 0;
}

ft_usb_t *ft_usb_open(const char *want, uint16_t vid, uint16_t pid,
                      unsigned ifno)
{
    ft_usb_t *u = calloc(1, sizeof *u);
    if (!u) return NULL;

    io_service_t svc = find_device(want ? want : "", vid, pid,
                                   u->name, sizeof u->name);
    if (!svc) {
        LOG_E("ft232h: no %04x:%04x%s%s found\n", vid, pid,
              (want && *want) ? " matching " : "", (want && *want) ? want : "");
        free(u);
        errno = ENODEV;
        return NULL;
    }

    u->dev = plugin_interface(svc, kIOUSBDeviceUserClientTypeID,
                              kIOUSBDeviceInterfaceID500);
    IOObjectRelease(svc);
    if (!u->dev) {
        LOG_E("ft232h: cannot reach %s through IOKit\n", u->name);
        free(u);
        errno = ENODEV;
        return NULL;
    }

    kern_return_t kr = (*u->dev)->USBDeviceOpen(u->dev);
    if (kr == kIOReturnExclusiveAccess)
        kr = (*u->dev)->USBDeviceOpenSeize(u->dev);
    if (kr != kIOReturnSuccess) {
        LOG_E("ft232h: cannot open %s (0x%x)\n", u->name, kr);
        (*u->dev)->Release(u->dev);
        free(u);
        errno = EBUSY;
        return NULL;
    }

    /* A device arrives unconfigured to a userspace client that seized it. */
    IOUSBConfigurationDescriptorPtr cd = NULL;
    if ((*u->dev)->GetConfigurationDescriptorPtr(u->dev, 0, &cd) == kIOReturnSuccess
        && cd)
        (*u->dev)->SetConfiguration(u->dev, cd->bConfigurationValue);

    if (open_interface(u, ifno) < 0) {
        if (u->intf) (*u->intf)->Release(u->intf);
        (*u->dev)->USBDeviceClose(u->dev);
        (*u->dev)->Release(u->dev);
        free(u);
        return NULL;
    }

    LOG_I("ft232h: %s, %u-byte packets\n", u->name, u->pktsz);
    return u;
}

void ft_usb_close(ft_usb_t *u)
{
    if (!u) return;

    if (u->intf) {
        (*u->intf)->USBInterfaceClose(u->intf);
        (*u->intf)->Release(u->intf);
    }
    if (u->dev) {
        /* Closing hands the part back to AppleUSBFTDI, so /dev/cu.usbserial-*
         * works again without a replug. */
        (*u->dev)->USBDeviceClose(u->dev);
        (*u->dev)->Release(u->dev);
    }
    free(u);
}

int ft_usb_control(ft_usb_t *u, uint8_t request, uint16_t value)
{
    IOUSBDevRequest r = {
        .bmRequestType = USBmakebmRequestType(kUSBOut, kUSBVendor, kUSBDevice),
        .bRequest      = request,
        .wValue        = value,
        .wIndex        = (UInt16)(u->ifno + 1),   /* FTDI indexes A as 1 */
        .wLength       = 0,
        .pData         = NULL,
    };
    if ((*u->dev)->DeviceRequest(u->dev, &r) != kIOReturnSuccess) {
        errno = EIO;
        return -1;
    }
    return 0;
}

int ft_usb_write(ft_usb_t *u, const uint8_t *buf, unsigned len,
                 unsigned timeout_ms)
{
    /* WritePipeTO does not write through this pointer. */
    kern_return_t kr = (*u->intf)->WritePipeTO(u->intf, u->pipe_out,
                                               (void *)(uintptr_t)buf, len,
                                               timeout_ms, timeout_ms);
    if (kr != kIOReturnSuccess) { errno = EIO; return -1; }
    return (int)len;
}

int ft_usb_read(ft_usb_t *u, uint8_t *buf, unsigned len, unsigned timeout_ms)
{
    if (len > u->pktsz) len = u->pktsz;

    UInt32 n = len;
    kern_return_t kr = (*u->intf)->ReadPipeTO(u->intf, u->pipe_in, buf, &n,
                                              timeout_ms, timeout_ms);
    /* A read with nothing queued is the normal case, not a failure — the
     * caller retries.  Every other status is real. */
    if (kr == kIOUSBTransactionTimeout) return 0;
    if (kr != kIOReturnSuccess) { errno = EIO; return -1; }
    return (int)n;
}

unsigned ft_usb_packet_size(const ft_usb_t *u) { return u->pktsz; }

const char *ft_usb_name(const ft_usb_t *u) { return u->name; }
