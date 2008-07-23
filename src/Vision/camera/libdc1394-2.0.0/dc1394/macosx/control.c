/*
 * 1394-Based Digital Camera Control Library
 *
 * Mac OS X Digital Camera Control Code
 *
 * Written by David Moore <dcm@acm.org>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <stdio.h>
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOCFPlugIn.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/firewire/IOFireWireLib.h>

#include "config.h"
#include "platform.h"
#include "internal.h"
#include "macosx.h"
#include <dc1394/dc1394.h>

platform_t *
platform_new (void)
{
    platform_t * p = calloc (1, sizeof (platform_t));
    return p;
}
void
platform_free (platform_t * p)
{
    free (p);
}

struct _platform_device_t {
    io_object_t node;
};

platform_device_list_t *
platform_get_device_list (platform_t * p)
{
    platform_device_list_t * list;
    uint32_t allocated_size = 64;
    kern_return_t res;
    mach_port_t master_port;
    io_iterator_t iterator;
    io_object_t node;
    CFMutableDictionaryRef dict;

    list = calloc (1, sizeof (platform_device_list_t));
    if (!list)
        return NULL;
    list->devices = malloc(allocated_size * sizeof(platform_device_t *));
    if (!list->devices) {
        free (list);
        return NULL;
    }

    res = IOMasterPort (MACH_PORT_NULL, &master_port);
    if (res != KERN_SUCCESS)
        return NULL;

    dict = IOServiceMatching ("IOFireWireDevice");
    if (!dict)
        return NULL;

    res = IOServiceGetMatchingServices (master_port, dict, &iterator);

    while ((node = IOIteratorNext (iterator))) {
        platform_device_t * device = malloc (sizeof (platform_device_t));
        if (!device) {
            IOObjectRelease (node);
            continue;
        }

        device->node = node;
        list->devices[list->num_devices] = device;
        list->num_devices++;

        if (list->num_devices >= allocated_size) {
            allocated_size += 64;
            list->devices = realloc (list->devices,
                                     allocated_size * sizeof (platform_device_t *));
            if (!list->devices)
                return NULL;
        }
    }
    IOObjectRelease (iterator);

    return list;
}

void
platform_free_device_list (platform_device_list_t * d)
{
    int i;
    for (i = 0; i < d->num_devices; i++) {
        IOObjectRelease (d->devices[i]->node);
        free (d->devices[i]);
    }
    free (d->devices);
    free (d);
}

int
platform_device_get_config_rom (platform_device_t * device,
    uint32_t * quads, int * num_quads)
{
    CFTypeRef prop;
    prop = IORegistryEntryCreateCFProperty (device->node,
                                            CFSTR ("FireWire Device ROM"), kCFAllocatorDefault, 0);
    if (!prop)
        return -1;
    CFDataRef data = CFDictionaryGetValue (prop, CFSTR ("Offset 0"));
    if (!data) {
        CFRelease (prop);
        return -1;
    }

    int nquads = CFDataGetLength (data) / 4;
    if (*num_quads > nquads)
        *num_quads = nquads;
    const uint8_t * d = CFDataGetBytePtr (data);
    int i;
    for (i = 0; i < *num_quads; i++)
        quads[i] = (d[4*i] << 24) | (d[4*i+1] << 16) | (d[4*i+2] << 8) | d[4*i+3];

    CFRelease (prop);
    return 0;
}

platform_camera_t *
platform_camera_new (platform_t * p, platform_device_t * device,
    uint32_t unit_directory_offset)
{
    kern_return_t res;
    platform_camera_t * camera;
    IOCFPlugInInterface ** plugin_interface = NULL;
    SInt32 score;
    IOFireWireLibDeviceRef iface = NULL;

    res = IOCreatePlugInInterfaceForService (device->node, kIOFireWireLibTypeID,
                                             kIOCFPlugInInterfaceID, &plugin_interface, &score);
    if (res != KERN_SUCCESS) {
        dc1394_log_error("Failed to get plugin interface");
        return NULL;
    }

    /* TODO: error check here */
    (*plugin_interface)->QueryInterface (plugin_interface,
                                         CFUUIDGetUUIDBytes (kIOFireWireDeviceInterfaceID),
                                       (void**) &iface);
    IODestroyPlugInInterface (plugin_interface);

    res = (*iface)->Open (iface);
    if (res != kIOReturnSuccess) {
        (*iface)->Release (iface);
        return NULL;
    }

    camera = calloc (1, sizeof (platform_camera_t));
    camera->iface = iface;
    (*camera->iface)->GetBusGeneration (camera->iface,
                                        &(camera->generation));
    return camera;
}

void platform_camera_free (platform_camera_t * cam)
{
    (*cam->iface)->Close (cam->iface);
    (*cam->iface)->Release (cam->iface);
    free (cam);
}

void
platform_camera_set_parent (platform_camera_t * cam,
        dc1394camera_t * parent)
{
    cam->camera = parent;
}

void
platform_camera_print_info (platform_camera_t * camera, FILE *fd)
{
    fprintf(fd,"------ Camera platform-specific information ------\n");
    fprintf(fd,"Interface                       :     %p\n", camera->iface);
    fprintf(fd,"Generation                      :     %lu\n", camera->generation);
}

dc1394error_t
platform_camera_read (platform_camera_t * cam, uint64_t offset,
    uint32_t * quads, int num_quads)
{
    IOFireWireLibDeviceRef d = cam->iface;
    FWAddress full_addr;
    int i, retval;
    UInt32 length;
    UInt64 addr = CONFIG_ROM_BASE + offset;

    full_addr.addressHi = addr >> 32;
    full_addr.addressLo = addr & 0xffffffff;

    length = 4 * num_quads;
    if (num_quads > 1)
        retval = (*d)->Read (d, (*d)->GetDevice (d), &full_addr, quads, &length,
                             false, cam->generation);
    else
        retval = (*d)->ReadQuadlet (d, (*d)->GetDevice (d), &full_addr,
                                    (UInt32 *) quads, false, cam->generation);
    if (retval != 0) {
        dc1394_log_error("Error reading (%x)...",retval);
        return DC1394_FAILURE;
    }
    for (i = 0; i < num_quads; i++)
        quads[i] = ntohl (quads[i]);
    return DC1394_SUCCESS;
}

dc1394error_t
platform_camera_write (platform_camera_t * cam, uint64_t offset,
    const uint32_t * quads, int num_quads)
{
    IOFireWireLibDeviceRef d = cam->iface;
    FWAddress full_addr;
    int i, retval;
    UInt32 length;
    UInt64 addr = CONFIG_ROM_BASE + offset;
    uint32_t values[num_quads];

    full_addr.addressHi = addr >> 32;
    full_addr.addressLo = addr & 0xffffffff;

    for (i = 0; i < num_quads; i++)
        values[i] = htonl (quads[i]);

    length = 4 * num_quads;
    if (num_quads > 1)
        retval = (*d)->Write (d, (*d)->GetDevice (d), &full_addr, values, &length,
                              false, cam->generation);
    else
        retval = (*d)->WriteQuadlet (d, (*d)->GetDevice (d), &full_addr, values[0],
                                     false, cam->generation);
    if (retval != 0) {
        dc1394_log_error("Error writing (%x)...",retval);
        return DC1394_FAILURE;
    }
    return DC1394_SUCCESS;
}

dc1394error_t
platform_reset_bus (platform_camera_t * cam)
{
    IOFireWireLibDeviceRef d = cam->iface;

    if ((*d)->BusReset (d) == 0)
        return DC1394_SUCCESS;
    else
        return DC1394_FAILURE;
}

dc1394error_t
platform_read_cycle_timer (platform_camera_t * cam,
        uint32_t * cycle_timer, uint64_t * local_time)
{
    IOFireWireLibDeviceRef d = cam->iface;
    struct timeval tv;

    if ((*d)->GetCycleTime (d, (UInt32 *) cycle_timer) != 0)
        return DC1394_FAILURE;

    gettimeofday (&tv, NULL);
    *local_time = (uint64_t)tv.tv_sec * 1000000ULL + tv.tv_usec;
    return DC1394_SUCCESS;
}

dc1394error_t
platform_iso_set_persist (platform_camera_t * cam)
{
    return DC1394_FUNCTION_NOT_SUPPORTED;
}

dc1394error_t
platform_iso_allocate_channel (platform_camera_t * cam,
        uint64_t channels_allowed, int * channel)
{
    return DC1394_FUNCTION_NOT_SUPPORTED;
}

dc1394error_t
platform_iso_release_channel (platform_camera_t * cam,
    int channel)
{
    return DC1394_FUNCTION_NOT_SUPPORTED;
}

dc1394error_t
platform_iso_allocate_bandwidth (platform_camera_t * cam,
    int bandwidth_units)
{
    return DC1394_FUNCTION_NOT_SUPPORTED;
}

dc1394error_t
platform_iso_release_bandwidth (platform_camera_t * cam,
    int bandwidth_units)
{
    return DC1394_FUNCTION_NOT_SUPPORTED;
}

dc1394error_t
platform_set_broadcast(platform_camera_t * craw, dc1394bool_t pwr)
{
    return DC1394_FUNCTION_NOT_SUPPORTED;
}


dc1394error_t
platform_get_broadcast(platform_camera_t * craw, dc1394bool_t *pwr)
{
    return DC1394_FUNCTION_NOT_SUPPORTED;
}
