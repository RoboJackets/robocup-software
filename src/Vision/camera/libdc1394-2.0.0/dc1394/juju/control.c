/*
 * 1394-Based Digital Camera Control Library
 *
 * Juju backend for dc1394
 * 
 * Written by Kristian Hoegsberg <krh@bitplanet.net>
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
#include <dirent.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/firewire-cdev.h>
#include <inttypes.h>

#include "config.h"
#include "platform.h"
#include <dc1394/dc1394.h>
#include "internal.h"
#include "juju.h"

#define ptr_to_u64(p) ((__u64)(unsigned long)(p))
#define u64_to_ptr(p) ((void *)(unsigned long)(p))


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
    uint32_t config_rom[256];
    char filename[32];
};

platform_device_list_t *
platform_get_device_list (platform_t * p)
{
    DIR * dir;
    struct dirent * de;
    platform_device_list_t * list;
    uint32_t allocated_size = 64;

    list = calloc (1, sizeof (platform_device_list_t));
    if (!list)
        return NULL;
    list->devices = malloc(allocated_size * sizeof(platform_device_t *));
    if (!list->devices) {
        free (list);
        return NULL;
    }

    dir = opendir("/dev");
    if (dir == NULL) {
        dc1394_log_debug("opendir: %m");
        free (list->devices);
        free (list);
        return NULL;
    }

    while ((de = readdir(dir))) {
        char filename[32];
        int fd;
        platform_device_t * device;
        struct fw_cdev_get_info get_info;
        struct fw_cdev_event_bus_reset reset;

        if (strncmp(de->d_name, "fw", 2) != 0)
            continue;

        snprintf(filename, sizeof filename, "/dev/%s", de->d_name);
        fd = open(filename, O_RDWR);
        if (fd < 0) {
            dc1394_log_warning("Failed to open %s: %s", filename, strerror (errno));
            continue;
        }

        device = malloc (sizeof (platform_device_t));
        if (!device) {
            close (fd);
            continue;
        }

        get_info.version = FW_CDEV_VERSION;
        get_info.rom = ptr_to_u64(&device->config_rom);
        get_info.rom_length = 1024;
        get_info.bus_reset = ptr_to_u64(&reset);
        if (ioctl(fd, FW_CDEV_IOC_GET_INFO, &get_info) < 0) {
            dc1394_log_error("GET_CONFIG_ROM failed for %s: %m",filename);
            free (device);
            close(fd);
            continue;
        }
        close (fd);

        strcpy (device->filename, filename);
        list->devices[list->num_devices] = device;
        list->num_devices++;

        if (list->num_devices >= allocated_size) {
            allocated_size += 64;
            list->devices = realloc (list->devices, allocated_size * sizeof (platform_device_t *));
            if (!list->devices)
                return NULL;
        }
    }
    closedir(dir);

    return list;
}

void
platform_free_device_list (platform_device_list_t * d)
{
    int i;
    for (i = 0; i < d->num_devices; i++)
        free (d->devices[i]);
    free (d->devices);
    free (d);
}

int
platform_device_get_config_rom (platform_device_t * device,
                                uint32_t * quads, int * num_quads)
{
    if (*num_quads > 256)
        *num_quads = 256;

    memcpy (quads, device->config_rom, *num_quads * sizeof (uint32_t));
    return 0;
}

platform_camera_t *
platform_camera_new (platform_t * p, platform_device_t * device, uint32_t unit_directory_offset)
{
    int fd;
    platform_camera_t * camera;
    struct fw_cdev_get_info get_info;
    struct fw_cdev_event_bus_reset reset;

    fd = open(device->filename, O_RDWR);
    if (fd < 0) {
        dc1394_log_error("could not open device %s: %m", device->filename);
        return NULL;
    }

    get_info.version = FW_CDEV_VERSION;
    get_info.rom = 0;
    get_info.rom_length = 0;
    get_info.bus_reset = ptr_to_u64(&reset);
    if (ioctl(fd, FW_CDEV_IOC_GET_INFO, &get_info) < 0) {
        dc1394_log_error("IOC_GET_INFO failed for a device %s: %m",device->filename);
        close(fd);
        return NULL;
    }

    camera = calloc (1, sizeof (platform_camera_t));
    camera->fd = fd;
    camera->generation = reset.generation;
    strcpy (camera->filename, device->filename);
    return camera;
}

void platform_camera_free (platform_camera_t * cam)
{
    close (cam->fd);
    free (cam);
}

void
platform_camera_set_parent (platform_camera_t * cam, dc1394camera_t * parent)
{
    cam->camera = parent;
}

void
platform_camera_print_info (platform_camera_t * camera, FILE *fd)
{
    fprintf(fd,"------ Camera platform-specific information ------\n");
    fprintf(fd,"Device filename                   :     %s\n", camera->filename);
}

int
_juju_await_response (platform_camera_t * cam, uint32_t * out, int num_quads)
{
    union {
        struct {
            struct fw_cdev_event_response r;
            __u32 buffer[out ? num_quads : 0];
        } response;
        struct fw_cdev_event_bus_reset reset;
    } u;
    int len, i;

    len = read (cam->fd, &u, sizeof u);
    if (len < 0) {
        dc1394_log_error("failed to read response for %s: %m",cam->filename);
        return -1;
    }

    switch (u.reset.type) {
    case FW_CDEV_EVENT_BUS_RESET:
        cam->generation = u.reset.generation;
        break;

    case FW_CDEV_EVENT_RESPONSE:
#if 0
        printf ("type %d rcode %d length %d", u.response.r.type,
                u.response.r.rcode, u.response.r.length);
#endif
        if (u.response.r.rcode == 4)
            return -2; // retry if we get "resp_conflict_error"
        if (u.response.r.rcode != 0)
            return -1;
        for (i = 0; i < u.response.r.length/4 && i < num_quads && out; i++)
            out[i] = ntohl (u.response.buffer[i]);
        return 1;
    }

    return 0;
}

static dc1394error_t
do_transaction(platform_camera_t * cam, int tcode, uint64_t offset, const uint32_t * in, uint32_t * out, uint32_t num_quads)
{
    struct fw_cdev_send_request request;
    int i, len;
    uint32_t in_buffer[in ? num_quads : 0];
    int retry = 300;

    for (i = 0; in && i < num_quads; i++)
        in_buffer[i] = htonl (in[i]);

    request.offset = CONFIG_ROM_BASE + offset;
    request.data = ptr_to_u64(in_buffer);
    request.length = num_quads * 4;
    request.tcode = tcode;
    request.generation = cam->generation;

    while (retry > 0) {
        int retval;
        len = ioctl (cam->fd, FW_CDEV_IOC_SEND_REQUEST, &request);
        if (len < 0) {
            dc1394_log_error("failed to write request: %m");
            return DC1394_FAILURE;
        }

        while ((retval = _juju_await_response (cam, out, num_quads)) == 0);
        if (retval > 0)
            break;
        if (retval == -1)
            return DC1394_FAILURE;

        /* retry if we get "resp_conflict_error" */
        retry--;
    }

    return DC1394_SUCCESS;
}

dc1394error_t
platform_camera_read (platform_camera_t * cam, uint64_t offset, uint32_t * quads, int num_quads)
{
    int tcode;

    if (num_quads > 1)
        tcode = TCODE_READ_BLOCK_REQUEST;
    else
        tcode = TCODE_READ_QUADLET_REQUEST;

    dc1394error_t err = do_transaction(cam, tcode, offset, NULL, quads, num_quads);
    return err;
}

dc1394error_t
platform_camera_write (platform_camera_t * cam, uint64_t offset, const uint32_t * quads, int num_quads)
{
    int tcode;

    if (num_quads > 1)
        tcode = TCODE_WRITE_BLOCK_REQUEST;
    else
        tcode = TCODE_WRITE_QUADLET_REQUEST;

    return do_transaction(cam, tcode, offset, quads, NULL, num_quads);
}

dc1394error_t
platform_reset_bus (platform_camera_t * cam)
{
    struct fw_cdev_initiate_bus_reset initiate;

    initiate.type = FW_CDEV_SHORT_RESET;
    if (ioctl (cam->fd, FW_CDEV_IOC_INITIATE_BUS_RESET, &initiate) == 0)
        return DC1394_SUCCESS;
    else
        return DC1394_FAILURE;
}

dc1394error_t
platform_read_cycle_timer (platform_camera_t * cam, uint32_t * cycle_timer, uint64_t * local_time)
{
    return DC1394_FUNCTION_NOT_SUPPORTED;
}

dc1394error_t
platform_iso_set_persist (platform_camera_t * cam)
{
    return DC1394_FUNCTION_NOT_SUPPORTED;
}

dc1394error_t
platform_iso_allocate_channel (platform_camera_t * cam, uint64_t channels_allowed, int * channel)
{
    return DC1394_FUNCTION_NOT_SUPPORTED;
}

dc1394error_t
platform_iso_release_channel (platform_camera_t * cam, int channel)
{
    return DC1394_FUNCTION_NOT_SUPPORTED;
}

dc1394error_t
platform_iso_allocate_bandwidth (platform_camera_t * cam, int bandwidth_units)
{
    return DC1394_FUNCTION_NOT_SUPPORTED;
}

dc1394error_t
platform_iso_release_bandwidth (platform_camera_t * cam, int bandwidth_units)
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
