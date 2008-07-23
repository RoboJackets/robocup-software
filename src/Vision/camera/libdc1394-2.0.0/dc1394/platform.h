/*
 * 1394-Based Digital Camera Control Library
 *
 * Platform specific headers
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

#ifndef __DC1394_PLATFORM_H__
#define __DC1394_PLATFORM_H__

#include <stdint.h>
#include <dc1394/dc1394.h>

typedef struct _platform_t platform_t;

platform_t * platform_new (void);
void platform_free (platform_t * p);

typedef struct _platform_device_t platform_device_t;

typedef struct _platform_device_list_t {
    platform_device_t ** devices;
    int num_devices;
} platform_device_list_t;

platform_device_list_t * platform_get_device_list (platform_t * p);
void platform_free_device_list (platform_device_list_t * d);
int platform_device_get_config_rom (platform_device_t * device,
    uint32_t * quads, int * num_quads);

typedef struct _platform_camera_t platform_camera_t;

platform_camera_t * platform_camera_new (platform_t * p,
    platform_device_t * device, uint32_t unit_directory_offset);
void platform_camera_free (platform_camera_t * cam);
void platform_camera_print_info (platform_camera_t * cam, FILE *fd);
void platform_camera_set_parent (platform_camera_t * cam,
        dc1394camera_t * parent);

dc1394error_t platform_camera_read (platform_camera_t * cam, uint64_t offset,
    uint32_t * quads, int num_quads);
dc1394error_t platform_camera_write (platform_camera_t * cam, uint64_t offset,
    const uint32_t * quads, int num_quads);

static inline dc1394error_t
platform_camera_read_quad (platform_camera_t * cam, uint64_t offset,
    uint32_t * quad)
{
    return platform_camera_read (cam, offset, quad, 1);
}

static inline dc1394error_t
platform_camera_write_quad (platform_camera_t * cam, uint64_t offset,
    uint32_t quad)
{
    return platform_camera_write (cam, offset, &quad, 1);
}

dc1394error_t platform_reset_bus (platform_camera_t * cam);
dc1394error_t platform_read_cycle_timer (platform_camera_t * cam,
        uint32_t * cycle_timer, uint64_t * local_time);

dc1394error_t platform_capture_setup (platform_camera_t *cam,
        uint32_t num_dma_buffers, uint32_t flags);
dc1394error_t platform_capture_stop (platform_camera_t *cam);

dc1394error_t platform_capture_dequeue (platform_camera_t * cam,
                        dc1394capture_policy_t policy,
                        dc1394video_frame_t **frame_return);
dc1394error_t platform_capture_enqueue (platform_camera_t * cam,
                        dc1394video_frame_t * frame);
int platform_capture_get_fileno (platform_camera_t * cam);

dc1394error_t platform_iso_set_persist (platform_camera_t * cam);
dc1394error_t platform_iso_allocate_channel (platform_camera_t * cam,
        uint64_t channels_allowed, int * channel);
dc1394error_t platform_iso_release_channel (platform_camera_t * cam,
    int channel);
dc1394error_t platform_iso_allocate_bandwidth (platform_camera_t * cam,
    int bandwidth_units);
dc1394error_t platform_iso_release_bandwidth (platform_camera_t * cam,
    int bandwidth_units);

dc1394error_t platform_set_broadcast(platform_camera_t * craw,
                                          dc1394bool_t pwr);
dc1394error_t platform_get_broadcast(platform_camera_t * craw,
                                          dc1394bool_t *pwr);

#endif
