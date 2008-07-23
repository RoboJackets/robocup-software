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

#ifndef __DC1394_JUJU_H__
#define __DC1394_JUJU_H__

#include <linux/firewire-cdev.h>
#include "config.h"
#include "internal.h"
#include "register.h"
#include "offsets.h"
#include <dc1394/dc1394.h>

struct _platform_t {
    int dummy;
};

struct _platform_camera_t {
    int fd;
    char filename[32];
    int generation;

    dc1394camera_t * camera;

    int iso_fd;
    int iso_handle;
    struct juju_frame        * frames;
    unsigned char        * buffer;
    size_t buffer_size;
    uint32_t flags;
    unsigned int num_frames;
    int current;
    int ready_frames;

    unsigned int iso_channel;
    int capture_is_set;
    int iso_auto_started;
};


struct juju_frame {
    dc1394video_frame_t                 frame;
    size_t                         size;
    struct fw_cdev_iso_packet        *packets;
};

dc1394error_t
_juju_iterate(dc1394camera_t *camera);

#endif
