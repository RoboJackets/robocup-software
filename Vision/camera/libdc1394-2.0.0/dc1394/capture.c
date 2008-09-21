/*
 * 1394-Based Digital Camera Control Library
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

#include "control.h"
#include "platform.h"
#include "internal.h"

dc1394error_t
dc1394_capture_setup (dc1394camera_t *camera, uint32_t num_dma_buffers,
        uint32_t flags)
{
    dc1394camera_priv_t * cpriv = DC1394_CAMERA_PRIV (camera);
    return platform_capture_setup (cpriv->pcam, num_dma_buffers, flags);
}

dc1394error_t
dc1394_capture_stop (dc1394camera_t *camera)
{
    dc1394camera_priv_t * cpriv = DC1394_CAMERA_PRIV (camera);
    return platform_capture_stop (cpriv->pcam);
}

int
dc1394_capture_get_fileno (dc1394camera_t * camera)
{
    dc1394camera_priv_t * cpriv = DC1394_CAMERA_PRIV (camera);
    return platform_capture_get_fileno (cpriv->pcam);
}

dc1394error_t
dc1394_capture_dequeue (dc1394camera_t * camera, dc1394capture_policy_t policy,
        dc1394video_frame_t **frame)
{
    dc1394camera_priv_t * cpriv = DC1394_CAMERA_PRIV (camera);
    return platform_capture_dequeue (cpriv->pcam, policy, frame);
}

dc1394error_t
dc1394_capture_enqueue (dc1394camera_t * camera, dc1394video_frame_t * frame)
{
    dc1394camera_priv_t * cpriv = DC1394_CAMERA_PRIV (camera);
    return platform_capture_enqueue (cpriv->pcam, frame);
}

