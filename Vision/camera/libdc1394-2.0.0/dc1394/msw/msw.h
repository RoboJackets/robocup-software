/*
 * 1394-Based Digital Camera Control Library
 * 
 * MS Windows Support Code
 * 
 * Written by Vladimir Avdonin <vldmr@users.sf.net>
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

#ifndef __DC1394_MSW_H__
#define __DC1394_MSW_H__

#include <windows.h>
#include <1394.h>

#define DC1394_CAST_CAMERA_TO_MSW(cammsw, camera) dc1394camera_msw_t * cammsw = (dc1394camera_msw_t *) camera

typedef struct __dc1394_camera_msw
{
        dc1394camera_t camera;
        int port;
        SELF_ID selfid;
        HANDLE bw_handle;
        msw1394_ISO ISO;
        dc1394video_frame_t *frames;
} dc1394camera_msw_t;

#endif

