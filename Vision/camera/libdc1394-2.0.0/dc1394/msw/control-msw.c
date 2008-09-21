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

#include <windows.h>

#include "config.h"
#include "internal.h"
#include "register.h"
#include "offsets.h"
#include <dc1394/dc1394.h>
#include "msw1394.h"

#define DC1394_CAST_CAMERA_TO_MSW(cammsw, camera) dc1394camera_msw_t * cammsw = (dc1394camera_msw_t *) camera

int usleep(long usec) {
    Sleep(usec/1000);
    return 0;
}

typedef struct __dc1394_camera_msw
{
    dc1394camera_t camera;
    sSELF_ID selfid;
    HANDLE bw_handle;
    msw1394_ISO ISO;
    dc1394video_frame_t *frames;
    uint32_t  capture_flags;
} dc1394camera_msw_t;

static dc1394error_t  ConvertError(msw1394error_t iErr1394)
{
    dc1394error_t res = DC1394_SUCCESS;
    switch (iErr1394) {
    case MSW1394_SUCCESS:
        res = DC1394_SUCCESS;
        break;
    case MSW1394_NO_SYSTEM_RESOURCES:
        res = DC1394_MEMORY_ALLOCATION_FAILURE;
        break;
    default:
        res = DC1394_RAW1394_FAILURE;
        break;
    }
    return res;
}

static msw1394_speed_t dc1394_2_msw1394_speed(dc1394speed_t dc1394_speed)
{
    msw1394_speed_t res = MSW1394_SPEED400;
    switch (dc1394_speed) {
    case DC1394_ISO_SPEED_100:
        res = MSW1394_SPEED100;
        break;
    case DC1394_ISO_SPEED_200:
        res = MSW1394_SPEED200;
        break;
    default:
        break;
    }
    return res;
}

static msw1394error_t GrabSelfIds(dc1394camera_t **cams, int ncams)
{
    msw1394error_t res = MSW1394_SUCCESS;
    int i, port,k;
    dc1394camera_msw_t* cam;
    ULONG port_num;
    res = msw1394_GetNumPorts(&port_num);
    if (res != MSW1394_SUCCESS)
        return res;
    for (port=0; port<port_num; port++) {
        sTOPOLOGY_MAP *top;
        res = msw1394_GetLocalHostMap(port, &top);
        if (res != MSW1394_SUCCESS)
            return res;
        for (i=0; i<top->TOP_Self_ID_Count; i++) {
            sSELF_ID* packet = top->TOP_Self_ID_Array+i;
            for (k=0;k<ncams;k++) {
                cam = (dc1394camera_msw_t *) cams[k];
                if ((cam->camera.node == packet->SID_Phys_ID) &&
                    (cam->camera.port == port)) {
                    cam->selfid = *packet;
                    break;
                }
            }
            if (packet->SID_More_Packets) {
                while ( i<top->TOP_Self_ID_Count && top->TOP_Self_ID_Array[i].SID_More_Packets)
                    i++;
                i--;
            }
        }
        free(top);
    }
    for (k=0; k<ncams; k++) {
        cam = (dc1394camera_msw_t *) cams[k];
        cam->camera.phy_delay=cam->selfid.SID_Delay+DC1394_PHY_DELAY_MIN;
        cam->camera.phy_speed=cam->selfid.SID_Speed+DC1394_ISO_SPEED_MIN;
        cam->camera.power_class=cam->selfid.SID_Power_Class+DC1394_POWER_CLASS_MIN;
    }
    return res;
}

// ############# public ##################################

dc1394camera_t*
dc1394_new_camera_platform (uint32_t port, uint16_t node)
{
    dc1394camera_msw_t *cam;

    cam=(dc1394camera_msw_t *)calloc(1,sizeof(dc1394camera_msw_t));
    if (cam==NULL)
        return NULL;

    cam->camera.port = (int) port;
    return (dc1394camera_t *) cam;
}

void
dc1394_free_camera_platform (dc1394camera_t *camera)
{
    DC1394_CAST_CAMERA_TO_MSW(cmsw, camera);
    if (cmsw == NULL)
        return;

    free(cmsw);
}

dc1394error_t
dc1394_print_camera_info_platform (dc1394camera_t *camera, FILE *fd)
{
    // DC1394_CAST_CAMERA_TO_MSW(cmsw, camera);
    fprintf(fd,"------ No camera platform-specific information ------\n");
    return DC1394_SUCCESS;
}

dc1394error_t
dc1394_find_cameras_platform(dc1394camera_t ***cameras_ptr, uint32_t* numCameras)
{
    dc1394error_t res = DC1394_SUCCESS;
    msw1394error_t err1394 = MSW1394_SUCCESS;
    dc1394error_t err=DC1394_SUCCESS;
    ULONG port_num, port;
    uint32_t allocated_size;
    dc1394camera_t **cameras, *tmpcam=NULL, **newcam;
    uint32_t numCam, i;
    ULONG numNodes, node;

    if (!msw1394_IsInited()) {
        err1394 = msw1394_Init();
        if (err1394 != MSW1394_SUCCESS)
            return ConvertError(err1394);
    }

    port_num = 0;
    err1394 = msw1394_GetNumPorts(&port_num);
    if (err1394 != MSW1394_SUCCESS)
        return ConvertError(err1394);
    allocated_size=64; // initial allocation, will be reallocated if necessary
    cameras=(dc1394camera_t**)malloc(allocated_size*sizeof(dc1394camera_t*));
    if (!cameras)
        return DC1394_MEMORY_ALLOCATION_FAILURE;
    numCam=0;

    for (port=0;port<port_num;port++) {
        err1394 = msw1394_GetNodeCount(port,&numNodes);
        if (err1394 != MSW1394_SUCCESS) {
            res =  ConvertError(err1394);
            goto fail;
        }
        USHORT local = 0;
        err1394 = msw1394_GetLocalId(port, &local);
        local &= 0x3f;
        for (node=0; node<numNodes; node++) {
            if (node == local)
                continue;
            int duplicate = 0;
            tmpcam = dc1394_new_camera(port,node);
            if (tmpcam == NULL) {
                res = DC1394_MEMORY_ALLOCATION_FAILURE;
                goto fail;
            }

            err=dc1394_update_camera_info(tmpcam);
            if (err != DC1394_SUCCESS) {
                dc1394_free_camera(tmpcam);
                tmpcam=NULL;
                continue;
            }

            // check if this camera was not found yet. (a camera might appear twice with strange bus topologies)

            if (numCam > 0)
                for (i=0; i<numCam && !duplicate; i++)
                    duplicate = tmpcam->euid_64 == cameras[i]->euid_64;
            if (duplicate) {
                dc1394_free_camera(tmpcam);
                tmpcam=NULL;
                continue;
            }

            cameras[numCam]=tmpcam;
            tmpcam=NULL;
            numCam++;

            if (numCam == allocated_size) {
                allocated_size*=2;
                newcam=realloc(cameras,allocated_size*sizeof(dc1394camera_t*));
                if (newcam == NULL) {
                    res = DC1394_MEMORY_ALLOCATION_FAILURE;
                    goto fail;
                }
                cameras = newcam;
            }
        }
    }

    *numCameras=numCam;
    *cameras_ptr=cameras;
    if (numCam==0) {
        res = DC1394_NO_CAMERA;
        free(cameras);
        *cameras_ptr = NULL;
    } else {
        res = GrabSelfIds(cameras, numCam);
        if (res != DC1394_SUCCESS)
            goto fail;
    }

    return res;
 fail:
    for (i=0;i<numCam;i++) {
        dc1394_free_camera(cameras[i]);
        cameras[i]=NULL;
    }
    free(cameras);
    *cameras_ptr = NULL;

    return res;
}

dc1394error_t
dc1394_reset_bus_platform (dc1394camera_t * camera)
{
    int port = camera->port;
    msw1394error_t err1394 = msw1394_ResetBus(port);
    dc1394error_t res = ConvertError(err1394);
    return res;
}

#define EPOCHFILETIME (116444736000000000LL)
dc1394error_t
dc1394_read_cycle_timer_platform (dc1394camera_t * camera, uint32_t * cycle_timer, uint64_t * local_time)
{
    msw1394error_t err1394;
    ULONG port = camera->port;
    FILETIME        ft;
    LARGE_INTEGER   li;

    err1394 = msw1394_GetCycleTime(port, (sCYCLE_TIME*)cycle_timer);
    if (err1394 != MSW1394_SUCCESS)
        return ConvertError(err1394);

    GetSystemTimeAsFileTime(&ft);
    li.LowPart  = ft.dwLowDateTime;
    li.HighPart = ft.dwHighDateTime;
    *local_time  = li.QuadPart;       /* In 100-nanosecond intervals */
    *local_time -= EPOCHFILETIME;     /* Offset to the Epoch time */
    *local_time /= 10;                /* In microseconds */

    return DC1394_SUCCESS;
}

dc1394error_t
GetCameraROMValues(dc1394camera_t *camera, uint64_t offset, uint32_t *value, uint32_t num_quads)
{
    msw1394error_t err1394;
    int retry= DC1394_MAX_RETRIES;
    int port = camera->port;
    int node = camera->node;

    while (retry--) {
#ifdef DC1394_DEBUG_LOWEST_LEVEL
        fprintf(stderr,"get %d regs at 0x%llx : ", num_quads, offset + CONFIG_ROM_BASE);
#endif
        err1394 = msw1394_ReadSync(port,0xffc0 | node, offset + CONFIG_ROM_BASE, 4 * num_quads, value);
        ifdef DC1394_DEBUG_LOWEST_LEVEL
            fprintf(stderr,"0x%lx [...]\n", value[0]);
#endif
        if (err1394 == MSW1394_SUCCESS)
            break;
        DWORD t = DC1394_SLOW_DOWN/1000;
        Sleep(t);
    }
    int i;
    for (i = 0; i < num_quads; i++)
        value[i] = ntohl(value[i]);
    dc1394error_t res = ConvertError(err1394);
    return res;
}

dc1394error_t
SetCameraROMValues(dc1394camera_t *camera, uint64_t offset, uint32_t *value, uint32_t num_quads)
{
    int retry= DC1394_MAX_RETRIES;
    int port = camera->port;
    int node = camera->node;
    msw1394error_t err1394;
    int i;
    for (i = 0; i < num_quads; i++)
        value[i] = ntohl(value[i]);
    while (retry--) {
#ifdef DC1394_DEBUG_LOWEST_LEVEL
        fprintf(stderr,"set %d regs at 0x%llx to value 0x%lx [...]\n", num_quads, offset + CONFIG_ROM_BASE, value);
#endif
        err1394 = msw1394_WriteSync(port,0xffc0 | node, offset + CONFIG_ROM_BASE, 4 * num_quads, value);
        if (err1394 == MSW1394_SUCCESS)
            break;
        DWORD t = DC1394_SLOW_DOWN/1000;
        Sleep(t);
    }
    dc1394error_t res = ConvertError(err1394);
    return res;
}

dc1394error_t
dc1394_allocate_iso_channel(dc1394camera_t *camera) {
    dc1394error_t err;
    dc1394switch_t iso_was_on;
    char ch;
    msw1394error_t err1394;

    if (camera->capture_is_set) {
        dc1394_log_error("capturing in progress, cannot allocate ISO channel");
        return DC1394_CAPTURE_IS_RUNNING;
    }

    if (camera->iso_channel_is_set) {
        dc1394_log_error("a channel is already allocated to this camera");
        return DC1394_FAILURE;
    }

    // if transmission is ON, abort
    err=dc1394_video_get_transmission(camera,&iso_was_on);
    DC1394_ERR_RTN(err, "Could not get ISO status");

    if (iso_was_on==DC1394_ON) {
        dc1394_log_error("Camera is already streaming, aborting ISO channel allocation... Perhaps it is in use by another application or a previous session was not cleaned up properly.");
        return DC1394_FAILURE;
    }

    // first we need to assign an ISO channel:
    if (camera->iso_channel >= 0) {
        // a specific channel is requested. try to book it.
        ch = camera->iso_channel;
        err1394 = msw1394_ISOAllocChan(camera->port,&ch);
        if (err1394 == MSW1394_SUCCESS) {
            // channel allocated.
            dc1394_log_debug("Allocated ISO channel %d as requested", camera->iso_channel);

            camera->iso_channel_is_set=1;
            err=dc1394_video_set_iso_channel(camera, camera->iso_channel);
            DC1394_ERR_RTN(err, "Could not set ISO channel in the camera");
            return DC1394_SUCCESS;
        }
        dc1394_log_warning("Channel %d already reserved. Trying other channels.", camera->iso_channel);
    }

    ch = -1;
    err1394 = msw1394_ISOAllocChan(camera->port,&ch);
    if (err1394 == MSW1394_SUCCESS) {
        // channel allocated.
        camera->iso_channel=ch;
        camera->iso_channel_is_set=1;
    }

    // check if channel was allocated:
    if (camera->iso_channel_is_set==0)
        return DC1394_NO_ISO_CHANNEL;

    // set channel in the camera
    err=dc1394_video_set_iso_channel(camera, camera->iso_channel);
    DC1394_ERR_RTN(err, "Could not set ISO channel in the camera");

    return DC1394_SUCCESS;
}

dc1394error_t
dc1394_allocate_bandwidth(dc1394camera_t *camera)
{
    DC1394_CAST_CAMERA_TO_MSW(cmsw, camera);
    dc1394error_t err;
    msw1394error_t err1394;

    if (camera->capture_is_set) {
        dc1394_log_error("capturing in progress, cannot allocate ISO bandwidth");
        return DC1394_FAILURE;
    }

    if (camera->iso_bandwidth) {
        dc1394_log_error("bandwidth already allocated for this camera");
        return DC1394_FAILURE;
    }

    err=dc1394_video_get_bandwidth_usage(camera, &camera->iso_bandwidth);
    DC1394_ERR_RTN(err, "Could not estimate ISO bandwidth");
    err1394 = msw1394_ISOAllocBandwidth(camera->port, camera->iso_bandwidth, &cmsw->bw_handle);
    if (err1394 != MSW1394_SUCCESS) {
        camera->iso_bandwidth=0;

        return DC1394_NO_BANDWIDTH;
    }
    dc1394_log_debug("Allocated %d bandwidth units",camera->iso_bandwidth);

    return DC1394_SUCCESS;
}

dc1394error_t
dc1394_free_iso_channel(dc1394camera_t *camera)
{
    dc1394error_t err;
    msw1394error_t err1394;

    if (camera->capture_is_set) {
        dc1394_log_error("capturing in progress, cannot free ISO channel");
        return DC1394_FAILURE;
    }

    if (camera->is_iso_on) {
        dc1394_log_warning("Warning: stopping transmission in order to free ISO channel");
        err=dc1394_video_set_transmission(camera, DC1394_OFF);
        DC1394_ERR_RTN(err, "Could not stop ISO transmission");
    }

    if (camera->iso_channel_is_set == 0) {
        dc1394_log_error("no ISO channel to free");
    return DC1394_FAILURE;
    }

    err1394 = msw1394_ISOFreeChan(camera->port, camera->iso_channel);
    if (err1394 != MSW1394_SUCCESS) {
        dc1394_log_error("could not free iso channel %d!", camera->iso_channel);
        return DC1394_RAW1394_FAILURE;
    }

    dc1394_log_debug("Freed iso channel %d", camera->iso_channel);

    //camera->iso_channel=-1; // we don't need this line anymore.
    camera->iso_channel_is_set=0;
    return DC1394_SUCCESS;

}

dc1394error_t
dc1394_free_bandwidth(dc1394camera_t *camera)
{
    DC1394_CAST_CAMERA_TO_MSW(cmsw, camera);
    dc1394error_t err;
    msw1394error_t err1394;

    if (camera->capture_is_set) {
        dc1394_log_error("capturing in progress, cannot free ISO bandwidth");
        return DC1394_FAILURE;
    }

    if (camera->is_iso_on) {
        dc1394_log_warning("Warning: stopping transmission in order to free ISO bandwidth");
        err=dc1394_video_set_transmission(camera, DC1394_OFF);
        DC1394_ERR_RTN(err, "Could not stop ISO transmission");
    }

    if (camera->iso_bandwidth == 0) {
        dc1394_log_error("no ISO bandwidth to free");
        return DC1394_FAILURE;
    }

    err1394 = msw1394_ISOFreeBandwidth(camera->port, cmsw->bw_handle);
    if (err1394 != MSW1394_SUCCESS) {
        dc1394_log_error("could not free %d units of bandwidth!", camera->iso_bandwidth);
        return DC1394_RAW1394_FAILURE;
    }

    dc1394_log_debug("Freed some bandwidth units");

    camera->iso_bandwidth=0;
    cmsw->bw_handle = INVALID_HANDLE_VALUE;
    return DC1394_SUCCESS;
}

// ########### capture #################

dc1394error_t
dc1394_capture_setup(dc1394camera_t *camera, uint32_t num_dma_buffers, uint32_t flags)
{
    DC1394_CAST_CAMERA_TO_MSW(cmsw, camera);
    dc1394error_t err;
    msw1394error_t err1394;
    uint32_t i;
    dc1394video_frame_t* f;

    // if capture is already set, abort
    if (camera->capture_is_set>0)
        return DC1394_CAPTURE_IS_RUNNING;

    if (flags & DC1394_CAPTURE_FLAGS_DEFAULT)
        flags = DC1394_CAPTURE_FLAGS_CHANNEL_ALLOC |
            DC1394_CAPTURE_FLAGS_BANDWIDTH_ALLOC;

    cmsw->capture_flags=flags;

    // if auto iso is requested, stop ISO (if necessary)
    if (flags & DC1394_CAPTURE_FLAGS_AUTO_ISO) {
        dc1394switch_t is_iso_on;
        dc1394_video_get_transmission(camera, &is_iso_on);
        if (is_iso_on == DC1394_ON) {
            err=dc1394_video_set_transmission(camera, DC1394_OFF);
            DC1394_ERR_RTN(err,"Could not stop ISO!");
        }
    }

    // allocate channel/bandwidth if requested
    if (flags & DC1394_CAPTURE_FLAGS_CHANNEL_ALLOC) {
        err=dc1394_allocate_iso_channel(camera);
        DC1394_ERR_RTN(err,"Could not allocate ISO channel!");
    }
    if (flags & DC1394_CAPTURE_FLAGS_BANDWIDTH_ALLOC) {
        err=dc1394_allocate_bandwidth(camera);
        DC1394_ERR_RTN(err,"Could not allocate bandwidth!");
    }

    cmsw->frames = malloc (num_dma_buffers * sizeof (dc1394video_frame_t));
    if (cmsw->frames == NULL) {
        err = DC1394_MEMORY_ALLOCATION_FAILURE;
        goto fail;
    }

    err=capture_basic_setup(camera, cmsw->frames);
    if (err != DC1394_SUCCESS)
        goto fail;

    memset(&(cmsw->ISO),0,sizeof(cmsw->ISO));
    cmsw->ISO.Port = camera->port;
    cmsw->ISO.nChannel = camera->iso_channel;
    cmsw->ISO.Speed = dc1394_2_msw1394_speed(camera->iso_speed);
    cmsw->ISO.nMaxBytesPerFrame = cmsw->frames[0].packet_size+4;
    cmsw->ISO.nNumberOfBuffers = num_dma_buffers;
    cmsw->ISO.nMaxBufferSize = cmsw->frames[0].total_bytes;
    cmsw->ISO.nQuadletsToStrip = 1;
    cmsw->ISO.hResource = INVALID_HANDLE_VALUE;
    err1394 = msw1394_ISOCaptureSetup(&cmsw->ISO);

    if (err1394 != MSW1394_SUCCESS) {
        err = ConvertError(err1394);
        goto fail;
    }

    for (i = 0; i < num_dma_buffers; i++) {
        f = cmsw->frames + i;
        if (i > 0)
            memcpy (f, cmsw->frames, sizeof (dc1394video_frame_t));
        f->image = (unsigned char *) cmsw->ISO.Buffers[i];
        f->id = i;
    }
    camera->capture_is_set = 1;

    // if auto iso is requested, start ISO
    if (flags & DC1394_CAPTURE_FLAGS_AUTO_ISO) {
        err=dc1394_video_set_transmission(camera, DC1394_ON);
        DC1394_ERR_RTN(err,"Could not start ISO!");
        camera->iso_auto_started=1;
    }

    return DC1394_SUCCESS;

 fail:
    // free resources if they were allocated
    if (flags & DC1394_CAPTURE_FLAGS_CHANNEL_ALLOC) {
        if (dc1394_free_iso_channel(camera) != DC1394_SUCCESS)
            dc1394_log_warning("Could not free ISO channel!");
    }
    if (flags & DC1394_CAPTURE_FLAGS_BANDWIDTH_ALLOC) {
        if (dc1394_free_bandwidth(camera) != DC1394_SUCCESS)
            dc1394_log_warning("Could not free ISO bandwidth!");
    }

    free (cmsw->frames);
    cmsw->frames = NULL;
    DC1394_ERR_RTN(err,"Could not setup DMA capture");

    return err;
}

/*****************************************************
 CAPTURE_STOP
*****************************************************/

dc1394error_t
dc1394_capture_stop(dc1394camera_t *camera)
{
    int err;
    DC1394_CAST_CAMERA_TO_MSW(cmsw, camera);
    msw1394error_t err1394 = MSW1394_SUCCESS;

    if (camera->capture_is_set>0) {
        switch (camera->capture_is_set) {
        case 1: // RAW 1394 is obsolete
            return DC1394_INVALID_CAPTURE_MODE;
        case 2: // DMA (VIDEO1394)
            err1394 = msw1394_ISOCaptureStop(&cmsw->ISO);
            if (err1394 != MSW1394_SUCCESS)
                return ConvertError(err1394);
            free (cmsw->frames);
            cmsw->frames = NULL;
            break;
        default:
            return DC1394_INVALID_CAPTURE_MODE;
            break;
        }

        // capture is not set anymore
        camera->capture_is_set=0;

        // free ressources if they were allocated
        if ((cmsw->capture_flags & DC1394_CAPTURE_FLAGS_CHANNEL_ALLOC) >0) {
            err=dc1394_free_iso_channel(camera);
            DC1394_ERR_RTN(err,"Could not free ISO channel!");
        }
        if ((cmsw->capture_flags & DC1394_CAPTURE_FLAGS_BANDWIDTH_ALLOC) >0) {
            err=dc1394_free_bandwidth(camera);
            DC1394_ERR_RTN(err,"Could not free bandwidth!");
        }

    }
    else {
        return DC1394_CAPTURE_IS_NOT_SET;
    }

    // stop ISO if it was started automatically
    if (camera->iso_auto_started>0) {
        dc1394error_t err=dc1394_video_set_transmission(camera, DC1394_OFF);
        DC1394_ERR_RTN(err,"Could not stop ISO!");
        camera->iso_auto_started=0;
    }

    return DC1394_SUCCESS;
}

dc1394error_t
dc1394_capture_dequeue(dc1394camera_t * camera, dc1394capture_policy_t policy, dc1394video_frame_t **frame)
{
    DC1394_CAST_CAMERA_TO_MSW(cmsw, camera);
    msw1394error_t err1394;
    sCYCLE_TIME* ct;

    if ( (policy<DC1394_CAPTURE_POLICY_MIN) || (policy>DC1394_CAPTURE_POLICY_MAX) )
        return DC1394_INVALID_CAPTURE_POLICY;

    // default: return NULL in case of failures or lack of frames
    *frame=NULL;

    int wait = policy != DC1394_CAPTURE_POLICY_POLL;
    ULONG idx = cmsw->ISO.nNumberOfBuffers;

    err1394 = msw1394_ISOCaptureDequeue(&cmsw->ISO, wait, &idx);

    if (err1394 != MSW1394_SUCCESS) {
        if (err1394 == MSW1394_NO_DATA_AVAILABLE && (policy==DC1394_CAPTURE_POLICY_POLL))
            return DC1394_SUCCESS;
        else
            return ConvertError(err1394);
    }

    *frame=cmsw->frames + idx;
    ct = (sCYCLE_TIME*)(cmsw->ISO.Times + idx);
    (*frame)->timestamp =
        (uint64_t)ct->CL_SecondCount*1000000 +
        (uint64_t)ct->CL_CycleCount*125 +
        (uint64_t)ct->CL_CycleOffset*125/3071;
    (*frame)->frames_behind = cmsw->ISO.ReadyBuffers;
    return DC1394_SUCCESS;
}

dc1394error_t
dc1394_capture_enqueue(dc1394camera_t * camera, dc1394video_frame_t * frame)
{
    DC1394_CAST_CAMERA_TO_MSW(cmsw, camera);
    msw1394error_t err1394;

    if (frame->camera != camera) {
        dc1394_log_error("camera does not match frame's camera");
        return DC1394_INVALID_ARGUMENT_VALUE;
    }

    err1394 = msw1394_ISOCaptureEnqueue(&cmsw->ISO, frame->id);

    return ConvertError(err1394);
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
