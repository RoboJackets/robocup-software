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

#define NOCRYPT
#define WINADVAPI

#include <windows.h>

//typedef LONG NTSTATUS;
#include <wxp/ntdd1394.h>

#include <winioctl.h>
#include <setupapi.h>
#include <dbt.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "msw1394.h"
#include <1394api.h>

#include <initguid.h>
#include <driver/common/common.h>

#ifdef _MSC_VER
#define snprintf sprintf_s
#endif

#define ISOCH_ANY_CHANNEL                       0xffffffff

#define kMaxPorts 4
#define kWinBus_Name "\\\\.\\1394BUS"
#define kWinVDev_Name "1394_VIRTUAL_DEVICE"

static BOOL gInited = 0;
static int gNPorts = 0;

typedef struct _sWinrawDevice {
    HANDLE h;
    char* name;
    char* path;
    ULONG gen;
    HANDLE BusResetThread;
    DWORD BusResetThreadID;
} sWinrawDevice;

sWinrawDevice gDevice[kMaxPorts];

static ULONG SpeedFlag(msw1394_speed_t msw1394_speed) {
    ULONG res = SPEED_FLAGS_400;
    switch (msw1394_speed) {
    case MSW1394_SPEED100:
        res = SPEED_FLAGS_100;
        break;
    case MSW1394_SPEED200:
        res = SPEED_FLAGS_200;
        break;
    default:
        break;
    }
    return res;
}

static LPVOID GetErrorText(DWORD iErr) {
    LPVOID lpMsgBuf;
    FormatMessage(
                  FORMAT_MESSAGE_ALLOCATE_BUFFER |
                  FORMAT_MESSAGE_FROM_SYSTEM,
                  NULL,
                  iErr,
                  MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                  (LPTSTR) &lpMsgBuf,
                  0, NULL );
    return lpMsgBuf;
}

static DWORD UpdateGenCount(int iPort) {
    assert(iPort >= 0 && iPort < gNPorts);
    DWORD dwRet = 0, dwBytesRet;
    ULONG GenerationCount;
    dwRet = DeviceIoControl( gDevice[iPort].h,
                             IOCTL_GET_GENERATION_COUNT,
                             &GenerationCount,
                             sizeof(ULONG),
                             &GenerationCount,
                             sizeof(ULONG),
                             &dwBytesRet,
                             NULL
                             );

    if (dwRet) {
        gDevice[iPort].gen = GenerationCount;
        return 0;
    } else {
        dwRet = GetLastError();
        return dwRet;
    }
}

#define kBusResetHandlers_MaxNum 16
HANDLE gBusResetMx = INVALID_HANDLE_VALUE;
BusResetHandler_t gBusResetHandlers[kBusResetHandlers_MaxNum];
ULONG gBusResetHandlersNum = 0;

static DWORD WINAPI
BusResetThread(LPVOID lpParameter) {
    int port = (int)lpParameter;
    ULONG i, n;
    BusResetHandler_t handlers[kBusResetHandlers_MaxNum];
    DWORD dwRet, dwBytesRet;
    OVERLAPPED overLapped;

    HANDLE hDevice =
        CreateFile( gDevice[port].path,
                    GENERIC_WRITE | GENERIC_READ,
                    FILE_SHARE_WRITE | FILE_SHARE_READ,
                    NULL,
                    OPEN_EXISTING,
                    FILE_FLAG_OVERLAPPED | FILE_FLAG_NO_BUFFERING,
                    NULL
                    );

    if (hDevice == INVALID_HANDLE_VALUE)
        ExitThread(0);
    memset(&overLapped,0,sizeof(overLapped));
    overLapped.hEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
    while (TRUE) {
        ResetEvent(overLapped.hEvent);
        DeviceIoControl( hDevice,
                         IOCTL_BUS_RESET_NOTIFY,
                         NULL,
                         0,
                         NULL,
                         0,
                         &dwBytesRet,
                         &overLapped
                         );
        dwRet = GetLastError();
        // we should always return pending, if not, something's wrong...
        if (dwRet == ERROR_IO_PENDING) {
            dwRet = WaitForSingleObject(overLapped.hEvent, INFINITE);
            if (dwRet != WAIT_FAILED) {
                // bus reset!

                ULONG GenerationCount;
                ResetEvent(overLapped.hEvent);
                DWORD res = DeviceIoControl( hDevice,
                                             IOCTL_GET_GENERATION_COUNT,
                                             &GenerationCount,
                                             sizeof(ULONG),
                                             &GenerationCount,
                                             sizeof(ULONG),
                                             &dwBytesRet,
                                             &overLapped
                                             );
                dwRet = WaitForSingleObject(overLapped.hEvent, INFINITE);

                if (res) {
                    gDevice[port].gen = GenerationCount;
                }
                dc1394_log_warning("Bus Reset: gen: %ld, wait res %lX!", gDevice[port].gen, dwRet);

                WaitForSingleObject(gBusResetMx,INFINITE);
                n = gBusResetHandlersNum;
                if (n>0)
                    memcpy(handlers,gBusResetHandlers,n*sizeof(BusResetHandler_t));
                ReleaseMutex(gBusResetMx);
                for (i=0; i<n; i++) {
                    BusResetHandler_t h;
                    h = handlers[i];
                    (*h)(port);
                }
            } else {
                dwRet = GetLastError();
                dc1394_log_error("Error %ld in Bus reset thread for port %d",dwRet,port);
            }
        } else {
            dc1394log_error("Error from IOCTL_BUS_RESET_NOTIFY");
            fflush(stdout);
        }
    }

    CloseHandle(hDevice);
    CloseHandle(overLapped.hEvent);
    ExitThread(0);
}

DWORD StartBusResetThread(int iPort) {
    DWORD dwRet;
    gDevice[iPort].BusResetThread = CreateThread(
                                                 NULL,
                                                 0,
                                                 BusResetThread,
                                                 (LPVOID)iPort,
                                                 0,
                                                 &(gDevice[iPort].BusResetThreadID)
                                                 );

    if (gDevice[iPort].BusResetThread == NULL) {
        dwRet = GetLastError();
        dc1394_log_error("Failed to start bus reset thread");
        return dwRet;
    }
    if (!SetThreadPriority(gDevice[iPort].BusResetThread,THREAD_PRIORITY_HIGHEST)) {
        dwRet = GetLastError();
        dc1394_log_error("Failed to increase reset thread priority");
    }
    return 0;
}

static msw1394error_t ConvertError(DWORD iWinErr) {
    msw1394error_t res = MSW1394_SUCCESS;
    switch (iWinErr) {
    case 0:
        res = MSW1394_SUCCESS;
        break;
    case ERROR_NO_SYSTEM_RESOURCES:
        res = MSW1394_NO_SYSTEM_RESOURCES;
        break;
    default:
        res = MSW1394_FAILURE;
        break;
    }
    return res;
}

// ############ public ###########################

int msw1394_IsInited() {
    return gInited;
}

msw1394error_t msw1394_Init() {
    if (msw1394_IsInited())
        return MSW1394_SUCCESS;

    HANDLE  hDevice;
    int Port, nPorts=0, i;
    DWORD  dwBytesRet,dwErr;
    BOOL  bRes;

    for (Port = 0; Port < kMaxPorts; Port++)
        memset(gDevice+Port,0,sizeof(sWinrawDevice));

    PIEEE1394_API_REQUEST p1394ApiReq = NULL;
    PIEEE1394_VDEV_PNP_REQUEST pDevPnpReq;
    for (Port = 0; Port<kMaxPorts; Port++) {

        char busName[128];
        snprintf(busName,sizeof(busName),"%s%d",kWinBus_Name,Port);

        hDevice =
            CreateFile(
                       busName,
                       GENERIC_WRITE | GENERIC_READ,
                       FILE_SHARE_WRITE | FILE_SHARE_READ,
                       NULL,
                       OPEN_EXISTING,
                       0,
                       NULL
                       );
        if (hDevice == INVALID_HANDLE_VALUE)
            break; // no more adaptors
        else
            nPorts++;

        size_t nameLen = strlen(kWinVDev_Name);
        size_t recLen = sizeof(IEEE1394_API_REQUEST) + nameLen;
        p1394ApiReq = (PIEEE1394_API_REQUEST) LocalAlloc( LPTR, recLen);
        if (p1394ApiReq == NULL)
            return MSW1394_NO_MEMORY;
        p1394ApiReq->RequestNumber = IEEE1394_API_ADD_VIRTUAL_DEVICE;
        //p1394ApiReq->Flags = IEEE1394_REQUEST_FLAG_PERSISTENT;
        // For the virtual device to use the host controller's unique instance ID
        // If used here it must be used for removal
        //p1394ApiReq->Flags |= IEEE1394_REQUEST_FLAG_USE_LOCAL_HOST_EUI;
        pDevPnpReq = &p1394ApiReq->u.AddVirtualDevice;
        pDevPnpReq->fulFlags = 0;
        pDevPnpReq->Reserved = 0;
        pDevPnpReq->InstanceId.LowPart = Port;
        pDevPnpReq->InstanceId.HighPart = 0;
        strncpy((char*)&pDevPnpReq->DeviceId,kWinVDev_Name,nameLen);

        bRes = DeviceIoControl(
                               hDevice,
                               IOCTL_IEEE1394_API_REQUEST,
                               p1394ApiReq,
                               recLen,
                               NULL,
                               0,
                               &dwBytesRet,
                               NULL);

        if (hDevice != INVALID_HANDLE_VALUE) {
            CloseHandle(hDevice);
        }

        if (p1394ApiReq != NULL) {
            LocalFree(p1394ApiReq);
            p1394ApiReq = NULL;
        }
    }

    Sleep(200);

    HDEVINFO hDevInfo;
    GUID guid = GUID_1394VDEV;
    hDevInfo = SetupDiGetClassDevs( &guid,
                                    NULL,
                                    NULL,
                                    (DIGCF_PRESENT | DIGCF_INTERFACEDEVICE)
                                    );
    if (!hDevInfo)
        return ConvertError(GetLastError());

    SP_DEVICE_INTERFACE_DATA            deviceInterfaceData;
    PSP_DEVICE_INTERFACE_DETAIL_DATA    DeviceInterfaceDetailData;
    ULONG requiredSize;
    for (Port = 0; Port<nPorts; Port++) {

        deviceInterfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
        if (SetupDiEnumDeviceInterfaces(hDevInfo, 0, &guid, Port, &deviceInterfaceData)) {
            bRes =
                SetupDiGetDeviceInterfaceDetail(
                                                hDevInfo,
                                                &deviceInterfaceData,
                                                NULL,
                                                0,
                                                &requiredSize,
                                                NULL
                                                );
            if (!bRes) {
                dwErr = GetLastError();
                if (dwErr != ERROR_INSUFFICIENT_BUFFER)
                    return ConvertError(dwErr);
            }

            DeviceInterfaceDetailData = (PSP_DEVICE_INTERFACE_DETAIL_DATA) LocalAlloc(LPTR, requiredSize);
            if (DeviceInterfaceDetailData == NULL) {
                dwErr = GetLastError();
                return ConvertError(dwErr);
            }
            DeviceInterfaceDetailData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);
            bRes =
                SetupDiGetDeviceInterfaceDetail(
                                                hDevInfo,
                                                &deviceInterfaceData,
                                                DeviceInterfaceDetailData,
                                                requiredSize,
                                                NULL,
                                                NULL
                                                );
            if (bRes) {
                i = strlen(DeviceInterfaceDetailData->DevicePath) + 1;
                gDevice[Port].path = malloc(i);
                memcpy(gDevice[Port].path,DeviceInterfaceDetailData->DevicePath,i);
            }

            LocalFree(DeviceInterfaceDetailData);

            if (bRes) {
                gDevice[Port].h = CreateFile( gDevice[Port].path,
                                              GENERIC_WRITE | GENERIC_READ,
                                              FILE_SHARE_WRITE | FILE_SHARE_READ,
                                              NULL,
                                              OPEN_EXISTING,
                                              0,
                                              NULL
                                              );
                if (gDevice[Port].h != INVALID_HANDLE_VALUE) {
                    gNPorts++;
                    gInited = TRUE;
                    UpdateGenCount(Port);
                    char pname[128];
                    snprintf(pname,sizeof(pname),"Port%d",Port);
                    i = strlen(pname)+1;
                    gDevice[Port].name = malloc(i);
                    memcpy(gDevice[Port].name,pname,i);
                } else
                    break;
            } else
                break;
        }
    }

    if (!gInited)
        return ConvertError(GetLastError());

    gBusResetMx = CreateMutex(NULL,FALSE, NULL);
    if (gBusResetMx == NULL) {
        gInited = FALSE;
        return ConvertError(GetLastError());
    }

    for (Port = 0; Port<nPorts; Port++) {
        dwErr = StartBusResetThread(Port);
        if (dwErr != 0) {
            gInited = FALSE;
            return ConvertError(dwErr);
        }
    }
    return MSW1394_SUCCESS;
}

msw1394error_t msw1394_GetNumPorts(ULONG *oPortNum) {
    *oPortNum = gNPorts;
    return MSW1394_SUCCESS;
}

msw1394error_t msw1394_GetNodeCount(ULONG iPort, ULONG *oNumNodes) {
    DWORD dwRet, dwBytesRet;
    USHORT TOP_Length = 0x400;
    ULONG CsrDataLength = TOP_Length;
    ULONG ulBufferSize = sizeof(GET_LOCAL_HOST_INFORMATION)+sizeof(GET_LOCAL_HOST_INFO6)+CsrDataLength;
    PGET_LOCAL_HOST_INFORMATION pGetLocalHostInfo;
    PGET_LOCAL_HOST_INFO6 LocalHostInfo6;
    PTOPOLOGY_MAP top;
    int n;

    if ( iPort >= gNPorts)
        return MSW1394_INVALID_PORT;


    pGetLocalHostInfo = (PGET_LOCAL_HOST_INFORMATION)LocalAlloc(LPTR, ulBufferSize);

    if (!pGetLocalHostInfo)
        return MSW1394_NO_MEMORY;
    pGetLocalHostInfo->nLevel = 6;
    pGetLocalHostInfo->ulBufferSize = sizeof(GET_LOCAL_HOST_INFO6)+sizeof(TOPOLOGY_MAP)+0x400;

    LocalHostInfo6 = (PGET_LOCAL_HOST_INFO6)&pGetLocalHostInfo->Information;
    LocalHostInfo6->CsrBaseAddress.Off_High = INITIAL_REGISTER_SPACE_HI;
    LocalHostInfo6->CsrBaseAddress.Off_Low = TOPOLOGY_MAP_LOCATION;
    LocalHostInfo6->CsrDataLength = CsrDataLength;

    dwRet =
        DeviceIoControl(
                        gDevice[iPort].h,
                        IOCTL_GET_LOCAL_HOST_INFORMATION,
                        pGetLocalHostInfo,
                        ulBufferSize,
                        pGetLocalHostInfo,
                        ulBufferSize,
                        &dwBytesRet,
                        NULL
                        );

    top = (PTOPOLOGY_MAP)(&(LocalHostInfo6->CsrDataBuffer[0]));
    n = top->TOP_Node_Count;
    LocalFree(pGetLocalHostInfo);

    if (dwRet == 0)
      return ConvertError(GetLastError());

    *oNumNodes = n;
    return MSW1394_SUCCESS;
}

msw1394error_t  msw1394_AddBusResetHandler(BusResetHandler_t iH) {
    msw1394error_t res = MSW1394_SUCCESS;
    if (!msw1394_IsInited())
        res = msw1394_Init();
    if (res != MSW1394_SUCCESS)
        return res;
    WaitForSingleObject(gBusResetMx,INFINITE);
    if (gBusResetHandlersNum < kBusResetHandlers_MaxNum)
        gBusResetHandlers[gBusResetHandlersNum++] = iH;
    else
        res = MSW1394_NO_MEMORY;
    ReleaseMutex(gBusResetMx);
    return res;
}

msw1394error_t  msw1394_RemoveBusResetHandler(BusResetHandler_t iH) {
    msw1394error_t res = MSW1394_SUCCESS;
    int i;
    WaitForSingleObject(gBusResetMx,INFINITE);
    for (i = 0; i < gBusResetHandlersNum; i++)
        if (gBusResetHandlers[i] == iH)
            break;
    if (i < gBusResetHandlersNum) {
        gBusResetHandlersNum--;
        if (i < gBusResetHandlersNum)
            gBusResetHandlers[i] = gBusResetHandlers[gBusResetHandlersNum];
    } else
        res = MSW1394_INVALID_ARGUMENT;
    ReleaseMutex(gBusResetMx);
    return res;
}

msw1394error_t msw1394_ResetBus(ULONG iPort) {
    if ( iPort >= gNPorts)
        return MSW1394_INVALID_PORT;
    msw1394error_t res = MSW1394_SUCCESS;
    ULONG fulFlags = BUS_RESET_FLAGS_FORCE_ROOT;
    DWORD dwRet, dwBytesRet;
    BOOL bres = DeviceIoControl( gDevice[iPort].h,
                                 IOCTL_BUS_RESET,
                                 &fulFlags,
                                 sizeof(ULONG),
                                 NULL,
                                 0,
                                 &dwBytesRet,
                                 NULL
                                 );
    if (bres) {
        dwRet = GetLastError();
        res = ConvertError(dwRet);
    }
    return res;
}

msw1394error_t msw1394_GetLocalHostMap(ULONG iPort, sTOPOLOGY_MAP** oMap) {
    ULONG ulBufferSize;
    PGET_LOCAL_HOST_INFORMATION pGetLocalHostInfo;
    PGET_LOCAL_HOST_INFO6 LocalHostInfo6;
    DWORD dwRet = 0, dwBytesRet;
    BOOL bres;
    ULONG CsrDataLength;

    if ( iPort >= gNPorts)
        return MSW1394_INVALID_PORT;

    ulBufferSize = sizeof(GET_LOCAL_HOST_INFORMATION)+sizeof(GET_LOCAL_HOST_INFO6);
    pGetLocalHostInfo = (PGET_LOCAL_HOST_INFORMATION)LocalAlloc(LPTR, ulBufferSize);
    if (!pGetLocalHostInfo)
        return ConvertError(GetLastError());
    pGetLocalHostInfo->nLevel = 6;
    pGetLocalHostInfo->ulBufferSize = sizeof(GET_LOCAL_HOST_INFO6);

    LocalHostInfo6 = (PGET_LOCAL_HOST_INFO6)&pGetLocalHostInfo->Information;
    LocalHostInfo6->CsrBaseAddress.Off_High = INITIAL_REGISTER_SPACE_HI;
    LocalHostInfo6->CsrBaseAddress.Off_Low = TOPOLOGY_MAP_LOCATION;
    LocalHostInfo6->CsrDataLength = 1;

    bres =
        DeviceIoControl(
                        gDevice[iPort].h,
                        IOCTL_GET_LOCAL_HOST_INFORMATION,
                        pGetLocalHostInfo,
                        ulBufferSize,
                        pGetLocalHostInfo,
                        ulBufferSize,
                        &dwBytesRet,
                        NULL
                        );

    CsrDataLength = LocalHostInfo6->CsrDataLength;

    LocalFree(pGetLocalHostInfo);

    if (!bres) {
        dwRet = GetLastError();
        // FIXME: check that error code is correct
        if (dwRet != ERROR_INSUFFICIENT_BUFFER)
            return ConvertError(dwRet);
    }

    ulBufferSize = sizeof(GET_LOCAL_HOST_INFORMATION)+sizeof(GET_LOCAL_HOST_INFO6) + CsrDataLength;
    pGetLocalHostInfo = (PGET_LOCAL_HOST_INFORMATION)LocalAlloc(LPTR, ulBufferSize);
    if (pGetLocalHostInfo == NULL)
        return ConvertError(GetLastError());

    pGetLocalHostInfo->nLevel = 6;
    pGetLocalHostInfo->ulBufferSize = sizeof(GET_LOCAL_HOST_INFO6);

    LocalHostInfo6 = (PGET_LOCAL_HOST_INFO6)&pGetLocalHostInfo->Information;
    LocalHostInfo6->CsrBaseAddress.Off_High = INITIAL_REGISTER_SPACE_HI;
    LocalHostInfo6->CsrBaseAddress.Off_Low = TOPOLOGY_MAP_LOCATION;
    LocalHostInfo6->CsrDataLength = CsrDataLength;

    bres =
        DeviceIoControl(
                        gDevice[iPort].h,
                        IOCTL_GET_LOCAL_HOST_INFORMATION,
                        pGetLocalHostInfo,
                        ulBufferSize,
                        pGetLocalHostInfo,
                        ulBufferSize,
                        &dwBytesRet,
                        NULL
                        );

    if (bres) {
        *oMap = (sTOPOLOGY_MAP*)malloc(LocalHostInfo6->CsrDataLength);
        memcpy(*oMap,&LocalHostInfo6->CsrDataBuffer[0],LocalHostInfo6->CsrDataLength);
    }
    LocalFree(pGetLocalHostInfo);
    return MSW1394_SUCCESS;
}

msw1394error_t msw1394_GetLocalId(ULONG iPort, USHORT* oNode) {
    msw1394error_t res = MSW1394_SUCCESS;
    DWORD       dwRet, dwBytesRet;
    GET_1394_ADDRESS addr;
    if ( iPort >= gNPorts)
        return MSW1394_INVALID_PORT;
    addr.fulFlags = USE_LOCAL_NODE;
    addr.NodeAddress.NA_Bus_Number = 0;
    addr.NodeAddress.NA_Node_Number = 0;
    dwRet = DeviceIoControl( gDevice[iPort].h,
                             IOCTL_GET_1394_ADDRESS_FROM_DEVICE_OBJECT,
                             &addr,
                             sizeof(addr),
                             &addr,
                             sizeof(addr),
                             &dwBytesRet,
                             NULL
                             );
    *oNode = (addr.NodeAddress.NA_Bus_Number<<6) | addr.NodeAddress.NA_Node_Number;

    if (!dwRet)
        res = ConvertError(GetLastError());
    return res;
}

msw1394error_t msw1394_GetCycleTime(ULONG iPort, sCYCLE_TIME *oCycleTime) {
    msw1394error_t res = MSW1394_SUCCESS;
    DWORD       dwRet, dwBytesRet;
    if ( iPort >= gNPorts)
        return MSW1394_INVALID_PORT;
    dwRet = DeviceIoControl( gDevice[iPort].h,
                             IOCTL_ISOCH_QUERY_CURRENT_CYCLE_TIME,
                             NULL,
                             0,
                             oCycleTime,
                             sizeof(CYCLE_TIME),
                             &dwBytesRet,
                             NULL
                             );

    if (!dwRet)
        res = ConvertError(GetLastError());
    return res;
}

msw1394error_t
msw1394_ReadSync(ULONG iPort, USHORT node, ULONGLONG addr, size_t length, void *data) {
    if ( iPort >= gNPorts)
        return MSW1394_INVALID_PORT;

    ULONG ulBufferSize;
    ulBufferSize = sizeof(ASYNC_READ) + length;
    PASYNC_READ asyncRead = (PASYNC_READ)LocalAlloc(LPTR, ulBufferSize);
    if (!asyncRead)
        return MSW1394_NO_MEMORY;
    asyncRead->bRawMode = 0;//TRUE;
    asyncRead->bGetGeneration = 0;//TRUE;
    asyncRead->DestinationAddress.IA_Destination_ID.NA_Bus_Number = 0x3FF;
    asyncRead->DestinationAddress.IA_Destination_ID.NA_Node_Number = node;
    DWORD *tw = (DWORD*) (void*)&addr;
    asyncRead->DestinationAddress.IA_Destination_Offset.Off_High = tw[1];
    asyncRead->DestinationAddress.IA_Destination_Offset.Off_Low = tw[0];
    asyncRead->nNumberOfBytesToRead = length;
    asyncRead->nBlockSize = 0;
    asyncRead->fulFlags = 0;
    asyncRead->ulGeneration = gDevice[iPort].gen;

    DWORD dwRet, dwBytesRet;
    dwRet = DeviceIoControl( gDevice[iPort].h,
                             IOCTL_ASYNC_READ,
                             asyncRead,
                             ulBufferSize,
                             asyncRead,
                             ulBufferSize,
                             &dwBytesRet,
                             NULL
                             );

    if (dwRet) {
        memcpy(data,&asyncRead->Data[0],length);
        dwRet = 0;
    } else
        dwRet = GetLastError();

    LocalFree(asyncRead);
    return ConvertError(dwRet);
}

msw1394error_t msw1394_WriteSync(ULONG iPort, USHORT node, ULONGLONG addr, size_t length, void *data) {
    if ( iPort >= gNPorts)
        return MSW1394_INVALID_PORT;

    ULONG ulBufferSize;
    ulBufferSize = sizeof(ASYNC_READ) + length;
    PASYNC_WRITE asyncWrite = (PASYNC_WRITE)LocalAlloc(LPTR, ulBufferSize);
    if (!asyncWrite)
        return MSW1394_NO_MEMORY;
    asyncWrite->bRawMode = 0;//TRUE;
    asyncWrite->bGetGeneration = 0;//TRUE;
    asyncWrite->DestinationAddress.IA_Destination_ID.NA_Bus_Number = 0x3FF;
    asyncWrite->DestinationAddress.IA_Destination_ID.NA_Node_Number = node;
    DWORD *tw = (DWORD*) (void*) &addr;
    asyncWrite->DestinationAddress.IA_Destination_Offset.Off_High = tw[1];
    asyncWrite->DestinationAddress.IA_Destination_Offset.Off_Low = tw[0];
    asyncWrite->nNumberOfBytesToWrite = length;
    asyncWrite->nBlockSize = 0;
    asyncWrite->fulFlags = 0;
    asyncWrite->ulGeneration = gDevice[iPort].gen;

    memcpy(&asyncWrite->Data[0],data,length);

    DWORD dwRet, dwBytesRet;
    dwRet = DeviceIoControl( gDevice[iPort].h,
                             IOCTL_ASYNC_WRITE,
                             asyncWrite,
                             ulBufferSize,
                             asyncWrite,
                             ulBufferSize,
                             &dwBytesRet,
                             NULL
                             );

    LocalFree(asyncWrite);
    if (dwRet)
        dwRet = 0;
    else
        dwRet = GetLastError();
    return ConvertError(dwRet);
}

msw1394error_t msw1394_ISOAllocChan(ULONG iPort, char *ioCh) {
    if ( iPort >= gNPorts)
        return MSW1394_INVALID_PORT;

    ISOCH_ALLOCATE_CHANNEL ac;
    DWORD       dwRet, dwBytesRet;
    ac.nRequestedChannel = (*ioCh >=0 )?*ioCh:ISOCH_ANY_CHANNEL;

    dwRet =
        DeviceIoControl(
                        gDevice[iPort].h,
                        IOCTL_ISOCH_ALLOCATE_CHANNEL,
                        &ac,
                        sizeof(ISOCH_ALLOCATE_CHANNEL),
                        &ac,
                        sizeof(ISOCH_ALLOCATE_CHANNEL),
                        &dwBytesRet,
                        NULL
                        );
    msw1394error_t res = MSW1394_SUCCESS;
    if (!dwRet) {
        dwRet = GetLastError();
        res = ConvertError(dwRet);
    } else
        *ioCh = ac.Channel;
    return res;
}

msw1394error_t msw1394_ISOFreeChan(ULONG iPort, char iCh) {
    if ( iPort >= gNPorts)
        return MSW1394_INVALID_PORT;
    ULONG nChannel = iCh;
    DWORD dwRet, dwBytesRet;
    dwRet =
        DeviceIoControl(
                        gDevice[iPort].h,
                        IOCTL_ISOCH_FREE_CHANNEL,
                        &nChannel,
                        sizeof(ULONG),
                        NULL,
                        0,
                        &dwBytesRet,
                        NULL
                        );

    msw1394error_t res = MSW1394_SUCCESS;
    if (!dwRet) {
        dwRet = GetLastError();
        res = ConvertError(dwRet);
    }
    return res;
}

msw1394error_t msw1394_ISOAllocBandwidth(int iPort, USHORT iBW, HANDLE *oBwH) {
    DWORD dwRet, dwBytesRet;
    ISOCH_ALLOCATE_BANDWIDTH bw;
    bw.nMaxBytesPerFrameRequested = iBW/4; // bandwidth is for rate 1600 Mbs
    bw.fulSpeed = SPEED_FLAGS_400;

    if ( iPort >= gNPorts)
        return MSW1394_INVALID_PORT;

    dwRet = DeviceIoControl( gDevice[iPort].h,
                             IOCTL_ISOCH_ALLOCATE_BANDWIDTH,
                             &bw,
                             sizeof(ISOCH_ALLOCATE_BANDWIDTH),
                             &bw,
                             sizeof(ISOCH_ALLOCATE_BANDWIDTH),
                             &dwBytesRet,
                             NULL
                             );
    msw1394error_t res = MSW1394_SUCCESS;
    if (dwRet)
        *oBwH = bw.hBandwidth;
    else {
        dwRet = GetLastError();
        res = ConvertError(dwRet);
    }
    return res;
}

msw1394error_t msw1394_ISOFreeBandwidth(int iPort, HANDLE iBwH) {
    DWORD dwRet, dwBytesRet;
    msw1394error_t res = MSW1394_SUCCESS;
    if ( iPort >= gNPorts)
        return MSW1394_INVALID_PORT;

    dwRet = DeviceIoControl( gDevice[iPort].h,
                             IOCTL_ISOCH_FREE_BANDWIDTH,
                             &iBwH,
                             sizeof(HANDLE),
                             NULL,
                             0,
                             &dwBytesRet,
                             NULL
                             );

    if (!dwRet) {
        dwRet = GetLastError();
        res = ConvertError(dwRet);
    }
    return res;

}

static void ISORecCleanup(msw1394_ISO* ioISO) {
    DWORD dwRet, dwBytesRet;
    ULONG i, nb = ioISO->nNumberOfBuffers;

    if (ioISO->hDevice != INVALID_HANDLE_VALUE &&  ioISO->hDevice != NULL) {
        if (ioISO->hResource != INVALID_HANDLE_VALUE && ioISO->hResource != NULL) {
            OVERLAPPED FreeOverlapped;
            FreeOverlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

            dwRet = DeviceIoControl(
                                    ioISO->hDevice,
                                    IOCTL_ISOCH_FREE_RESOURCES,
                                    &(ioISO->hResource),
                                    sizeof(HANDLE),
                                    NULL,
                                    0,
                                    &dwBytesRet,
                                    &FreeOverlapped
                                    );
            if (! dwRet) {
                dwRet = GetLastError();
                if (dwRet == ERROR_IO_PENDING) {
                    dwRet = GetOverlappedResult(
                                                ioISO->hDevice,
                                                &FreeOverlapped,
                                                &dwBytesRet,
                                                TRUE
                                                );
                    if (!dwRet) {
                        dwRet = GetLastError();
                        char* msg = GetErrorText(dwRet);
                        dc1394_log_warning("GetOverlappedResult failed");
                        LocalFree(msg);
                    }
                } else {
                    char* msg = GetErrorText(dwRet);
                    dc1394_log_error("IOCTL_ISOCH_FREE_RESOURCES failed");
                    LocalFree(msg);
                }
            }

            CloseHandle(FreeOverlapped.hEvent);
            ioISO->hResource = INVALID_HANDLE_VALUE;
        }

        CloseHandle(ioISO->hDevice);
        ioISO->hDevice = INVALID_HANDLE_VALUE;
    }

    if (ioISO->Overlapped != NULL) {
        for (i = 0; i<nb; i++)
            if (ioISO->Overlapped[i].hEvent != NULL)
                CloseHandle(ioISO->Overlapped[i].hEvent);
            else
                break;
        free(ioISO->Overlapped);
        ioISO->Overlapped = NULL;
    }
    if (ioISO->BufferMemory != NULL) {
        PISOCH_ATTACH_BUFFERS *AttBuffers = (PISOCH_ATTACH_BUFFERS*)ioISO->BufferMemory;
        if (AttBuffers[0] != NULL) {
            LocalFree(AttBuffers[0]);
            AttBuffers[0] = NULL;
        }
        free(ioISO->BufferMemory);
        ioISO->BufferMemory = NULL;
    }
    if (ioISO->Buffers != NULL) {
        free(ioISO->Buffers);
        ioISO->Buffers = NULL;
    }
    if (ioISO->Times != NULL) {
        free(ioISO->Times);
        ioISO->Times = NULL;
    }
    if(ioISO->FIFO != NULL) {
        free(ioISO->FIFO);
        ioISO->FIFO = NULL;
    }
}

msw1394error_t msw1394_ISOCaptureSetup(msw1394_ISO* ioISO) {
    msw1394error_t res = MSW1394_SUCCESS;
    DWORD dwRet, dwBytesRet;
    PISOCH_ATTACH_BUFFERS *AttBuffers;
    OVERLAPPED AllocOverlapped;
    ULONG i, nb = ioISO->nNumberOfBuffers, bfsz;

    if ( ioISO->Port >= gNPorts)
        return MSW1394_INVALID_PORT;
    if (ioISO->hResource != INVALID_HANDLE_VALUE)
        return MSW1394_FAILURE;

    ioISO->hDevice =
        CreateFile( gDevice[ioISO->Port].path,
                    GENERIC_WRITE | GENERIC_READ,
                    FILE_SHARE_WRITE | FILE_SHARE_READ,
                    NULL,
                    OPEN_EXISTING,
                    FILE_FLAG_OVERLAPPED,
                    NULL
                    );
    if (ioISO->hDevice == INVALID_HANDLE_VALUE) {
        dwRet = GetLastError();
        res = ConvertError(dwRet);
        goto fail;
    }

    AllocOverlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    ioISO->CurrentBuffer = ioISO->QuedBuffers = ioISO->ReadyBuffers = 0;
    ISOCH_ALLOCATE_RESOURCES Res;
    Res.fulSpeed = SpeedFlag(ioISO->Speed);
    Res.fulFlags = RESOURCE_USED_IN_LISTENING;
    if (ioISO->nQuadletsToStrip != 0)
        Res.fulFlags |= RESOURCE_STRIP_ADDITIONAL_QUADLETS;
    Res.nChannel = ioISO->nChannel;
    Res.nMaxBytesPerFrame = ioISO->nMaxBytesPerFrame;
    Res.nNumberOfBuffers = ioISO->nNumberOfBuffers + 1;
    Res.nMaxBufferSize = ioISO->nMaxBufferSize;
    Res.nQuadletsToStrip = ioISO->nQuadletsToStrip;
    Res.hResource = NULL;

    dwRet =
        DeviceIoControl(
                        ioISO->hDevice,
                        IOCTL_ISOCH_ALLOCATE_RESOURCES,
                        &(Res),
                        sizeof(ISOCH_ALLOCATE_RESOURCES),
                        &(Res),
                        sizeof(ISOCH_ALLOCATE_RESOURCES),
                        &dwBytesRet,
                        &AllocOverlapped
                        );

    if (! dwRet) {
        dwRet = GetLastError();
        if (dwRet == ERROR_IO_PENDING) {
            dwRet = GetOverlappedResult(
                                        ioISO->hDevice,
                                        &AllocOverlapped,
                                        &dwBytesRet,
                                        TRUE
                                        );
            if (!dwRet)
                res = ConvertError(GetLastError());
        } else
            res = ConvertError(dwRet);
    }
    CloseHandle(AllocOverlapped.hEvent);
    ioISO->hResource = Res.hResource;

    if (res != MSW1394_SUCCESS)
        goto fail;

    ioISO->Overlapped = (OVERLAPPED*)calloc(1,sizeof(OVERLAPPED)*nb);
    if (ioISO->Overlapped == NULL) {
        res = MSW1394_NO_MEMORY;
        goto fail;
    }
    for (i=0; i<nb; i++) {
        ioISO->Overlapped[i].hEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
    if (ioISO->Overlapped[i].hEvent == NULL) {
        res = ConvertError(GetLastError());
        goto fail;
    }
    }

    ioISO->BufferMemory = (PISOCH_ATTACH_BUFFERS*)calloc(1,sizeof(PISOCH_ATTACH_BUFFERS)*nb);
    if (ioISO->BufferMemory == NULL) {
        res = MSW1394_NO_MEMORY;
        goto fail;
    }
    bfsz = sizeof(ISOCH_ATTACH_BUFFERS) + ioISO->nMaxBufferSize;
    AttBuffers = (PISOCH_ATTACH_BUFFERS*)ioISO->BufferMemory;
    AttBuffers[0] = (PISOCH_ATTACH_BUFFERS)LocalAlloc(LPTR, bfsz*nb);
    if (AttBuffers[0] == NULL) {
        dwRet = GetLastError();
        res = ConvertError(dwRet);
        goto fail;
    }

    ioISO->Buffers = (char**)calloc(1,sizeof(char*)*nb);
    if (ioISO->Buffers == NULL) {
        res = MSW1394_NO_MEMORY;
        goto fail;
    }

    ioISO->Times = (unsigned long long *)calloc(1,sizeof(unsigned long long)*nb);
    if (ioISO->Times == NULL) {
        res = MSW1394_NO_MEMORY;
        goto fail;
    }

    ioISO->FIFO = (ULONG *)calloc(1,sizeof(ULONG)*nb);
    if (ioISO->FIFO == NULL) {
        res = MSW1394_NO_MEMORY;
        goto fail;
    }

    ZeroMemory(AttBuffers[0], bfsz*nb);

    for (i = 1; i < nb; i++)
        AttBuffers[i] = (PISOCH_ATTACH_BUFFERS)
            ((char*)(AttBuffers[i-1])+bfsz);

    for (i = 0; i < nb; i++)
        ioISO->Buffers[i] = (char*)(AttBuffers[i]) +
            sizeof(ISOCH_ATTACH_BUFFERS);

    for (i = 0; i < nb; i++) {
        PISOCH_ATTACH_BUFFERS iab = AttBuffers[i];
        iab->hResource = ioISO->hResource;
        iab->nNumberOfDescriptors = 1;
        iab->ulBufferSize = bfsz;

        PRING3_ISOCH_DESCRIPTOR r3d = &iab->R3_IsochDescriptor[0];
        r3d->fulFlags = DESCRIPTOR_TIME_STAMP_ON_COMPLETION;
        r3d->ulLength = ioISO->nMaxBufferSize;
        r3d->nMaxBytesPerFrame = ioISO->nMaxBytesPerFrame;
        r3d->bAutoDetach = TRUE;
        r3d->bUseCallback = TRUE;

        ResetEvent(ioISO->Overlapped[i].hEvent);

        dwRet = DeviceIoControl(
                                ioISO->hDevice,
                                IOCTL_ISOCH_ATTACH_BUFFERS,
                                AttBuffers[i],
                                bfsz,
                                AttBuffers[i],
                                bfsz,
                                &dwBytesRet,
                                &(ioISO->Overlapped[i])
                                );

        if (!dwRet) {
            dwRet = GetLastError();
            if ((dwRet != ERROR_IO_PENDING) && (dwRet != ERROR_SUCCESS)) {
                res = ConvertError(dwRet);
                goto fail;
            }
        }
        ioISO->FIFO[ioISO->QuedBuffers++] = i;
    }

    ISOCH_LISTEN    isochListen;
    OVERLAPPED      ListenOverlapped;

    ListenOverlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    isochListen.hResource = ioISO->hResource;
    isochListen.fulFlags = 0;
    isochListen.StartTime.CL_SecondCount = 0;
    isochListen.StartTime.CL_CycleCount = 0;
    isochListen.StartTime.CL_CycleOffset = 0;

    dwRet = DeviceIoControl(
                            ioISO->hDevice,
                            IOCTL_ISOCH_LISTEN,
                            &isochListen,
                            sizeof(ISOCH_LISTEN),
                            NULL,
                            0,
                            &dwBytesRet,
                            &ListenOverlapped
                            );

    if (!dwRet) {
        dwRet = GetLastError();
        if (dwRet == ERROR_IO_PENDING) {
            dwRet = GetOverlappedResult(
                                        ioISO->hDevice,
                                        &ListenOverlapped,
                                        &dwBytesRet,
                                        TRUE
                                        );
            if (!dwRet)
                res = ConvertError(GetLastError());
        } else
            res = ConvertError(dwRet);
    }

    CloseHandle(ListenOverlapped.hEvent);

    if (res != MSW1394_SUCCESS)
        goto fail;

    return MSW1394_SUCCESS;

 fail:
    assert(res != MSW1394_SUCCESS);
    ISORecCleanup(ioISO);
    return res;
}

msw1394error_t msw1394_ISOCaptureStop(msw1394_ISO* ioISO) {
    msw1394error_t res = MSW1394_SUCCESS;
    DWORD dwRet, dwBytesRet;
    ISOCH_STOP isochStop;
    OVERLAPPED StopOverlapped;
    PISOCH_ATTACH_BUFFERS *AttBuffers = (PISOCH_ATTACH_BUFFERS*)ioISO->BufferMemory;

    dwRet = CancelIo(ioISO->hDevice);
    if (!dwRet) {
        dwRet = GetLastError();
        dc1394_log_error("CancelIo failed");
    }

    StopOverlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    isochStop.hResource = ioISO->hResource;
    isochStop.fulFlags = 0;

    dwRet = DeviceIoControl(
                            ioISO->hDevice,
                            IOCTL_ISOCH_STOP,
                            &isochStop,
                            sizeof(ISOCH_STOP),
                            NULL,
                            0,
                            &dwBytesRet,
                            &StopOverlapped
                            );

    if (!dwRet) {
        dwRet = GetLastError();
        if (dwRet == ERROR_IO_PENDING) {
            dwRet = GetOverlappedResult(
                                        ioISO->hDevice,
                                        &StopOverlapped,
                                        &dwBytesRet,
                                        TRUE
                                        );
            if (!dwRet)
                res = ConvertError(GetLastError());
        } else
            res = ConvertError(dwRet);
    }

    CloseHandle(StopOverlapped.hEvent);
    if (res != MSW1394_SUCCESS)
        return res;


    ISORecCleanup(ioISO);
    return MSW1394_SUCCESS;
}

msw1394error_t msw1394_ISOCaptureDequeue(msw1394_ISO* ioISO, int iWait, ULONG *oIdx) {
    DWORD dwRet, dwBytesRet;
    int cb;
    PISOCH_ATTACH_BUFFERS *AttBuffers = (PISOCH_ATTACH_BUFFERS*)ioISO->BufferMemory;

    while (1) {
        DWORD timeout = (iWait && ioISO->ReadyBuffers == 0)?INFINITE:0;
        cb = ioISO->FIFO[(ioISO->CurrentBuffer+ioISO->ReadyBuffers)%ioISO->nNumberOfBuffers];
        dwRet = WaitForSingleObject(ioISO->Overlapped[cb].hEvent,timeout);
        if (dwRet != WAIT_OBJECT_0) {
            if (dwRet == WAIT_TIMEOUT)
                break;
            else
                return ConvertError(GetLastError());
        }

        if (!GetOverlappedResult(ioISO->hDevice, &ioISO->Overlapped[cb], &dwBytesRet, TRUE)) {
            dwRet = GetLastError();
            return ConvertError(dwRet);
        }
        ioISO->Times[cb] = *(unsigned long long *)&(AttBuffers[cb]->R3_IsochDescriptor[0].CycleTime);
        ioISO->ReadyBuffers++;
        if (ioISO->ReadyBuffers == ioISO->QuedBuffers)
            break;
    }
    if (ioISO->ReadyBuffers == 0)
        return MSW1394_NO_DATA_AVAILABLE;

    *oIdx = ioISO->CurrentBuffer;
    ioISO->CurrentBuffer = (ioISO->CurrentBuffer+1)%ioISO->nNumberOfBuffers;
    ioISO->ReadyBuffers--;
    ioISO->QuedBuffers--;

    return MSW1394_SUCCESS;
}

msw1394error_t msw1394_ISOCaptureEnqueue(msw1394_ISO* ioISO, ULONG iIdx) {
    msw1394error_t res = MSW1394_SUCCESS;
    DWORD dwRet, dwBytesRet;
    PISOCH_ATTACH_BUFFERS *AttBuffers = (PISOCH_ATTACH_BUFFERS*)ioISO->BufferMemory;
    if (iIdx >= ioISO->nNumberOfBuffers)
        return MSW1394_FAILURE;

    if (ioISO->QuedBuffers == ioISO->nNumberOfBuffers)
        return MSW1394_NO_MEMORY;
    ResetEvent(ioISO->Overlapped[iIdx].hEvent);
    DeviceIoControl(
                    ioISO->hDevice,
                    IOCTL_ISOCH_ATTACH_BUFFERS,
                    AttBuffers[iIdx],
                    AttBuffers[iIdx]->ulBufferSize,
                    AttBuffers[iIdx],
                    AttBuffers[iIdx]->ulBufferSize,
                    &dwBytesRet,
                    &ioISO->Overlapped[iIdx]
                    );
    if (dwRet) {
        ioISO->FIFO[(ioISO->CurrentBuffer + ioISO->QuedBuffers)%ioISO->nNumberOfBuffers] = iIdx;
        ioISO->QuedBuffers++;
    }
    dwRet = GetLastError();
    if ((dwRet != ERROR_IO_PENDING) && (dwRet != ERROR_SUCCESS))
        res = ConvertError(dwRet);
    return res;
}

