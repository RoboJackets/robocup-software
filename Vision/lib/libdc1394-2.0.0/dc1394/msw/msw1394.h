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

#ifndef msw1394_h
#define msw1394_h

#if defined( _X86_ ) && defined (__GNUC__)
#  undef _X86_
#endif

typedef enum {
  MSW1394_SUCCESS = 0,
  MSW1394_FAILURE,
  MSW1394_INVALID_PORT,
  MSW1394_NO_MEMORY,
  MSW1394_NO_DATA_AVAILABLE,
  MSW1394_INVALID_ARGUMENT,
  MSW1394_NO_SYSTEM_RESOURCES
} msw1394error_t;

typedef enum {
  MSW1394_SPEED100,
  MSW1394_SPEED200,
  MSW1394_SPEED400
} msw1394_speed_t;

typedef struct _msw1394_ISO {
  HANDLE  hDevice;
  ULONG Port;
  msw1394_speed_t Speed;
  ULONG nChannel;
  ULONG nMaxBytesPerFrame;
  ULONG nNumberOfBuffers;
  ULONG nMaxBufferSize;
  ULONG nQuadletsToStrip;
  HANDLE hResource;
  void *BufferMemory;
  char** Buffers;
  unsigned long long *Times;
  //PISOCH_ATTACH_BUFFERS *AttBuffers;
  OVERLAPPED *Overlapped;
  ULONG *FIFO;
  ULONG CurrentBuffer;
  ULONG QuedBuffers;
  ULONG ReadyBuffers;
} msw1394_ISO;

//
// 1394 Self ID packet format
//
typedef struct _sSELF_ID {
    ULONG               SID_Phys_ID:6;          // Byte 0 - Bits 0-5
    ULONG               SID_Packet_ID:2;        // Byte 0 - Bits 6-7
    ULONG               SID_Gap_Count:6;        // Byte 1 - Bits 0-5
    ULONG               SID_Link_Active:1;      // Byte 1 - Bit 6
    ULONG               SID_Zero:1;             // Byte 1 - Bit 7
    ULONG               SID_Power_Class:3;      // Byte 2 - Bits 0-2
    ULONG               SID_Contender:1;        // Byte 2 - Bit 3
    ULONG               SID_Delay:2;            // Byte 2 - Bits 4-5
    ULONG               SID_Speed:2;            // Byte 2 - Bits 6-7
    ULONG               SID_More_Packets:1;     // Byte 3 - Bit 0
    ULONG               SID_Initiated_Rst:1;    // Byte 3 - Bit 1
    ULONG               SID_Port3:2;            // Byte 3 - Bits 2-3
    ULONG               SID_Port2:2;            // Byte 3 - Bits 4-5
    ULONG               SID_Port1:2;            // Byte 3 - Bits 6-7
} sSELF_ID;

//
// 1394 Topology Map format
//
typedef struct _sTOPOLOGY_MAP {
    USHORT              TOP_Length;             // number of quadlets in map
    USHORT              TOP_CRC;                // 16 bit CRC defined by 1212
    ULONG               TOP_Generation;         // Generation number
    USHORT              TOP_Node_Count;         // Node count
    USHORT              TOP_Self_ID_Count;      // Number of Self IDs
    sSELF_ID             TOP_Self_ID_Array[];    // Array of Self IDs
} sTOPOLOGY_MAP;

//
// 1394 Cycle Time format
//
typedef struct _sCYCLE_TIME {
    ULONG               CL_CycleOffset:12;      // Bits 0-11
    ULONG               CL_CycleCount:13;       // Bits 12-24
    ULONG               CL_SecondCount:7;       // Bits 25-31
} sCYCLE_TIME;

typedef void (*BusResetHandler_t)(ULONG port);

#ifdef __cplusplus
extern "C" {
#endif

int msw1394_IsInited();
msw1394error_t msw1394_Init();
msw1394error_t msw1394_GetNumPorts(ULONG *oPort);
msw1394error_t msw1394_GetNodeCount(ULONG iPort, ULONG *numNodes);
msw1394error_t msw1394_AddBusResetHandler(BusResetHandler_t iH);
msw1394error_t msw1394_RemoveBusResetHandler(BusResetHandler_t iH);
msw1394error_t msw1394_ResetBus(ULONG iPort);
msw1394error_t msw1394_GetLocalHostMap(ULONG iPort, sTOPOLOGY_MAP **oMap);
msw1394error_t msw1394_GetLocalId(ULONG iPort, USHORT* oNode);
msw1394error_t msw1394_GetCycleTime(ULONG iPort, sCYCLE_TIME *oCycleTime);
msw1394error_t msw1394_ReadSync(ULONG iPort, USHORT node, ULONGLONG addr, size_t length, void *data);
msw1394error_t msw1394_WriteSync(ULONG iPort, USHORT node, ULONGLONG addr, size_t length, void *data);
msw1394error_t msw1394_ISOAllocChan(ULONG iPort, char *ioCh);
msw1394error_t msw1394_ISOFreeChan(ULONG iPort, char iCh);
msw1394error_t msw1394_ISOAllocBandwidth(int iPort, USHORT iBW, HANDLE *oBwH);
msw1394error_t msw1394_ISOFreeBandwidth(int iPort, HANDLE iBwH);
msw1394error_t msw1394_ISOCaptureSetup(msw1394_ISO* ioISO);
msw1394error_t msw1394_ISOCaptureStop(msw1394_ISO* ioISO);
msw1394error_t msw1394_ISOCaptureDequeue(msw1394_ISO* ioISO, int iWait, ULONG *oIdx);
msw1394error_t msw1394_ISOCaptureEnqueue(msw1394_ISO* ioISO, ULONG iIdx);

#ifdef __cplusplus
}
#endif

#endif

