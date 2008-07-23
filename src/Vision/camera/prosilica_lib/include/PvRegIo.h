/*
|==============================================================================
| Copyright (C) 2006-2007 Prosilica.  All Rights Reserved.
|
| Redistribution of this header file, in original or modified form, without
| prior written consent of Prosilica is prohibited.
|
|=============================================================================
|
| File:         PvRegIo.h
|
| Project/lib:  PvAPI
|
| Target:       Win32, Linux, QNX
|
| Description:  Camera register I/O.
|
|  Notes:       These functions can cause corruption of the internal state of
|               PvAPI.  These functions should be used as advised by Prosilica
|               engineering.
|
|==============================================================================
|
| THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
| WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
| NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
| DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
| INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
| LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
| OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED  AND ON ANY THEORY OF
| LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
| NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
| EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
|
|==============================================================================
| dd/mon/yy     Notes
|------------------------------------------------------------------------------
| 10/Feb/06     Original.
| 04/Jun/07     Licence changes
|==============================================================================
*/

#ifndef PVREGIO_H_INCLUDE
#define PVREGIO_H_INCLUDE

#ifdef __cplusplus
extern "C" {
#endif

//===== INCLUDE FILES =========================================================

#include <PvApi.h>

//===== #DEFINES ==============================================================

#define PvRegisterRead  _Pv_Factory_Test_10
#define PvRegisterWrite _Pv_Factory_Test_11
#define PvMemoryRead	_Pv_Factory_Test_12
#define PvMemoryWrite	_Pv_Factory_Test_13

//===== TYPE DEFINITIONS ======================================================

//===== FUNCTION PROTOTYPES ===================================================

/*
 * Function:  PvRegisterRead()
 *
 * Purpose:   Read camera registers.  A backdoor command.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,                      Handle to the camera 
 * [ IN] unsigned long NumReads                 Number of registers to read
 * [ IN] const unsigned long* pAddressArray,    Array of addresses, NumReads long
 * [OUT] unsigned long* pDataArray,             Data is returned here; array
 *                                                must be NumReads long
 * [OUT] unsigned long* pNumComplete,           Number of registers successfully
 *                                                read; if an error occurs this
 *                                                is less than NumReads; may be
 *                                                NULL
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvBadParameter,       NumReads is incorrect
 *               pPvAccessDenied,       read is not permitted (or register does
 *                                        not exist)
 */
tPvErr PVDECL _Pv_Factory_Test_10(tPvHandle Camera,
                                  unsigned long NumReads,
                                  const unsigned long* pAddressArray,
                                  unsigned long* pDataArray,
                                  unsigned long* pNumComplete);


/*
 * Function:  PvRegisterWrite()
 *
 * Purpose:   Write camera registers.  A backdoor command.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,                      Handle to the camera 
 * [ IN] unsigned long NumWrites                Number of registers to write
 * [ IN] const unsigned long* pAddressArray,    Array of addresses, NumWrites long
 * [ IN] const unsigned long* pDataArray,       Array of data, NumWrites long
 * [OUT] unsigned long* pNumComplete,           Number of registers successfully
 *                                                written; if an error occurs this
 *                                                is less than NumWrites; may be
 *                                                NULL
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvBadParameter,       NumWrites is incorrect
 *               pPvAccessDenied,       write is not permitted (or register does
 *                                        not exist)
 */
tPvErr PVDECL _Pv_Factory_Test_11(tPvHandle Camera,
                                  unsigned long NumWrites,
                                  const unsigned long* pAddressArray,
                                  const unsigned long* pDataArray,
                                  unsigned long* pNumComplete);



/*
 * Function:  PvMemoryRead()
 *
 * Purpose:   Read camera memory (READMEM_CMD).  A backdoor command.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,                      Handle to the camera 
 * [ IN] unsigned long Address,                 Address
 * [ IN] unsigned long Size,                    Size of read in bytes; must
 *                                                be a multiple of 4
 * [OUT] unsigned char* pDataBuffer             Data is returned here; buffer
 *                                                must be Size bytes long
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvBadParameter,       Size is incorrect
 *               pPvAccessDenied,       read is not permitted (or register does
 *                                        not exist)
 */
tPvErr PVDECL _Pv_Factory_Test_12(tPvHandle Camera,
                                  unsigned long Address,
                                  unsigned long Size,
                                  unsigned char* pDataBuffer);


/*
 * Function:  PvMemoryWrite()
 *
 * Purpose:   Write camera memory (WRITEMEM_CMD).  A backdoor command.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,                      Handle to the camera 
 * [ IN] unsigned long Address,                 Address
 * [ IN] unsigned long Size,                    Size of write in bytes; must
 *                                                be a multiple of 4
 * [ IN] const unsigned char* pDataBuffer,      Data buffer, Size bytes long
 * [OUT] unsigned long* pSizeComplete,          Number of bytes successfully
 *                                                written; if an error occurs this
 *                                                is less than Size; may be
 *                                                NULL
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvBadParameter,       Size is incorrect
 *               pPvAccessDenied,       write is not permitted (or register does
 *                                        not exist)
 */
tPvErr PVDECL _Pv_Factory_Test_13(tPvHandle Camera,
                                  unsigned long Address,
                                  unsigned long Size,
                                  const unsigned char* pDataBuffer,
                                  unsigned long* pSizeComplete);


//===== DATA ==================================================================


#ifdef __cplusplus
}
#endif

#endif // PVREGIO_H_INCLUDE

