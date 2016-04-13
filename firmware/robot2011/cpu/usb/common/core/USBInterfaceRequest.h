/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/**
 \unit

 !!!Purpose

    Definitions for manipulating SET_INTERFACE and GET_INTERFACE request.

 !!!Usage

    -# After a SET_INTERFACE request has been received, retrieve the
       target interface using USBInterfaceRequest_GetInterface and its
       new alternate setting with USBInterfaceRequest_GetAlternateSetting.
    -# After a GET_INTERFACE request has been received, retrieve the target
       interface using USBInterfaceRequest_GetInterface.

*/

#ifndef USBINTERFACEREQUEST_H
#define USBINTERFACEREQUEST_H

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include "USBGenericRequest.h"

//------------------------------------------------------------------------------
//         Exported functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Indicates which interface is targetted by a GET_INTERFACE or
/// SET_INTERFACE request.
/// \param request Pointer to a USBGenericRequest instance.
/// \return Interface number.
//------------------------------------------------------------------------------
static inline unsigned char USBInterfaceRequest_GetInterface(const USBGenericRequest *request)
{
    return (USBGenericRequest_GetIndex(request) & 0xFF);
}

//------------------------------------------------------------------------------
/// Indicates the new alternate setting that the interface targetted by a
/// SET_INTERFACE request should use.
/// \param request Pointer to a USBGenericRequest instance.
/// \return New active setting for the interface.
//------------------------------------------------------------------------------
static inline unsigned char USBInterfaceRequest_GetAlternateSetting(
    const USBGenericRequest *request)
{
    return (USBGenericRequest_GetValue(request) & 0xFF);
}

#endif //#ifndef USBINTERFACEREQUEST_H

