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

    Definition of a class for manipulating CLEAR_FEATURE and SET_FEATURE
    requests.

 !!!Usage

    - To get USB feature request information (field values) from the
      USBGenericRequest instance, use
       - USBFeatureRequest_GetFeatureSelector
       - USBFeatureRequest_GetTestSelector
*/

#ifndef USBFEATUREREQUEST_H
#define USBFEATUREREQUEST_H

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include "USBGenericRequest.h"

//------------------------------------------------------------------------------
//         Definitions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// \page "USB Feature Request definitions"
///
/// This page lists codes of USB Feature Request
///
/// - USB Feature selectors
///    - USBFeatureRequest_ENDPOINTHALT
///    - USBFeatureRequest_DEVICEREMOTEWAKEUP
///    - USBFeatureRequest_TESTMODE
///
/// - USB Test mode selectors
///    - USBFeatureRequest_TESTJ
///    - USBFeatureRequest_TESTK
///    - USBFeatureRequest_TESTSE0NAK
///    - USBFeatureRequest_TESTPACKET
///    - USBFeatureRequest_TESTFORCEENABLE
///    - USBFeatureRequest_TESTSENDZLP
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// \page "USB Feature selectors"
///
/// This page lists codes of USB feature selectors.
///
/// !Selectors
/// - USBFeatureRequest_ENDPOINTHALT
/// - USBFeatureRequest_DEVICEREMOTEWAKEUP
/// - USBFeatureRequest_TESTMODE

/// Halt feature of an endpoint.
#define USBFeatureRequest_ENDPOINTHALT          0
/// Remote wake-up feature of the device.
#define USBFeatureRequest_DEVICEREMOTEWAKEUP    1
/// Test mode of the device.
#define USBFeatureRequest_TESTMODE              2
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// \page "USB Test mode selectors"
///
/// This page lists codes of USB high speed test mode selectors.
///
/// !Selectors
/// - USBFeatureRequest_TESTJ
/// - USBFeatureRequest_TESTK
/// - USBFeatureRequest_TESTSE0NAK
/// - USBFeatureRequest_TESTPACKET
/// - USBFeatureRequest_TESTFORCEENABLE
/// - USBFeatureRequest_TESTSENDZLP

/// Tests the high-output drive level on the D+ line.
#define USBFeatureRequest_TESTJ                 1
/// Tests the high-output drive level on the D- line.
#define USBFeatureRequest_TESTK                 2
/// Tests the output impedance, low-level output voltage and loading
/// characteristics.
#define USBFeatureRequest_TESTSE0NAK            3
/// Tests rise and fall times, eye patterns and jitter.
#define USBFeatureRequest_TESTPACKET            4
/// Tests the hub disconnect detection.
#define USBFeatureRequest_TESTFORCEENABLE       5
/// Send a ZLP in Test Mode.
#define USBFeatureRequest_TESTSENDZLP           6
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//         Exported functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Returns the feature selector of a given CLEAR_FEATURE or SET_FEATURE
/// request.
/// \param request Pointer to a USBGenericRequest instance.
/// \return Feature selector.
//------------------------------------------------------------------------------
static inline unsigned char USBFeatureRequest_GetFeatureSelector(
    const USBGenericRequest *request)
{
    return USBGenericRequest_GetValue(request);
}

//------------------------------------------------------------------------------
/// Indicates the test that the device must undertake following a
/// SET_FEATURE request.
/// \param request Pointer to a USBGenericRequest instance.
/// \return Test selector.
//------------------------------------------------------------------------------
static inline unsigned char USBFeatureRequest_GetTestSelector(
    const USBGenericRequest *request)
{
    return (USBGenericRequest_GetIndex(request) >> 8) & 0xFF;
}

#endif //#ifndef USBFEATUREREQUEST_H

