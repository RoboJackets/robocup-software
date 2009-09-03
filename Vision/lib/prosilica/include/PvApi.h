/*
|==============================================================================
| Copyright (C) 2006-2007 Prosilica.  All Rights Reserved.
|
| Redistribution of this header file, in original or modified form, without
| prior written consent of Prosilica is prohibited.
|
|=============================================================================
|
| File:         PvApi.h
|
| Project/lib:  PvAPI
|
| Target:       Win32, Linux, QNX
|
| Description:  Main header file for PvAPI, the programming interface for
|               Prosilica's GigE and Firewire cameras.
|
| Notes:        GigE support is available now; firewire support to follow.
|
|------------------------------------------------------------------------------
|
| Here are the basic functions:
|
|       PvInitialize()          - load and initialize the PvApi module
|       PvUnInitialize()        - shut down the module
|
|       PvCameraList()          - list the cameras available
|
|       PvCameraOpen()          - open & close your cameras
|       PvCameraClose()
|
|       PvCaptureStart()        - start & end image capture
|       PvCaptureEnd()
|
|       PvCaptureQueueFrame()   - queue a frame buffer for image capture
|
|       PvCaptureQueueClear()   - clear all frames off the queue (i.e. abort)
|
|       PvAttrList()            - list all attributes for this camera
|       PvAttrInfo()            - get information on an attribute
|
| Camera attributes are used to represent controls such as acquisition mode or
| shutter exposure time.  Camera attributes are also used to represent constant
| values like serial numbers, or read-only values like bytes-per-frame.
|
| A few basic attribute datatypes are supported, such as 32-bit unsigned,
| enumerated set, and float.
|
| Here are the basic attributes common to all cameras:
|
|       AcquisitionMode         - idle, or continuous acquisition
|       Width                   - image width in pixels
|       Height                  - image height in pixels
|       PixelFormat             - image data format (ex. mono8, mono16)
|       TotalBytesPerFrame      - number of bytes per image
|
| Most attributes are specific to each camera model and revision, and are
| documented elsewhere.
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
| 27/Jun/05     Original.
| 04/Jun/07     Licence changes
| 27/Jun/07     Added function PvCaptureAdjustPacketSize
| 09/Jul/07     Added error code ePvErrFirewall
| 17/Aug/07     Added new pixel formats
|==============================================================================
*/

#ifndef PVAPI_H_INCLUDE
#define PVAPI_H_INCLUDE

#ifdef __cplusplus
extern "C" {
#endif

//===== INCLUDE FILES =========================================================

//===== #DEFINES ==============================================================

#ifndef PVDECL
        #ifdef _MSC_VER
                #define PVDECL           __stdcall
        #else
                #if defined(_LINUX) || defined(_QNX)
                        #define PVDECL
                #else
                        #error Define PVDECL to be your compiler keyword for "standard call"
                #endif
        #endif
#endif

#define PVINFINITE      0xFFFFFFFF  // Never timeout


//===== TYPE DEFINITIONS ======================================================

typedef void*           tPvHandle;  // Camera handle


//
// Error codes, returned by most functions:
//
typedef enum
{
    ePvErrSuccess       = 0,        // No error
    ePvErrCameraFault   = 1,        // Unexpected camera fault
    ePvErrInternalFault = 2,        // Unexpected fault in PvApi or driver
    ePvErrBadHandle     = 3,        // Camera handle is invalid
    ePvErrBadParameter  = 4,        // Bad parameter to API call
    ePvErrBadSequence   = 5,        // Sequence of API calls is incorrect
    ePvErrNotFound      = 6,        // Camera or attribute not found
    ePvErrAccessDenied  = 7,        // Camera cannot be opened in the specified mode
    ePvErrUnplugged     = 8,        // Camera was unplugged
    ePvErrInvalidSetup  = 9,        // Setup is invalid (an attribute is invalid)
    ePvErrResources     = 10,       // System/network resources or memory not available
    ePvErrBandwidth     = 11,       // 1394 bandwidth not available
    ePvErrQueueFull     = 12,       // Too many frames on queue
    ePvErrBufferTooSmall= 13,       // Frame buffer is too small
    ePvErrCancelled     = 14,       // Frame cancelled by user
    ePvErrDataLost      = 15,       // The data for the frame was lost
    ePvErrDataMissing   = 16,       // Some data in the frame is missing
    ePvErrTimeout       = 17,       // Timeout during wait
    ePvErrOutOfRange    = 18,       // Attribute value is out of the expected range
    ePvErrWrongType     = 19,       // Attribute is not this type (wrong access function) 
    ePvErrForbidden     = 20,       // Attribute write forbidden at this time
    ePvErrUnavailable   = 21,       // Attribute is not available at this time
    ePvErrFirewall      = 22,       // A firewall is blocking the traffic (Windows only)
    __ePvErr_force_32   = 0xFFFFFFFF

} tPvErr;


//----- Camera Enumeration & Information --------------------------------------

//
// Camera access mode.  Used as flags in tPvCameraInfo data, and as the access
// mode in PvOpenCamera().
//
typedef enum
{
    ePvAccessMonitor        = 2, // Monitor access: no control, read & listen only
    ePvAccessMaster         = 4, // Master access: full control
    __ePvAccess_force_32    = 0xFFFFFFFF

} tPvAccessFlags;


//
// Camera interface type (i.e. firewire, ethernet):
//
typedef enum
{
    ePvInterfaceFirewire    = 1,
    ePvInterfaceEthernet    = 2,
    __ePvInterface_force_32 = 0xFFFFFFFF

} tPvInterface;


//
// Camera information type.
//
typedef struct
{
    unsigned long       UniqueId;         // Unique value for each camera
    char                SerialString[32]; // Camera's serial number
    unsigned long       PartNumber;       // Camera part number
    unsigned long       PartVersion;      // Camera part version
    unsigned long       PermittedAccess;  // A combination of tPvAccessFlags
    unsigned long       InterfaceId;      // Unique value for each interface or bus
    tPvInterface        InterfaceType;    // Interface type; see tPvInterface
    char                DisplayName[16];  // People-friendly camera name
    unsigned long       _reserved[4];     // Always zero

} tPvCameraInfo;


//
// IP configuration mode for ethernet cameras.
//
typedef enum
{
    ePvIpConfigPersistent   = 1,            // Use persistent IP settings
    ePvIpConfigDhcp         = 2,            // Use DHCP, fallback to AutoIP
    ePvIpConfigAutoIp       = 4,            // Use AutoIP only
    __ePvIpConfig_force_32  = 0xFFFFFFFF

} tPvIpConfig;


//
// Structure used for PvCameraIpSettingsGet() and PvCameraIpSettingsChange().
//
typedef struct
{
    // IP configuration mode: persistent, DHCP & AutoIp, or AutoIp only.
    tPvIpConfig         ConfigMode;
    // IP configuration mode supported by the camera
    unsigned long       ConfigModeSupport;

    // Current IP configuration.  Ignored for PvCameraIpSettingsChange().  All
    // values are in network byte order (i.e. big endian).
    unsigned long       CurrentIpAddress;
    unsigned long       CurrentIpSubnet;
    unsigned long       CurrentIpGateway;

    // Persistent IP configuration.  See "ConfigMode" to enable persistent IP
    // settings.  All values are in network byte order.
    unsigned long       PersistentIpAddr;
    unsigned long       PersistentIpSubnet;
    unsigned long       PersistentIpGateway;

    unsigned long       _reserved1[8];

} tPvIpSettings;


//----- Interface-Link Callback -----------------------------------------------

//
// Link (aka interface) event type
//
typedef enum
{
    ePvLinkAdd          = 1, // A camera was plugged in
    ePvLinkRemove       = 2, // A camera was unplugged
    _ePvLink_reserved1  = 3, 
    __ePvLink_force_32  = 0xFFFFFFFF

} tPvLinkEvent;


//
// Link (aka interface) event Callback type
//
// Arguments:
//
//  [i] void* Context,          Context, as provided to PvLinkCallbackRegister
//  [i] tPvInterface Interface, Interface on which the event occurred
//  [i] tPvLinkEvent Event,     Event which occurred
//  [i] unsigned long UniqueId, Unique ID of the camera related to the event
//
typedef void (PVDECL *tPvLinkCallback)(void* Context,
                                       tPvInterface Interface,
                                       tPvLinkEvent Event,
                                       unsigned long UniqueId);


//----- Image Capture ---------------------------------------------------------

//
// Frame image format type
//
typedef enum
{
    ePvFmtMono8         = 0,            // Monochrome, 8 bits
    ePvFmtMono16        = 1,            // Monochrome, 16 bits, data is LSB aligned
    ePvFmtBayer8        = 2,            // Bayer-color, 8 bits
    ePvFmtBayer16       = 3,            // Bayer-color, 16 bits, data is LSB aligned
    ePvFmtRgb24         = 4,            // RGB, 8 bits x 3
    ePvFmtRgb48         = 5,            // RGB, 16 bits x 3, data is LSB aligned
    ePvFmtYuv411        = 6,            // YUV 411
    ePvFmtYuv422        = 7,            // YUV 422
    ePvFmtYuv444        = 8,            // YUV 444
    ePvFmtBgr24         = 9,            // BGR, 8 bits x 3
    ePvFmtRgba32        = 10,           // RGBA, 8 bits x 4
    ePvFmtBgra32        = 11,           // BGRA, 8 bits x 4
    __ePvFmt_force_32   = 0xFFFFFFFF

} tPvImageFormat;


//
// Bayer pattern.  Applicable when a Bayer-color camera is sending raw bayer
// data.
//
typedef enum
{
    ePvBayerRGGB        = 0,            // First line RGRG, second line GBGB...
    ePvBayerGBRG        = 1,            // First line GBGB, second line RGRG...
    ePvBayerGRBG        = 2,            // First line GRGR, second line BGBG...
    ePvBayerBGGR        = 3,            // First line BGBG, second line GRGR...
    __ePvBayer_force_32 = 0xFFFFFFFF

} tPvBayerPattern;


//
// The frame structure passed to PvQueueFrame().
//
typedef struct
{
    //----- In -----
    void*               ImageBuffer;        // Your image buffer
    unsigned long       ImageBufferSize;    // Size of your image buffer in bytes

    void*               AncillaryBuffer;    // Your buffer to capture associated 
                                            //   header & trailer data for this image.
    unsigned long       AncillaryBufferSize;// Size of your ancillary buffer in bytes
                                            //   (can be 0 for no buffer).

    void*               Context[4];         // For your use (valuable for your
                                            //   frame-done callback).
    unsigned long       _reserved1[8];

    //----- Out -----

    tPvErr              Status;             // Status of this frame

    unsigned long       ImageSize;          // Image size, in bytes
    unsigned long       AncillarySize;      // Ancillary data size, in bytes

    unsigned long       Width;              // Image width
    unsigned long       Height;             // Image height
    unsigned long       RegionX;            // Start of readout region (left)
    unsigned long       RegionY;            // Start of readout region (top)
    tPvImageFormat      Format;             // Image format
    unsigned long       BitDepth;           // Number of significant bits
    tPvBayerPattern     BayerPattern;       // Bayer pattern, if bayer format

    unsigned long       FrameCount;         // Rolling frame counter
    unsigned long       TimestampLo;        // Time stamp, lower 32-bits
    unsigned long       TimestampHi;        // Time stamp, upper 32-bits

    unsigned long       _reserved2[32];

} tPvFrame;


//
// Frame Callback type
//
// Arguments:
//
//  [i] tPvFrame* Frame, Frame completed
//
typedef void (PVDECL *tPvFrameCallback)(tPvFrame* Frame);


//----- Attributes ------------------------------------------------------------


#if defined(_M_IX86) || defined(_x86) || defined(_WIN64) || defined(_x64)
typedef long            tPvInt32;   // 32-bit integer
typedef unsigned long   tPvUint32;  // 32-bit unsigned integer
typedef float           tPvFloat32; // IEEE 32-bit float
#elif defined(_ppc)
typedef long            tPvInt32;   // 32-bit integer
typedef unsigned long   tPvUint32;  // 32-bit unsigned integer
typedef float           tPvFloat32; // IEEE 32-bit float
#else
#error Define specific data types for your platform.
#endif


//
// List of attributes, used by PvAttrList.  This is an array of string
// pointers.  The array, and all the string pointers, are const.
//
typedef const char* const* tPvAttrListPtr;


//
// Attribute data type supported
//
typedef enum
{
    ePvDatatypeUnknown  = 0,
    ePvDatatypeCommand  = 1,
    ePvDatatypeRaw      = 2,
    ePvDatatypeString   = 3,
    ePvDatatypeEnum     = 4,
    ePvDatatypeUint32   = 5,
    ePvDatatypeFloat32  = 6,
    __ePvDatatypeforce_32= 0xFFFFFFFF

} tPvDatatype;


//
// Attribute flags type
//
typedef enum
{
    ePvFlagRead         = 0x01,     // Read access is permitted
    ePvFlagWrite        = 0x02,     // Write access is permitted
    ePvFlagVolatile     = 0x04,     // The camera may change the value any time
    ePvFlagConst        = 0x08,     // Value is read only and never changes
    __ePvFlag_force_32  = 0xFFFFFFFF

} tPvAttributeFlags;


//
// Attribute information type
//
typedef struct
{
    tPvDatatype         Datatype;       // Data type
    unsigned long       Flags;          // Combination of tPvAttribute flags
    const char*         Category;       // Advanced: see documentation
    const char*         Impact;         // Advanced: see documentation
    unsigned long       _reserved[4];   // Always zero

} tPvAttributeInfo;


//===== FUNCTION PROTOTYPES ===================================================

//----- API Version -----------------------------------------------------------

/*
 * Function:  PvVersion()
 *
 * Purpose:   Retreive the version number of PvAPI
 *
 * Arguments: 
 *
 * [OUT] unsigned long* pMajor, major version number
 * [OUT] unsigned long* pMinor, minor version number
 *
 * Return:    none
 *
 * This function can be called at anytime, including before the API is
 * initialized.
 */
void PVDECL PvVersion(unsigned long* pMajor,unsigned long* pMinor);


//----- API Initialization ----------------------------------------------------

/*
 * Function:  PvInitialize()
 *
 * Purpose:   Initialize the PvApi module.  This must be called before any
 *            other PvApi function is run.
 *
 * Arguments: none
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *
 *               ePvErrResources,       resources requested from the OS were not
 *                                      available           
 *               ePvErrInternalFault,   an internal fault occurred
 */
tPvErr PVDECL PvInitialize(void);


/*
 * Function:  PvUnInitialize()
 *
 * Purpose:   Uninitialize the API module.  This will free some resources,
 *            and shut down network activity if applicable.
 *
 * Arguments: none
 *
 * Return:    none
 */
void PVDECL PvUnInitialize(void);


//----- Interface-Link Callback -----------------------------------------------

/*
 * Function:  PvLinkCallbackRegister()
 *
 * Purpose:   Register a callback for interface events.
 *
 * Arguments:
 *
 * [ IN] tPvLinkCallback Callback, Callback function to run when an event occurs
 * [ IN] tPvLinkEvent Event,       Event to trigger the callback
 * [ IN] void* Context,            For your use: Context is passed to your
 *                                   callback function
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *
 *               ePvErrResources,       resources requested from the OS were not
 *                                      available
 *               ePvErrBadSequence,     API isn't initialized
 *
 * Here's the rules:
 *   - Multiple callback functions may be registered with the same event
 *   - The same callback function may be shared by different events
 *   - The same callback function with the same event may not be registered twice
 *
 * The callback functions are called from a thread within PvApi.  The callbacks
 * are sequenced; i.e. they will not be called simultaneously.
 *
 * Use PvLinkCallbackUnRegister() to stop receiving callbacks.
 */
tPvErr PVDECL PvLinkCallbackRegister(tPvLinkCallback Callback,
                                     tPvLinkEvent Event,
                                     void* Context);


/*
 * Function:  PvLinkCallbackUnRegister()
 *
 * Purpose:   Unregister a callback for interface events.
 *
 * Arguments:
 *
 * [ IN] tPvLinkCallback Callback, Callback function previously registered
 * [ IN] tPvLinkEvent Event,       Event associated with the callback
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *
 *               ePvErrNotFound,        registered callback was not found
 *               ePvErrResources,       resources requested from the OS were not
 *                                      available
 *               ePvErrBadSequence,     API isn't initialized
 */
tPvErr PVDECL PvLinkCallbackUnRegister(tPvLinkCallback Callback,
                                       tPvLinkEvent Event);


//----- Camera Enumeration & Information --------------------------------------

/*
 * Function:  PvCameraList()
 *
 * Purpose:   List all the cameras currently visible to PvApi
 *
 * Arguments:
 *
 * [OUT] tPvCameraInfo* pList,          Array of tPvCameraInfo, allocated by
 *                                        the caller.  The camera list is
 *	                                      copied here.
 * [ IN] unsigned long ListLength,      Length of the caller's pList array
 * [OUT] unsigned long* pConnectedNum,  Number of cameras found (may be more
 *                                        than ListLength!) returned here.
 *                                        May be NULL.
 *
 * Return:    Number of pList entries filled, up to ListLength.
 */
unsigned long PVDECL PvCameraList(tPvCameraInfo* pList,
                                  unsigned long ListLength,
                                  unsigned long* pConnectedNum);


/*
 * Function:  PvCameraCount()
 *
 * Purpose:   Number of cameras visible to PvApi (at the time of the call).
 *            Does not include unreachable cameras.
 *
 * Arguments: none
 *
 * Return:    The number of cameras found
 *
 * The number of cameras is dynamic, and may change at any time.
 */
unsigned long PVDECL PvCameraCount(void);


/*
 * Function:  PvCameraInfo()
 *
 * Purpose:   Retreive information on a given camera
 *
 * Arguments:
 *
 * [ IN] unsigned long UniqueId,    Unique ID of the camera
 * [OUT] tPvCameraInfo* pInfo,      Structure where the information will be
 *                                    copied
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *
 *               ePvErrNotFound,        the camera was not found (unplugged)
 *               ePvErrUnplugged,       the camera was found but unplugged during the
 *                                      function call
 *               ePvErrBadParameter,    a valid pointer for pInfo was not supplied
 *               ePvErrResources,       resources requested from the OS were not
 *                                      available
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 */
tPvErr PVDECL PvCameraInfo(unsigned long UniqueId, tPvCameraInfo* pInfo);


/*
 * Function:  PvCameraInfoByAddr()
 *
 * Purpose:   Retreive information on a camera, by IP address.  This function
 *            is required if the ethernet camera is not on the local ethernet
 *            network.
 *
 * Arguments:
 *
 * [ IN] unsigned long IpAddr,          IP address of camera, in network byte
 *                                        order.
 * [OUT] tPvCameraInfo* pInfo,          The camera information will be copied
 *                                        here.
 * [OUT] tPvIpSettings* pIpSettings,    The IP settings will be copied here;
 *                                        NULL pointer OK.
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *
 *               ePvErrNotFound,        the camera was not found
 *               ePvErrResources,       resources requested from the OS were not
 *                                      available
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 *               ePvErrBadParameter,    pIpSettings->size is invalid
 *
 * The specified camera may not be visible to PvCameraList(); it might be on a
 * different ethernet network.  In this case, communication with the camera is
 * routed to the local gateway.
 */
tPvErr PVDECL PvCameraInfoByAddr(unsigned long IpAddr,
                                 tPvCameraInfo* pInfo,
                                 tPvIpSettings* pIpSettings);

/*
 * Function:  PvCameraListUnreachable()
 *
 * Purpose:   List all the cameras currently inaccessable by PvApi.  This lists
 *            the ethernet cameras which are connected to the local ethernet
 *            network, but are on a different subnet.
 *
 * Arguments:
 *
 * [OUT] tPvCameraInfo* pList,          Array of tPvCameraInfo, allocated by
 *                                        the caller.  The camera list is
 *	                                      copied here.
 * [ IN] unsigned long ListLength,      Length of the caller's pList array
 * [OUT] unsigned long* pConnectedNum,  Number of cameras found (may be more
 *                                        than ListLength!) returned here.
 *                                        May be NULL.
 *
 * Return:    Number of pList entries filled, up to ListLength.
 */
unsigned long PVDECL PvCameraListUnreachable(tPvCameraInfo* pList,
                                             unsigned long ListLength,
                                             unsigned long* pConnectedNum);


//----- Opening & Closing -----------------------------------------------------

/*
 * Function:  PvCameraOpen()
 *
 * Purpose:   Open the specified camera.  This function must be called before
 *            you can control the camera.
 *
 * Arguments:
 *
 * [ IN] unsigned long UniqueId,    UniqueId of the camera
 * [ IN] tPvAccessFlags AccessFlag, Access flag {monitor, master}
 * [OUT] tPvHandle* pCamera,        Handle to the opened camera returned here
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *
 *               ePvErrAccessDenied,    the camera couldn't be open in the requested mode
 *               ePvErrNotFound,        the camera was not found (unplugged)
 *               ePvErrUnplugged,       the camera was found but unplugged during the
 *                                      function call
 *               ePvErrBadParameter,    a valid pointer for pCamera was not supplied
 *               ePvErrResources,       resources requested from the OS were not
 *                                      available
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized or camera is alreay open
 *
 * If ePvErrSuccess is returned, you must eventually call PvCameraClose().
 *
 * Alternatively, under special circumstances, you might open an ethernet
 * camera with PvCameraOpenByAddr().
 */
tPvErr PVDECL PvCameraOpen(unsigned long UniqueId,
                           tPvAccessFlags AccessFlag,
                           tPvHandle* pCamera);

/*
 * Function:  PvCameraOpenByAddr()
 *
 * Purpose:   Open the specified camera, by IP address.  This function is
 *            required, in place of PvCameraOpen(), if the ethernet camera
 *            is not on the local ethernet network.
 *
 * Arguments:
 *
 * [ IN] unsigned long IpAddr,          IP address of camera, in network byte
 *                                        order.
 * [ IN] tPvAccessFlags AccessFlag,     Access flag {monitor, master}
 * [OUT] tPvHandle* pCamera,            Handle to the opened camera returned here
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *
 *               ePvErrAccessDenied,    the camera couldn't be open in the requested mode
 *               ePvErrNotFound,        the camera was not found (unplugged)
 *               ePvErrUnplugged,       the camera was found but unplugged during the
 *                                      function call
 *               ePvErrBadParameter,    a valid pointer for pCamera was not supplied
 *               ePvErrResources,       resources requested from the OS were not
 *                                      available
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized or camera is alreay open
 *
 * If ePvErrSuccess is returned, you must eventually call PvCameraClose().
 *
 * The specified camera may not be visible to PvCameraList(); it might be on a
 * different ethernet network.  In this case, communication with the camera is
 * routed to the local gateway.
 */
tPvErr PVDECL PvCameraOpenByAddr(unsigned long IpAddr,
                                 tPvAccessFlags AccessFlag,
                                 tPvHandle* pCamera);

/*
 * Function:  PvCameraClose()
 *
 * Purpose:   Close the specified camera.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,      Handle of an opened camera
 *
 * Return:    ePvErrBadHandle if the handle is bad, otherwise likely to be any of the
 *            following error codes:
 *
 *               ePvErrBadHandle,    the handle of the camera is invalid
 *               ePvErrBadSequence,  API isn't initialized
 *
 */
tPvErr PVDECL PvCameraClose(tPvHandle Camera);


/*
 * Function:  PvCameraIpSettingsGet()
 *
 * Purpose:   Get IP settings for an ethernet camera.  This command will work
 *            for all cameras on the local ethernet network, including
 *            "unreachable" cameras.
 *
 * Arguments:
 *
 * [ IN] unsigned long UniqueId,    UniqueId of the camera
 * [OUT] tPvIpSettings* pSettings,  Camera settings are copied here
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *
 *               ePvErrNotFound,        the camera was not found (or is not
 *                                      an ethernet camera)
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 *               ePvBadParameter,       pSettings->size is incorrect
 *
 * The camera does not have to be opened to run this command.
 */
tPvErr PVDECL PvCameraIpSettingsGet(unsigned long UniqueId,
                                    tPvIpSettings* pSettings);


/*
 * Function:  PvCameraIpSettingsChange()
 *
 * Purpose:   Change the IP settings for an ethernet camera.  This command
 *            will work for all cameras on the local ethernet network,
 *            including "unreachable" cameras.
 *
 * Arguments:
 *
 * [ IN] unsigned long UniqueId,            UniqueId of the camera
 * [ IN] const tPvIpSettings* pSettings,    New camera settings
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *
 *               ePvErrNotFound,        the camera was not found
 *               ePvErrAccessDenied,    the camera was already open
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 *               ePvBadParameter,       pSettings->size is incorrect
 *
 * This command will fail if any application on any host has opened the camera.
 */
tPvErr PVDECL PvCameraIpSettingsChange(unsigned long UniqueId,
                                       const tPvIpSettings*	pSettings);


//----- Image Capture ---------------------------------------------------------

/*
 * Function:  PvCaptureStart()
 *
 * Purpose:   Setup the camera interface for image transfer.  This does not
 *            necessarily start acquisition.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,      Handle to the camera 
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvErrResources,       resources requested from the OS were not
 *                                      available
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized or capture already started
 *
 * PvCaptureStart() must be run before PvCaptureQueueFrame() is allowed.  But
 * the camera will not acquire images before the AcquisitionMode attribute is
 * set to a non-idle mode.
 */
tPvErr PVDECL PvCaptureStart(tPvHandle Camera);


/*
 * Function:  PvCaptureEnd()
 *
 * Purpose:   Disable the image transfer mechanism.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,      Handle to the camera 
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized or capture already stopped
 *
 * This cannot be called until the frame queue is empty.
 */
tPvErr PVDECL PvCaptureEnd(tPvHandle Camera);


/*
 * Function:  PvCaptureQuery()
 *
 * Purpose:   Check to see if a camera interface is ready to transfer images.
 *            I.e. has PvCaptureStart() been called?
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,          Handle to the camera 
 * [OUT] unsigned long* pIsStarted, Result returned here: 1=started, 0=disabled
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvBadParameter,       a valid pointer for pIsStarted was not supplied
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 */
tPvErr PVDECL PvCaptureQuery(tPvHandle Camera, unsigned long* pIsStarted);

/*
 * Function:  PvCaptureAdjustPacketSize()
 *
 * Purpose:   Determine the maximum packet size supported by the system.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,                   Handle to the camera 
 * [ IN] unsigned long MaximumPacketSize     Upper limit: the packet size will
 *                                             not be set higher than this value.
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvErrResources,       resources requested from the OS were not
 *                                      available
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized or capture already started
 *
 * The maximum packet size can be limited by the camera, host adapter, and
 * ethernet switch.
 *
 * PvCaptureAdjustPacketSize() cannot be run when capture has started.
 */
tPvErr PVDECL PvCaptureAdjustPacketSize(tPvHandle Camera,unsigned long MaximumPacketSize);

/*
 * Function:  PvCaptureQueueFrame()
 *
 * Purpose:   Queue a frame buffer for image capture.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,          Handle to the camera 
 * [ IN] tPvFrame* pFrame,          Frame to queue
 * [ IN] tPvFrameCallback Callback, Callback to run when the frame is done;
 *                                    may be NULL for no callback
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvErrQueueFull,       the frame queue is full
 *               ePvErrResources,       resources requested from the OS were not
 *                                      available
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized or capture not started
 *
 * This function returns immediately.  If ePvErrSuccess is returned, the frame
 * will remain in the queue until it is complete, or aborted due to an error or
 * a call to PvCaptureQueueClear().
 *
 * Frames are completed (or aborted) in the order they are queued.
 *
 * You can specify a callback function (the "frame-done callback") to occur
 * when the frame is complete, or you can use PvCaptureWaitForFrameDone() to
 * block until the frame is complete.
 *
 * When the frame callback starts, the tPvFrame data structure is no longer in
 * use and you are free to do with it as you please (for example, reuse or
 * deallocation.)
 *
 * Each frame on the queue must have a unique tPvFrame data structure.
 */
tPvErr PVDECL PvCaptureQueueFrame(tPvHandle Camera,
                                  tPvFrame* pFrame,
                                  tPvFrameCallback Callback);


/*
 * Function:  PvCaptureQueueClear()
 *
 * Purpose:   Empty the frame queue.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,          Handle to the camera 
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 *
 * Queued frames are returned with status ePvErrCancelled.
 *
 * PvCaptureQueueClear() cannot be called from the frame-done callback.
 *
 * When this function returns, no more frames are left on the queue and you
 * will not receive another frame callback.
 */
tPvErr PVDECL PvCaptureQueueClear(tPvHandle Camera);


/*
 * Function:  PvCaptureWaitForFrameDone()
 *
 * Purpose:   Wait for a frame capture to complete.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,          Handle to the camera 
 * [ IN] tPvFrame* pFrame,          Frame to wait upon
 * [ IN] unsigned long Timeout,     Wait timeout (in milliseconds); use
 *                                    PVINFINITE for no timeout
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrTimeout,         timeout while waiting for the frame
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 *
 * This function cannot be called from the frame-done callback.
 * 
 * When this function returns, the frame structure is no longer in use and you
 * are free to do with it as you please (for example, reuse or deallocation).
 *
 * If you are using the frame-done callback: this function might return first,
 * or the frame callback might be called first.
 *
 * If the specified frame is not on the queue, this function returns
 * ePvErrSuccess, since we do not know if the frame just left the queue.
 */
tPvErr PVDECL PvCaptureWaitForFrameDone(tPvHandle Camera,
                                        const tPvFrame* pFrame,
                                        unsigned long Timeout);


//----- Attributes ------------------------------------------------------------

/*
 * Function:  PvAttrList()
 *
 * Purpose:   List all the attributes for a given camera.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,          Handle to the camera
 * [OUT] tPvAttrListPtr* pListPtr   Pointer to the attribute list is returned
 *                                    here.  The attribute list is valid until
 *                                    the camera is closed.
 * [OUT] unsigned long* pLength     Length of the list is returned here.  The
 *                                    length never changes while the camera
 *                                    remains opened.
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvBadParameter,       a valid pointer for pListPtr was not supplied
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 *
 * The attribute list is contained in memory allocated by the PvApi module.
 * This memory remains static until the camera is closed.
 */
tPvErr PVDECL PvAttrList(tPvHandle Camera,
                         tPvAttrListPtr* pListPtr,
                         unsigned long* pLength);


/*
 * Function:  PvAttrInfo()
 *
 * Purpose:   Retrieve information on an attribute.  This information is
 *            static for each camera.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,          Handle to the camera 
 * [ IN] const char* Name,          Attribute name 
 * [OUT] tPvAttributeInfo* pInfo,   The information is copied here
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvErrNotFound,        the requested attribute doesn't exist
 *               ePvBadParameter,       a valid pointer for pInfo was not supplied
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 */
tPvErr PVDECL PvAttrInfo(tPvHandle Camera,
                         const char* Name,
                         tPvAttributeInfo* pInfo);


/*
 * Function:  PvAttrExists()
 *
 * Purpose:   Check if an attribute exists for the camera.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,          Handle to the camera 
 * [ IN] const char* Name,          Attribute name
 *
 * Return:       ePvErrSuccess,         the attribute exists
 *               ePvErrNotFound,        the attribute does not exist
 *
 *            The following error codes may also occur:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 */
tPvErr PVDECL PvAttrExists(tPvHandle Camera, const char* Name);


/*
 * Function:  PvAttrIsAvailable()
 *
 * Purpose:   Check if an attribute is available.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,              Handle to the camera 
 * [ IN] const char* Name,              Attribute name 
 *
 * Return:       ePvErrSuccess,         the attribute is available
 *               ePvErrUnavailable,     the attribute is not available
 *
 *            The following error codes may also occur:
 *
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvErrNotFound,        the requested attribute doesn't exist
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 */
tPvErr PVDECL PvAttrIsAvailable(tPvHandle Camera, const char* Name);


/*
 * Function:  PvAttrIsValid()
 *
 * Purpose:   Check if an attribute's value is valid.
 *
 * Arguments: 
 *
 * [ IN] tPvHandle Camera,          Handle to the camera 
 * [ IN] const char* Name,          Attribute name 
 *
 * Return:       ePvErrSuccess,         the attribute is valid
 *               ePvErrOutOfRange,      the attribute is not valid
 *
 *            The following error codes may also occur:
 *
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvErrOutOfRange,      the requested attribute is not valid
 *               ePvErrNotFound,        the requested attribute doesn't exist
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 */
tPvErr PVDECL PvAttrIsValid(tPvHandle Camera, const char* Name);


/*
 * Function:  PvAttrRangeEnum()
 *
 * Purpose:   Get the enumeration set for an enumerated attribute.  The set is
 *            returned as a comma separated string containing all allowed
 *            values.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,          Handle to the camera 
 * [ IN] const char* Name,          Attribute name 
 * [OUT] char* pBuffer,             Caller-allocated buffer; the string is
 *                                    copied here
 * [ IN] unsigned long BufferSize,  Size of buffer, in bytes
 * [OUT] unsigned long* pSize,      Size of the enumeration set string is
 *                                      returned here.  This may be bigger
 *                                      than BufferSize!  (pSize may be NULL.)
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvErrNotFound,        the requested attribute doesn't exist
 *               ePvErrWrongType,       the requested attribute is not of the correct type
 *               ePvBadParameter,       a valid pointer for pBuffer was not supplied
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 *
 * Enumeration sets must be considered dynamic.  For some attributes, the set
 * may change at any time.
 */
tPvErr PVDECL PvAttrRangeEnum(tPvHandle Camera,
                              const char* Name,
                              char* pBuffer,
                              unsigned long BufferSize,
                              unsigned long* pSize);


/*
 * Function:  PvAttrRangeUint32()
 *
 * Purpose:   Get the value range for a uint32 attribute.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,          Handle to the camera 
 * [ IN] const char* Name,          Attribute name 
 * [OUT] tPvUint32* pMin,           Minimum value returned here
 * [OUT] tPvUint32* pMax,           Maximum value returned here
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvErrNotFound,        the requested attribute doesn't exist
 *               ePvErrWrongType,       the requested attribute is not of the correct type
 *               ePvBadParameter,       a valid pointer for pMin or pMax was not supplied
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 */
tPvErr PVDECL PvAttrRangeUint32(tPvHandle Camera,
                                const char* Name,
                                tPvUint32* pMin,
                                tPvUint32* pMax);


/*
 * Function:  PvAttrRangeFloat32()
 *
 * Purpose:   Get the value range for a float32 attribute.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,          Handle to the camera 
 * [ IN] const char* Name,          Attribute name 
 * [OUT] tPvFloat32* pMin,          Minimum value returned here           
 * [OUT] tPvFloat32* pMax,          Maximum value returned here
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvErrNotFound,        the requested attribute doesn't exist
 *               ePvErrWrongType,       the requested attribute is not of the correct type
 *               ePvBadParameter,       a valid pointer for pMin or pMax was not supplied
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 */
tPvErr PVDECL PvAttrRangeFloat32(tPvHandle Camera,
                                 const char* Name,
                                 tPvFloat32* pMin,
                                 tPvFloat32* pMax);


/*
 * Function:  PvCommandRun()
 *
 * Purpose:   Run a specific command on the camera
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,          Handle to the camera 
 * [ IN] const char* Name,          Attribute name 
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvErrNotFound,        the requested attribute doesn't exist
 *               ePvErrWrongType,       the requested attribute is not of the correct type
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 */
tPvErr PVDECL PvCommandRun(tPvHandle Camera, const char* Name);


/*
 * Function:  PvAttrStringGet()
 *
 * Purpose:   Get the value of a string attribute.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,          Handle to the camera 
 * [ IN] const char* Name,          Attribute name 
 * [OUT] char* pBuffer,             Caller-allocated buffer; the string is
 *                                    copied here
 * [ IN] unsigned long BufferSize,  Size of buffer, in bytes
 * [OUT] unsigned long* pSize,      Size of the string is returned here.  This
 *                                    may be bigger than BufferSize!  (pSize
 *                                    may be NULL.)
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvErrNotFound,        the requested attribute doesn't exist
 *               ePvErrWrongType,       the requested attribute is not of the correct type
 *               ePvBadParameter,       a valid pointer for pBuffer was not supplied
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 */
tPvErr PVDECL PvAttrStringGet(tPvHandle Camera,
                              const char* Name,
                              char* pBuffer,
                              unsigned long BufferSize,
                              unsigned long* pSize);


/*
 * Function:  PvAttrStringSet()
 *
 * Purpose:   Set the value of a string attribute.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,          Handle to the camera 
 * [ IN] const char* Name,          Attribute name 
 * [ IN] const char* Value,  Value to set
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvErrNotFound,        the requested attribute doesn't exist
 *               ePvErrWrongType,       the requested attribute is not of the correct type
 *               ePvErrForbidden,       the requested attribute forbid this operation
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 */
tPvErr PVDECL PvAttrStringSet(tPvHandle Camera,
                              const char* Name,
                              const char* Value);


/*
 * Function:  PvAttrEnumGet()
 *
 * Purpose:   Get the value of an enumerated attribute.  The enumeration value
 *            is a string.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,          Handle to the camera 
 * [ IN] const char* Name,          Attribute name 
 * [OUT] char* pBuffer,             Caller-allocated buffer; the string is
 *                                    copied here
 * [ IN] unsigned long BufferSize,  Size of buffer, in bytes
 * [OUT] unsigned long* pSize,      Size of the string is returned here.  This
 *                                    may be bigger than BufferSize!  (pSize
 *                                    may be NULL.)
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvErrNotFound,        the requested attribute doesn't exist
 *               ePvErrWrongType,       the requested attribute is not of the correct type
 *               ePvBadParameter,       a valid pointer for pBuffer was not supplied
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 */
tPvErr PVDECL PvAttrEnumGet(tPvHandle Camera,
                            const char* Name,
                            char* pBuffer,
                            unsigned long BufferSize,
                            unsigned long* pSize);


/*
 * Function:  PvAttrEnumSet()
 *
 * Purpose:   Set the value of an enumerated attribute.  The enumeration value
 *            is a string.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,          Handle to the camera 
 * [ IN] const char* Name,          Attribute name 
 * [ IN] const char* Value,         Value to set
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvErrNotFound,        the requested attribute doesn't exist
 *               ePvErrWrongType,       the requested attribute is not of the correct type
 *               ePvErrForbidden,       the requested attribute forbid this operation
 *               ePvErrOutOfRange,      the supplied value is out of range
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 */
tPvErr PVDECL PvAttrEnumSet(tPvHandle Camera,
                            const char* Name,
                            const char* Value);


/*
 * Function:  PvAttrUint32Get()
 *
 * Purpose:   Get the value of a uint32 attribute.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,          Handle to the camera 
 * [ IN] const char* Name,          Attribute name 
 * [OUT] tPvUint32* pValue,         Value is returned here
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvErrNotFound,        the requested attribute doesn't exist
 *               ePvErrWrongType,       the requested attribute is not of the correct type
 *               ePvBadParameter,       a valid pointer for pValue was not supplied
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 */
tPvErr PVDECL PvAttrUint32Get(tPvHandle Camera,
                              const char* Name,
                              tPvUint32* pValue);


/*
 * Function:  PvAttrUint32Set()
 *
 * Purpose:   Set the value of a uint32 attribute.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,          Handle to the camera 
 * [ IN] const char* Name,          Attribute name 
 * [ IN] tPvUint32 Value,           Value to set
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvErrNotFound,        the requested attribute doesn't exist
 *               ePvErrWrongType,       the requested attribute is not of the correct type
 *               ePvErrForbidden,       the requested attribute forbid this operation
 *               ePvErrOutOfRange,      the supplied value is out of range
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 */
tPvErr PVDECL PvAttrUint32Set(tPvHandle Camera,
                              const char* Name,
                              tPvUint32 Value);


/*
 * Function:  PvAttrFloat32Get()
 *
 * Purpose:   Get the value of a float32 attribute.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,          Handle to the camera 
 * [ IN] const char* Name,          Attribute name 
 * [OUT] tPvFloat32* pValue,        Value is returned here
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvErrNotFound,        the requested attribute doesn't exist
 *               ePvErrWrongType,       the requested attribute is not of the correct type
 *               ePvBadParameter,       a valid pointer for pValue was not supplied
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 */
tPvErr PVDECL PvAttrFloat32Get(tPvHandle Camera,
                               const char* Name,
                               tPvFloat32* pValue);


/*
 * Function:  PvAttrFloat32Set()
 *
 * Purpose:   Set the value of a float32 attribute.
 *
 * Arguments:
 *
 * [ IN] tPvHandle Camera,          Handle to the camera 
 * [ IN] const char* Name,          Attribute name 
 * [ IN] tPvFloat32 Value,          Value to set
 *
 * Return:    ePvErrSuccess if no error, otherwise likely to be any of the
 *            following error codes:
 *              
 *               ePvErrBadHandle,       the handle of the camera is invalid
 *               ePvErrUnplugged,       the camera has been unplugged 
 *               ePvErrNotFound,        the requested attribute doesn't exist
 *               ePvErrWrongType,       the requested attribute is not of the correct type
 *               ePvErrForbidden,       the requested attribute forbid this operation
 *               ePvErrOutOfRange,      the supplied value is out of range
 *               ePvErrInternalFault,   an internal fault occurred
 *               ePvErrBadSequence,     API isn't initialized
 */
tPvErr PVDECL PvAttrFloat32Set(tPvHandle Camera,
                               const char* Name,
                               tPvFloat32 Value);

//----- Utility ---------------------------------------------------------------

/*
 * Function:  PvUtilityColorInterpolate()
 *
 * Purpose:   Perform color interpolation.  Input format is bayer8 or bayer16
 *            (raw bayer image).  Output format is RGB or separate color
 *            planes.
 *
 * Arguments:
 *
 * [ IN] const tPvFrame* pFrame,        Raw bayer image
 * [OUT] void* BufferRed,               Red plane for output image
 * [OUT] void* BufferGreen,             Green plane for output image
 * [OUT] void* BufferBlue,              Blue plane for output image
 * [ IN] unsigned long PixelPadding,    Padding after each pixel in raw-pixel units
 * [ IN] unsigned long LinePadding,     Padding after each line in raw-pixel units
 *
 * Return:    none
 *
 * Caller must allocate the output buffers:
 *
 *      num_pixels = (((1 + pixel_padding) * width) + line_padding) * height
 *
 *      buffer_size = raw_pixel_size * num_pixels
 *
 * Perhaps the most common way to use this function will be to generate a
 * Win32::StretchDiBits compatible BGR buffer:
 *
 *      #define ULONG_PADDING(x)                        (((x+3) & ~3) - x)
 *
 *      unsigned long line_padding = ULONG_PADDING(width*3);
 *      unsigned long buffer_size = ((width*3) + line_padding) * height;
 *
 *      unsigned char* buffer = new unsigned char[buffer_size];
 *
 *      PvUtilityColorInterpolate(&frame, &buffer[2], &buffer[1],
 *                                &buffer[0], 2, line_padding);
 *
 */
void PVDECL PvUtilityColorInterpolate(const tPvFrame* pFrame,
                                      void* BufferRed,
                                      void* BufferGreen,
                                      void* BufferBlue,
                                      unsigned long PixelPadding,
                                      unsigned long LinePadding);


#ifdef __cplusplus
}
#endif

#endif // PVAPI_H_INCLUDE

