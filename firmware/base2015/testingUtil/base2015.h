/**
 * add tunable parameters here
 */
typedef enum TuneableParameter {

} TuneableParameter;

//////////////////////////////////////
//  forward packets (soccer->base)  //
//////////////////////////////////////


#define HID_FWD_GLOB_PKT           0x11
#define HID_FWD_GLOB_TX            0x12

#define HID_FWD_BOT_TUNE_PKT       0x20
#define HID_FWD_BOT_TUNE_TX        0x21

#define HID_FWD_FW_PKT             0x30
#define HID_FWD_FW_REQ_CHECKSUM    0x31
#define HID_FWD_FW_MASK_APPLY      0x32
#define HID_FWD_FW_START_FLASH     0x33
#define HID_FWD_FW_INTERRUPT_FLASH 0x34


#define HID_FWD_BASE_PROTO_VER     0xF0
#define HID_FWD_BASE_MODE          0xF1
#define HID_FWD_BASE_MODE_SET      0xF2
#define HID_FWD_BASE_MODE_REQ      0xF3
#define HID_FWD_BASE_REQ_STATUS    0xF4
#define HID_FWD_BASE_REBOOT        0xFA
#define HID_FWD_E_STOP             0xFF

//////////////////////////////////////
//  reverse packets (base->soccer)  //
//////////////////////////////////////
#define HID_REV_BASE_MODE 0x00;
#define HID_REV_BASE_STATUS 0x00;
#define HID_REV_BOT_STATUS 0x01;
