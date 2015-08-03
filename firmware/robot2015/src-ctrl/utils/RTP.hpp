#pragma once

#include <cstdint>

#define RTP_HEADER(port, subclass, ack, sfs)    (((sfs & 0x01) << 7) | ((ack & 0x01) << 6) | ((subclass & 0x03) << 4) | (port & 0x0F))

#define RTP_MAX_DATA_SIZE 122

enum RTPPort {
    RTP_PORT_CONSOLE     = 0x00,
    RTP_PORT_PARAM       = 0x02,
    RTP_PORT_COMMANDER   = 0x03,
    RTP_PORT_MEM         = 0x04,
    RTP_PORT_LOG         = 0x05,
    RTP_PORT_PLATFORM    = 0x0D,
    RTP_PORT_LINK        = 0x0F
};

struct RTP_t {
    uint8_t total_size;
    union { // [128 Bytes]
        struct {
            uint8_t raw[RTP_MAX_DATA_SIZE + 6]; // [128 Bytes] 1.1
        };
        struct {    // [128 Bytes] 1.2
            //union { // [4 Bytes]
            //uint8_t full_header[4]; // [4 Bytes] 2.1
            //struct {    // 2.2
            union { // [4 Bytes]
                struct {
                    uint8_t header[3];  // 3.1
                };
                struct {    // 3.2
                    //uint8_t field_size; // [1 Byte]
                    uint8_t payload_size;
                    uint8_t address;
                    union {
                        struct {
                            uint8_t header_link;
                        };
                        struct {
                            uint8_t sfs : 1, ack : 1, subclass : 2, port : 4;
                        };
                    };

                };
            };
            //};
            //};
            uint8_t payload[RTP_MAX_DATA_SIZE]; // [122 Bytes]
            uint8_t rssi;   // [1 Byte]
            uint8_t lqi;    // [1 Byte]
        };
    };
};
