#pragma once

#include "mbed.h"

#define RTP_MAX_DATA_SIZE 122

typedef struct _RTP_t {
    uint8_t total_size;
    union { // [128 Bytes]
        uint8_t raw[RTP_MAX_DATA_SIZE + 6]; // [128 Bytes] 1.1
        struct {    // [128 Bytes] 1.2
            //union { // [4 Bytes]
            //uint8_t full_header[4]; // [4 Bytes] 2.1
            //struct {    // 2.2
            union { // [4 Bytes]
                uint8_t header[3];  // 3.1
                struct {    // 3.2
                    //uint8_t field_size; // [1 Byte]
                    uint8_t payload_size;
                    uint8_t address;
                    uint8_t sfs : 1, ack : 1, subclass : 2, port : 4;
                };
            };

            //};
            //};
            uint8_t payload[RTP_MAX_DATA_SIZE]; // [122 Bytes]
            uint8_t rssi;   // [1 Byte]
            uint8_t lqi;    // [1 Byte]
        };
    };
} RTP_t;
