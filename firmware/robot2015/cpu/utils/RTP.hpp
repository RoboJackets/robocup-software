#ifndef REAL_TIME_PACKET_H
#define REAL_TIME_PACKET_H

#include "mbed.h"

#define RTP_MAX_DATA_SIZE 32

typedef struct _RTP_t {
    uint8_t data_size;
    union {
        struct {
            union {
                uint8_t header;
                struct {
                    uint8_t port : 4;
                    uint8_t subclass : 4;
                };
            };
            uint8_t data[RTP_MAX_DATA_SIZE];
        };
        uint8_t raw[RTP_MAX_DATA_SIZE + 1];
    };
} RTP_t;

#endif  // REAL_TIME_PACKET_H
