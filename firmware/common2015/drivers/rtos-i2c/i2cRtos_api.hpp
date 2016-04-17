#pragma once

#include "i2c_api.h"

#if DEVICE_I2C

#ifdef __cplusplus
extern "C" {
#endif

void i2cRtos_init(i2c_t* obj, PinName sda, PinName scl);

int i2cRtos_read(i2c_t* obj, int address, char* data, int length, int stop);
int i2cRtos_write(i2c_t* obj, int address, const char* data, int length,
                  int stop);
int i2cRtos_byte_read(i2c_t* obj, int last);
int i2cRtos_byte_write(i2c_t* obj, int data);
int i2cRtos_stop(i2c_t* obj);

#if DEVICE_I2CSLAVE
int i2cRtos_slave_receive(i2c_t* obj, uint32_t tmOut);
int i2cRtos_slave_read(i2c_t* obj, char* data, int length);
int i2cRtos_slave_write(i2c_t* obj, const char* data, int length);
#endif

#ifdef __cplusplus
}
#endif

#endif
