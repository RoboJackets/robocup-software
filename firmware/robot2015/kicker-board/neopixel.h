#pragma once

/**
 * I/O port 
 */
static const uint8_t NEOPIXEL_PORT = 0x18;

/**
 * pin on the I/O port connected to DI on the neo pixel chain
 */
static const uint8_t NEOPIXEL_PIN  = 3;

/**
 * the number of leds on the chain
 */
static const uint8_t LED_COUNT     = 1;

/*
 * functions
 */
void initNeopixelBuffer(void);
void freeNeopixelBuffer(void);
void setBytes(uint8_t* newBytes);
uint8_t* getBytes(void);
void setLed(uint8_t red, uint8_t green, uint8_t blue, uint8_t pos);
void setLeds(uint8_t red, uint8_t green, uint8_t blue);
void writeNeopixels(void);
