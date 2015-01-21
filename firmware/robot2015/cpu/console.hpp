#pragma once

#include <string>

/*
 * uncomment the following line to enable serial transmit and receive debugging
 * on LED's three and four
 *
 * comment the second line to print the command being executed upon enter key
 * hit
 */
//#define DEBUG
#define DISABLE_CMD_FEEDBACK

/**
 * enable scrolling vi sequence
 */
const string ENABLE_SCROLL_SEQ = "\033[r";

/**
 * clear screen vi sequence
 */
const string CLEAR_SCREEN_SEQ = "\033[2J";

/**
 * Console initialization routine. Attaches interrupt handlers and clears the 
 * buffers.
 */
void initConsole(void);

/**
 * Console communications check. Should be called in the main loop. See doc in
 * console.cpp
 */
void conComCheck(void);

/**
 * flushes stdout. Should be called after every putc or printf block.
 */
void flush(void);

/**
 * requests the main loop break
 */
void reqSysStop(void);

/**
 * returns if the main loop is requested to break
 */
bool isSysStopReq(void);

