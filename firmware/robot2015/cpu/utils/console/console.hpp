#pragma once


#include "mbed.h"
#include "commands.hpp"

#include <stdio.h>
#include <string>
#include <memory>


/**
 * enable scrolling vi sequence
 */
const std::string ENABLE_SCROLL_SEQ = "\033[r";

/**
 * clear screen vi sequence
 */
const std::string CLEAR_SCREEN_SEQ = "\033[2J";

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


/**
 * Manages serial-over-USB communication with the PC
 */
class Console
{

 public:
  /**
   * max buffer length. Default 252 (three lines)
   */
  static constexpr uint8_t BUFFER_LENGTH = 252;

  /**
   * new line character. Default '\r'
   */
  static constexpr char NEW_LINE_CHAR = 13;  //ASCII CR (\r) (0x0D)

  /**
   * backspace flag char. (What char does the console send when the backspace key
   * is pressed?). Default DEL
   */
  static constexpr char BACKSPACE_FLAG_CHAR = 127; //ASCII DEL (0x7F)

  /**
   * backspace reply char. (What char causes screen to delete the last
   * character?). Default '\b'
   */
  static constexpr char BACKSPACE_REPLY_CHAR = 8;   //ASCII BK (\b) (0x08)

  /**
   * when the console backspaces, what does the last character become. Default ' '
   */
  static constexpr char BACKSPACE_REPLACE_CHAR = 32;

  /**
   * default ETX (0x3)
   */
  static constexpr char BREAK_CHAR = 3;

  /**
   * define the sequence for arrow key flags
   */
  static constexpr char ARROW_KEY_SEQUENCE_ONE = 27;
  static constexpr char ARROW_KEY_SEQUENCE_TWO = 91;

  /**
   * arrow up value, following arrow key flag
   */
  static constexpr char ARROW_UP_KEY = 65;

  /**
   * arrow down value, following arrow key flag
   */
  static constexpr char ARROW_DOWN_KEY = 66;

  /**
   * console header string.
   */
  static const std::string CONSOLE_HEADER;

  /**
   * receice buffer full error message
   */
  static const std::string RX_BUFFER_FULL_MSG;

  /**
   * break message
   */
  static const std::string COMMAND_BREAK_MSG;

  /**
   * Serial (over USB) baud rate. Default 9600. Screen default 9600
   */
  static constexpr uint16_t BAUD_RATE = 9600;

  /**
   * Console initialization routine. Attaches interrupt handlers and clears the
   * buffers.
   */
  static void Init();

  /**
   * console communications check. should be called in the main loop.
   */
  static void ConComCheck();

  /**
   * flushes stdout. Should be called after every putc or printf block.
   */
  static void Flush();

  /**
   * requests the main loop break
   */
  static void RequestSystemStop(void);

  /**
   * returns if the main loop should break
   */
  static bool IsSystemStopRequested(void);

 private:
  // Constructor is only used in init branch of Instance()
  Console();

  static std::shared_ptr<Console> &Instance();

  void ClearRXBuffer();

  void ClearTXBuffer();

  void RXCallback();

  void TXCallback();

  static std::shared_ptr<Console> instance;

  /**
   * serial connection
   */
  Serial pc;

  /**
   * receive buffer
   */
  char rxBuffer[BUFFER_LENGTH];

  /**
   * transmission buffer
   */
  char txBuffer[BUFFER_LENGTH];

  /**
   * is a system stop requested
   */
  bool sysStopReq = false;

  /**
   * flags for arrow key sequences. Arroy keys aren't in ASCII so we have to
   * process the the three key sequence
   */
  bool flagOne = false;
  bool flagTwo = false;

  /**
   * receive buffer index
   */
  uint8_t rxIndex = 0;

  /**
   * transmission buffer index
   */
  uint8_t txIndex = 0;
};