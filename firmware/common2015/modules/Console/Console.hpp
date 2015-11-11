#pragma once

#include <mbed.h>

#include <memory>

// #include "MODDMA.h"
#include "MODSERIAL.h"


/**
 * enable scrolling vi sequence
 */
const std::string ENABLE_SCROLL_SEQ = "\033[r";

/**
 * clear screen vi sequence
 */
const std::string CLEAR_SCREEN_SEQ = "\033[2J";

const std::string ANSI_SD = "\033[1T";

const std::string ANSI_SU = "\033[1S";

/**
 * Manages serial-over-USB communication with the PC
 */
class Console
{

public:
  /**
   * max buffer length. Default 400 (five lines)
   */
  static const uint16_t BUFFER_LENGTH = 400;

  /**
   * new line character. Default '\r'
   */
  static const char NEW_LINE_CHAR = '\r';  //ASCII CR (\r) (0x0D)

  /**
   * backspace flag char. (What char does the console send when the backspace key
   * is pressed?). Default DEL
   */
  static const char BACKSPACE_FLAG_CHAR = 0x7F; //ASCII DEL (0x7F)

  /**
   * backspace reply char. (What char causes screen to delete the last
   * character?). Default '\b'
   */
  static const char BACKSPACE_REPLY_CHAR = '\b';   //ASCII BK (\b) (0x08)

  /**
   * when the console backspaces, what does the last character become. Default ' '
   */
  static const char BACKSPACE_REPLACE_CHAR = ' ';

  /**
   * default ETX (0x3)
   */
  static const char BREAK_CHAR = 3;

  static const char ESC_START = 0x1B;
  static const char ESC_SEQ_START = '[';

  /**
   * define the sequence for arrow key flags
   */
  static const char ARROW_KEY_SEQUENCE_ONE = 27;
  static const char ARROW_KEY_SEQUENCE_TWO = 91;
  static const char ARROW_UP_KEY = 65;
  static const char ARROW_DOWN_KEY = 66;

  static const char CMD_END_CHAR = ';';

  /**
   * receice buffer full error message
   */
  static const std::string RX_BUFFER_FULL_MSG;

  /**
   * break message
   */
  static const std::string COMMAND_BREAK_MSG;

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
  static void RequestSystemStop();

  /**
   * returns if the main loop should break
   */
  static bool IsSystemStopRequested();

  static bool IterCmdBreakReq();
  static void IterCmdBreakReq(bool newState);

  static char* rxBufferPtr();

  static bool CommandReady();
  static void CommandHandled(bool);

  static void changeHostname(const std::string&);
  static void changeUser(const std::string&);

  static void Baudrate(uint16_t);
  static uint16_t Baudrate();

  static void PrintHeader();
  static void ShowLogo();
  static void SetEscEnd(char c);
  static const std::string& GetHostResponse();

private:
  // Constructor is only used in init branch of Instance()
  Console();

  static std::shared_ptr<Console>& Instance();

  void ClearRXBuffer();

  void ClearTXBuffer();

  void RXCallback();

  void TXCallback();

  void setHeader();

  void RXCallback_MODSERIAL(MODSERIAL_IRQ_INFO* info);
  void TXCallback_MODSERIAL(MODSERIAL_IRQ_INFO* info);

  static std::shared_ptr<Console> instance;

  // Flags for command execution states
  static bool   iter_break_req;
  static bool   command_handled;
  static bool   command_ready;

  /**
  * Console header string.
  */
  std::string CONSOLE_HEADER;
  std::string CONSOLE_USER;
  std::string CONSOLE_HOSTNAME;

  /**
  * Serial (over USB) baud rate. Default 9600. Screen default 9600
  */
  uint16_t baudrate;

  /**
  * Serial connection
  */
  Serial pc;
  // MODSERIAL pc;

  /**
   * Receive buffer
   */
  char rxBuffer[BUFFER_LENGTH];

  /**
   * Transmission buffer
   */
  char txBuffer[BUFFER_LENGTH];

  /**
   * Is a system stop requested
   */
  bool sysStopReq = false;

  /**
   * flags for arrow key sequences. Arroy keys aren't in ASCII so we have to
   * process the the three key sequence
   */
  bool flagOne = false;
  bool flagTwo = false;

  bool esc_en = false;
  bool esc_seq_en = false;
  bool esc_host_res_rdy = false;

  std::string esc_host_res;

  char esc_host_end_char = 'R';


  /**
   * receive buffer index
   */
  uint16_t rxIndex = 0;

  /**
   * transmission buffer index
   */
  uint16_t txIndex = 0;
};
