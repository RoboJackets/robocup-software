#pragma once

#include <deque>
#include <memory>

#include <mbed.h>

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
class Console {
public:
    /// Get a pointer to the global Console instance
    static std::shared_ptr<Console>& Instance();

    /**
     * new line character. Default '\r'
     */
    static const char NEW_LINE_CHAR = '\r';  // ASCII CR (\r) (0x0D)

    /**
     * backspace flag char. (What char does the console send when the backspace
     * key
     * is pressed?). Default DEL
     */
    static const char BACKSPACE_FLAG_CHAR = 0x7F;  // ASCII DEL (0x7F)

    /**
     * backspace reply char. (What char causes screen to delete the last
     * character?). Default '\b'
     */
    static const char BACKSPACE_REPLY_CHAR = '\b';  // ASCII BK (\b) (0x08)

    /**
     * when the console backspaces, what does the last character become. Default
     * ' '
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
    static const char ESCAPE_SEQ_ONE = 27;
    // static const char ESCAPE_SEQ_ONE = '\033';
    static const char ESCAPE_SEQ_TWO = '[';
    static const char ARROW_UP_KEY = 'A';
    static const char ARROW_DOWN_KEY = 'B';
    static const char ARROW_LEFT_KEY = 'C';
    static const char ARROW_RIGHT_KEY = 'D';

    static const char CMD_END_CHAR = ';';

    /**
     * break message
     */
    static const std::string COMMAND_BREAK_MSG;

    /**
     * Deconstructor
     */
    ~Console();

    /**
     * flushes stdout. Should be called after every putc or printf block.
     */
    void Flush();

    /**
     * requests the main loop break
     */
    void RequestSystemStop();

    /// Attach the Serial connection's RX handler to @RXCallback.
    void attachInputHandler();

    /// Detach the Serial connection's RX handler to allow something else
    /// besides the Console to read from stdin.
    void detachInputHandler();

    /**
     * returns if the main loop should break
     */
    bool IsSystemStopRequested() const;

    bool IterCmdBreakReq() const;
    void IterCmdBreakReq(bool newState);

    std::string& rxBuffer();

    bool CommandReady() const;

    /// mark the current command as being handled and cleanup
    void CommandHandled();

    void changeHostname(const std::string&);
    void changeUser(const std::string&);

    void Baudrate(uint16_t);
    uint16_t Baudrate() const;

    void PrintHeader();
    void ShowLogo();
    void SetTitle(const std::string&);
    std::string GetHostResponse();

    /**
    * Serial connection
    */
    Serial pc;

private:
    /**
     * Console initialization routine. Attaches interrupt handlers and clears
     * the buffers.
     */
    Console();

    void RXCallback();

    void setHeader();

    static std::shared_ptr<Console> instance;

    // Flags for command execution states
    bool iter_break_req = false;
    bool command_ready = false;

    /**
    * Console header string.
    */
    std::string CONSOLE_HEADER;
    std::string CONSOLE_USER;
    std::string CONSOLE_HOSTNAME;

    /// baud rate of serial connection
    uint16_t _baudRate;

    /**
     * Receive buffer
     */
    std::string _rxBuffer;

    /**
     * Is a system stop requested
     */
    bool sysStopReq = false;

    /**
     * flags for arrow key sequences. Arroy keys aren't in ASCII so we have to
     * process the the three key sequence
     */
    bool esc_flag_one = false;
    bool esc_flag_two = false;
    bool esc_en = false;
    bool esc_host_res_rdy = false;

    std::string esc_host_res;

    char esc_host_end_char = 'R';

    const size_t MAX_HISTORY = 10;
    int history_index = 0;
    std::deque<std::string> history;
};
