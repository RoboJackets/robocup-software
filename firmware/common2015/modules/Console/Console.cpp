#include "Console.hpp"

#include "logger.hpp"

const std::string Console::RX_BUFFER_FULL_MSG = "RX BUFFER FULL";
const std::string Console::COMMAND_BREAK_MSG = "*BREAK*\033[K";

shared_ptr<Console> Console::instance;

Console::Console() : pc(USBTX, USBRX) {
    // Set default values for the header parameters
    CONSOLE_USER = "anon";
    CONSOLE_HOSTNAME = "robot";
    setHeader();

    // Use a higher baudrate than the default for a faster console
    Baudrate(57600);

    // clear buffers
    ClearRXBuffer();

    // attach interrupt handlers
    pc.attach(this, &Console::RXCallback, Serial::RxIrq);

    // reset indices
    rxIndex = 0;
}

Console::~Console() {}

shared_ptr<Console>& Console::Instance() {
    if (!instance) instance.reset(new Console);

    return instance;
}

void Console::PrintHeader() {
    // prints out a bash-like header
    Flush();
    pc.printf("\r\n%s", CONSOLE_HEADER.c_str());
    Flush();
}

void Console::ClearRXBuffer() { memset(rxBuffer, '\0', BUFFER_LENGTH); }

void Console::Flush() { fflush(stdout); }  // TODO: pc.fflush()?

void Console::RXCallback() {
    // If for some reason more than one character is in the buffer when the
    // interrupt is called, handle them all.
    while (pc.readable()) {
        // If there is a command that hasn't finished yet, ignore the character
        // for now
        if (command_ready == true) {
            return;

        } else {
            // Otherwise, continue as normal
            // read the char that caused the interrupt
            char c = pc.getc();

            if (esc_flag_one == true && esc_flag_two == true) {
                esc_en = true;
            } else {
                esc_en = false;
            }

            // if the buffer is full, ignore the chracter and print a
            // warning to the console
            if (rxIndex >= (BUFFER_LENGTH - 5) && c != BACKSPACE_FLAG_CHAR) {
                rxIndex = 0;
                pc.printf("%s\r\n", RX_BUFFER_FULL_MSG.c_str());
                Flush();

                // Execute the function that sets up the console after a
                // command's execution.
                // This will ensure everything flushes corectly on a full buffer
                CommandHandled();
            }

            // if a new line character is sent, process the current buffer
            else if (c == NEW_LINE_CHAR) {
                // print new line prior to executing
                pc.printf("%c\n", NEW_LINE_CHAR);
                Flush();
                rxBuffer[rxIndex] = '\0';

                if (history.size() >= MAX_HISTORY) history.pop_front();
                if (rxIndex != 0) history.push_back(rxBuffer);

                history_index = 0;
                command_ready = true;
            }

            // if a backspace is requested, handle it.
            else if (c == BACKSPACE_FLAG_CHAR)
                if (rxIndex > 0) {  // instance->CONSOLE_HEADER.length()) {
                    // re-terminate the string
                    rxBuffer[--rxIndex] = '\0';

                    // 1) Move cursor back
                    // 2) Write a space to clear the character
                    // 3) Move back cursor again
                    pc.putc(BACKSPACE_REPLY_CHAR);
                    pc.putc(BACKSPACE_REPLACE_CHAR);
                    pc.putc(BACKSPACE_REPLY_CHAR);
                    Flush();
                } else {
                    /* do nothing if we can't back space any more */
                }

            // set that a command break was requested flag if we received a
            // break character
            else if (c == BREAK_CHAR) {
                iter_break_req = true;
            }

            else if (c == ESCAPE_SEQ_ONE) {
                esc_flag_one = true;
            }

            else if (c == ESCAPE_SEQ_TWO) {
                if (esc_flag_one == true) {
                    esc_flag_two = true;
                } else {
                    esc_flag_two = false;
                }
            }

            else if (c == ARROW_UP_KEY || c == ARROW_DOWN_KEY) {
                if (esc_en == false) {
                    rxBuffer[rxIndex++] = c;
                    pc.putc(c);
                    Flush();
                } else {
                    if (history_index < 0) history_index = 0;
                    if ((size_t)history_index >= history.size())
                        history_index =
                            history.size() - (history.empty() ? 0 : 1);

                    if (history.size() > 0 &&
                        !(rxIndex == 0 && c == ARROW_DOWN_KEY)) {
                        std::string cmd =
                            history.at(history.size() - 1 - history_index);
                        pc.printf("\r%s%s", CONSOLE_HEADER.c_str(),
                                  cmd.c_str());
                        rxIndex = cmd.size();
                        memcpy(rxBuffer, cmd.c_str(), rxIndex + 1);
                    }

                    switch (c) {
                        case ARROW_UP_KEY:
                            history_index++;
                            break;
                        case ARROW_DOWN_KEY:
                            history_index--;
                            break;
                        default:
                            break;
                    }
                }
                esc_flag_one = false;
                esc_flag_two = false;
            }

            else if (c == ARROW_LEFT_KEY || c == ARROW_RIGHT_KEY) {
                if (esc_en == false) {
                    rxBuffer[rxIndex++] = c;
                } else {
                    pc.putc(ESCAPE_SEQ_ONE);
                    pc.putc(ESCAPE_SEQ_TWO);
                }
                pc.putc(c);
                Flush();
                esc_flag_one = false;
                esc_flag_two = false;
            }

            // No special character, add it to the buffer and return it to
            // the terminal to be visible.
            else {
                rxBuffer[rxIndex++] = c;
                pc.putc(c);
                Flush();
                esc_flag_one = false;
                esc_flag_two = false;
            }
        }
    }
}

void Console::RequestSystemStop() { sysStopReq = true; }

bool Console::IsSystemStopRequested() const { return sysStopReq; }

bool Console::IterCmdBreakReq() const { return iter_break_req; }

void Console::IterCmdBreakReq(bool newState) {
    iter_break_req = newState;

    // Print out the header if an iterating command is stopped
    if (newState == false) {
        pc.printf("%s", COMMAND_BREAK_MSG.c_str());
        PrintHeader();
    }
}

char* Console::rxBufferPtr() { return rxBuffer; }

bool Console::CommandReady() const { return command_ready; }

void Console::CommandHandled() {
    // Clean up after command execution
    rxIndex = 0;

    // reset our outgoing flag saying if there's a valid command sequence in the
    // RX buffer or now
    command_ready = false;

    // print out the header without a newline first
    if (iter_break_req == false) {
        pc.printf("%s", CONSOLE_HEADER.c_str());
        Flush();
    }
}

void Console::changeHostname(const std::string& hostname) {
    CONSOLE_HOSTNAME = hostname;
    setHeader();
}

void Console::changeUser(const std::string& user) {
    CONSOLE_USER = user;
    setHeader();
}

void Console::setHeader() {
    CONSOLE_HEADER = "\033[1;36m" + CONSOLE_USER + "\033[1;32m@\033[1;33m" +
                     CONSOLE_HOSTNAME + " \033[36m$\033[m \033[J\033[m";
}

void Console::Baudrate(uint16_t baud) {
    _baudRate = baud;
    pc.baud(_baudRate);
}

uint16_t Console::Baudrate() const { return _baudRate; }

void Console::SetEscEnd(char c) { esc_host_end_char = c; }

std::string Console::GetHostResponse() {
    if (esc_host_res_rdy == true) {
        esc_host_res_rdy = false;

        return esc_host_res;
    } else {
        return "";
    }
}

void Console::ShowLogo() {
    Flush();

    pc.printf(
        "\033[01;33m"
        "   _____       _                _            _        _\r\n"
        "  |  __ \\     | |              | |          | |      | |      \r\n"
        "  | |__) |___ | |__   ___      | | __ _  ___| | _____| |_ ___ \r\n"
        "  |  _  // _ \\| '_ \\ / _ \\ _   | |/ _` |/ __| |/ / _ \\ __/ __|\r\n"
        "  | | \\ \\ (_) | |_) | (_) | |__| | (_| | (__|   <  __/ |_\\__ \\\r\n"
        "  |_|  \\_\\___/|_.__/ \\___/ \\____/ "
        "\\__,_|\\___|_|\\_\\___|\\__|___/\r\n\033[0m");

    Flush();
}

void Console::SetTitle(const std::string& title) {
    pc.printf("\033]0;%s\007", title.c_str());
    Flush();
}
