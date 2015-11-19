#include "Console.hpp"

#include "logger.hpp"

#define PUTC(c) pc.putc(c)
#define GETC pc.getc
#define PRINTF(...) pc.printf(__VA_ARGS__)

const std::string Console::RX_BUFFER_FULL_MSG = "RX BUFFER FULL";
const std::string Console::COMMAND_BREAK_MSG = "*BREAK*\033[K";

shared_ptr<Console> Console::instance;

bool Console::iter_break_req = false;
bool Console::command_handled = true;
bool Console::command_ready = false;

Console::Console() : pc(USBTX, USBRX) {}

shared_ptr<Console>& Console::Instance() {
    if (instance.get() == nullptr) instance.reset(new Console);

    return instance;
}

void Console::Init() {
    auto instance = Instance();

    // Set default values for the header parameters
    instance->CONSOLE_USER = "anon";
    instance->CONSOLE_HOSTNAME = "robot";
    instance->setHeader();

    // set baud rate, store the value before
    Baudrate(9600);

    // clear buffers
    instance->ClearRXBuffer();
    instance->ClearTXBuffer();

    // attach interrupt handlers
    // instance->pc.attach(&Console::RXCallback_MODSERIAL, MODSERIAL::RxIrq);
    // instance->pc.attach(&Console::TXCallback_MODSERIAL, MODSERIAL::TxIrq);
    instance->pc.attach(instance.get(), &Console::RXCallback, Serial::RxIrq);
    instance->pc.attach(instance.get(), &Console::TXCallback, Serial::TxIrq);

    // reset indicces
    instance->rxIndex = 0;
    instance->txIndex = 0;

    // Default for a host response escaped input terminating character
    // instance->SetEscEnd('R');

    LOG(INF3, "Hello from the 'common2015' library!");
}

void Console::PrintHeader() {
    // prints out a bash-like header
    Flush();
    instance->PRINTF("\r\n%s", instance->CONSOLE_HEADER.c_str());
    Flush();
}

void Console::ClearRXBuffer() { memset(rxBuffer, '\0', BUFFER_LENGTH); }

void Console::ClearTXBuffer() { memset(txBuffer, '\0', BUFFER_LENGTH); }

void Console::Flush() { fflush(stdout); }

void Console::RXCallback() {
    // If for some reason more than one character is in the buffer when the
    // interrupt is called, handle them all.
    while (pc.readable()) {
        // If there is an command that hasn't finished yet, ignore the character
        // for now
        if (command_handled == false && command_ready == true) {
            return;

        } else {
            // Otherwise, continue as normal

            // read the char that caused the interrupt
            char c = GETC();

            // flag the start of an arrow key sequence
            // if (c == ARROW_KEY_SEQUENCE_ONE) {
            // 	flagOne = true;
            // }
            // check if we're in a sequence if flagOne is set - do things if
            // necessary
            // else if (flagOne) {
            // 	if (flagTwo) {
            // 		switch (c) {
            // 		case ARROW_UP_KEY:
            // 			PRINTF("\033M");
            // 			break;

            // 		case ARROW_DOWN_KEY:
            // 			PRINTF("\033D");
            // 			break;

            // 		default:
            // 			flagOne = false;
            // 			flagTwo = false;
            // 		}

            // 		Flush();
            // 		continue;

            // 	} else {	// flagTwo not set
            // 		switch (c) {
            // 		case ARROW_KEY_SEQUENCE_TWO:
            // 			flagTwo = true;
            // 			break;

            // 		default:
            // 			flagOne = false;
            // 			break;
            // 		}
            // 	}
            // }

            // if the buffer is full, ignore the chracter and print a
            // warning to the console
            if (rxIndex >= (BUFFER_LENGTH - 5) && c != BACKSPACE_FLAG_CHAR) {
                rxIndex = 0;
                PRINTF("%s\r\n", RX_BUFFER_FULL_MSG.c_str());
                Flush();

                // Execute the function that sets up the console after a
                // command's execution.
                // This will ensure everything flushes corectly on a full buffer
                CommandHandled(true);
            }

            // if a new line character is sent, process the current buffer
            else if (c == NEW_LINE_CHAR) {
                // print new line prior to executing
                PRINTF("%c\n", NEW_LINE_CHAR);
                Flush();
                rxBuffer[rxIndex] = '\0';

                command_ready = true;
                command_handled = false;
            }

            // if a backspace is requested, handle it.
            else if (c == BACKSPACE_FLAG_CHAR)
                if (rxIndex > 0) {  // instance->CONSOLE_HEADER.length()) {
                    // re-terminate the string
                    rxBuffer[--rxIndex] = '\0';

                    // 1) Move cursor back
                    // 2) Write a space to clear the character
                    // 3) Move back cursor again
                    PUTC(BACKSPACE_REPLY_CHAR);
                    PUTC(BACKSPACE_REPLACE_CHAR);
                    PUTC(BACKSPACE_REPLY_CHAR);
                    Flush();
                } else {
                    /* do nothing if we can't back space any more */
                }

            // set that a command break was requested flag if we received a
            // break character
            else if (c == BREAK_CHAR) {
                iter_break_req = true;
            }

            // No special character, add it to the buffer and return it to
            // the terminal to be visible.
            else {
                if ((c != ESC_START) && (esc_en == false)) {
                    if (!(c == ARROW_UP_KEY || c == ARROW_DOWN_KEY)) {
                        rxBuffer[rxIndex++] = c;
                    }

                    esc_host_res = "";
                    PUTC(c);
                    Flush();

                }

                else {
                    if (esc_seq_en == true) {
                        esc_host_res += c;
                    }

                    if (c == ESC_START) {
                        esc_en = true;
                    } else if (c == ESC_SEQ_START) {
                        esc_seq_en = true;
                    } else if (c == esc_host_end_char) {
                        // Remove the terminating character
                        esc_host_res.pop_back();
                        // Set the flag for knowing we've received an expected
                        // response
                        esc_host_res_rdy = true;
                        // Reset everything back to normal
                        esc_en = false;
                        esc_seq_en = false;
                    }
                }
            }
        }
    }
}

void Console::TXCallback() {
    // NVIC_DisableIRQ(UART0_IRQn);
    // handle transmission interrupts if necessary here
    // NVIC_EnableIRQ(UART0_IRQn);
}

void Console::ConComCheck() {
    /*
     * Currently no way to check if a vbus has been disconnected or
     * reconnected. Will continue to keep track of a beta library located
     * here: http://developer.mbed.org/handbook/USBHostSerial
     * Source here:
     * http://developer.mbed.org/users/mbed_official/code/USBHost/file/607951c26872/USBHostSerial/USBHostSerial.cpp
     * It's not present in our currently library, and is not working
     * for most people. It could however, greatly reduce what we have to
     * implement while adding more functionality.
     */

    /*
     * Note for the above ^. The vbus can be monitored through ADC0, input 4.
     * This is possible when bits 29..28 of the PINSEL3 register are set to
     * 0b01.
     *
     * 		- Jon
     */
    return;
}

void Console::RequestSystemStop() {
    Instance()->sysStopReq = true;
    instance.reset();
}

bool Console::IsSystemStopRequested() { return Instance()->sysStopReq; }

bool Console::IterCmdBreakReq() { return iter_break_req; }

void Console::IterCmdBreakReq(bool newState) {
    iter_break_req = newState;

    // Print out the header if an iterating command is stopped
    if (newState == false) {
        instance->PRINTF("%s", COMMAND_BREAK_MSG.c_str());
        PrintHeader();
    }
}

char* Console::rxBufferPtr() { return instance->rxBuffer; }

bool Console::CommandReady() { return command_ready; }

void Console::CommandHandled(bool cmdDoneState) {
    // update the class's flag for if a command was handled or not
    command_handled = cmdDoneState;

    // Clean up after command execution
    instance->rxIndex = 0;

    // reset our outgoing flag saying if there's a valid command sequence in the
    // RX buffer or now
    command_ready = false;

    // print out the header without a newline first
    if (iter_break_req == false) {
        instance->PRINTF("%s", instance->CONSOLE_HEADER.c_str());
        Flush();
    }
}

void Console::changeHostname(const std::string& hostname) {
    instance->CONSOLE_HOSTNAME = hostname;
    instance->setHeader();
}

void Console::changeUser(const std::string& user) {
    instance->CONSOLE_USER = user;
    instance->setHeader();
}

void Console::setHeader() {
    instance->CONSOLE_HEADER =
        "\033[36m" + instance->CONSOLE_USER + "\033[34m@\033[33m" +
        instance->CONSOLE_HOSTNAME + " \033[36m$\033[0m \033[0J\033[0m";
    // instance->CONSOLE_HEADER = instance->CONSOLE_USER + "@" +
    // instance->CONSOLE_HOSTNAME + " $ ";
}

void Console::Baudrate(uint16_t baud) {
    instance->baudrate = baud;
    instance->pc.baud(instance->baudrate);
}

uint16_t Console::Baudrate() { return instance->baudrate; }

void Console::RXCallback_MODSERIAL(MODSERIAL_IRQ_INFO* info) {
    Console::RXCallback();
}

void Console::TXCallback_MODSERIAL(MODSERIAL_IRQ_INFO* info) {
    Console::TXCallback();
}

void Console::SetEscEnd(char c) { instance->esc_host_end_char = c; }

std::string Console::GetHostResponse() {
    if (instance->esc_host_res_rdy == true) {
        instance->esc_host_res_rdy = false;

        return instance->esc_host_res;
    } else {
        return "";
    }
}

void Console::ShowLogo() {
    Flush();

    instance->PRINTF(
        "   _____       _                _            _        _\r\n"
        "  |  __ \\     | |              | |          | |      | |      \r\n"
        "  | |__) |___ | |__   ___      | | __ _  ___| | _____| |_ ___ \r\n"
        "  |  _  // _ \\| '_ \\ / _ \\ _   | |/ _` |/ __| |/ / _ \\ __/ __|\r\n"
        "  | | \\ \\ (_) | |_) | (_) | |__| | (_| | (__|   <  __/ |_\\__ \\\r\n"
        "  |_|  \\_\\___/|_.__/ \\___/ \\____/ "
        "\\__,_|\\___|_|\\_\\___|\\__|___/\r\n");

    Flush();
}
