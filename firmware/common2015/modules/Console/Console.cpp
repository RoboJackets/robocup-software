#include "Console.hpp"

#include "logger.hpp"


#define PUTC(c) 	pc.putc(c)
#define GETC 		pc.getc
#define PRINTF(...)	pc.printf(__VA_ARGS__)


const std::string Console::RX_BUFFER_FULL_MSG = "RX BUFFER FULL";
const std::string Console::COMMAND_BREAK_MSG = "*BREAK*";

shared_ptr<Console> Console::instance;

bool Console::iter_break_req = false;
bool Console::command_handled = true;
bool Console::command_ready = false;


Console::Console() : pc(USBTX, USBRX) {  }

shared_ptr<Console>& Console::Instance()
{
	if (instance.get() == nullptr)
		instance.reset(new Console);

	return instance;
}


char* Console::Init(void)
{
	auto instance = Instance();

	// Set default values for the header parameters
	instance->CONSOLE_USER = "jon";
	instance->CONSOLE_HOSTNAME = "robot";
	instance->CONSOLE_HEADER = instance->CONSOLE_USER + "@" + instance->CONSOLE_HOSTNAME + " $ ";

	// set baud rate, store the value before
	Baudrate(9600);

	// Uncomment this is you don't want to see any logging
	// statements that were called before this is setup
	//Flush();
	//printf(ENABLE_SCROLL_SEQ.c_str());
	//printf(CLEAR_SCREEN_SEQ.c_str());
	//Flush();

	// clear buffers
	instance->ClearRXBuffer();
	instance->ClearTXBuffer();

	// attach interrupt handlers
	instance->pc.attach(instance.get(), &Console::RXCallback, Serial::RxIrq);
	instance->pc.attach(instance.get(), &Console::TXCallback, Serial::TxIrq);

	// reset indicces
	instance->rxIndex = 0;
	instance->txIndex = 0;

	LOG(INIT, "Hello from the 'common2015' library!");

	printf(" ");
	Flush();
	printf("\b");
	Flush();

	return instance->rxBuffer;
}

void Console::PrintHeader(void)
{
	// prints out a bash-like header
	Flush();
	instance->PRINTF("\r\n%s", instance->CONSOLE_HEADER.c_str());
	Flush();
}

void Console::ClearRXBuffer(void)
{
	memset(rxBuffer, '\0', BUFFER_LENGTH);
}

void Console::ClearTXBuffer(void)
{
	memset(txBuffer, '\0', BUFFER_LENGTH);
}

void Console::Flush(void)
{
	fflush(stdout);
}

void Console::RXCallback(void)
{
	// If for some reason more than one character is in the buffer when the
	// interrupt is called, handle them all.
	while (pc.readable()) {

		// If there is an command that hasn't finished yet, ignore the character for now
		if (command_handled == false && command_ready == true) {

			return;

		} else {
			// Otherwise, continue as normal

			// read the char that caused the interrupt
			char c = GETC();

			// flag the start of an arrow key sequence
			if (c == ARROW_KEY_SEQUENCE_ONE) {
				flagOne = true;
			}
			// check if we're in a sequence if flagOne is set - do things if necessary
			else if (flagOne) {
				if (flagTwo) {
					switch (c) {
						case ARROW_UP_KEY:
							PRINTF("\033M");
							break;

						case ARROW_DOWN_KEY:
							PRINTF("\033D");
							break;

						default:
							flagOne = false;
							flagTwo = false;
					}

					Flush();
					continue;

				} else {	// flagTwo not set
					switch (c) {
						case ARROW_KEY_SEQUENCE_TWO:
							flagTwo = true;
							break;

						default:
							flagOne = false;
							break;
					}
				}
			}

			// if the buffer is full, ignore the chracter and print a
			// warning to the console
			if (rxIndex >= BUFFER_LENGTH - 1 && c != BACKSPACE_FLAG_CHAR) {
				PRINTF("%s\r\n", RX_BUFFER_FULL_MSG.c_str());
				Flush();
			}

			//if a new line character is sent, process the current buffer
			else if (c == NEW_LINE_CHAR) {
				// print new line prior to executing
				PRINTF("%c\n", NEW_LINE_CHAR);
				Flush();
				rxBuffer[rxIndex] = '\0';

				command_ready = true;
				command_handled = false;
			}

			//if a backspace is requested, handle it.
			else if (c == BACKSPACE_FLAG_CHAR && rxIndex > 0) {
				//re-terminate the string
				rxBuffer[--rxIndex] = '\0';

				// 1) Move cursor back
				// 2) Write a space to clear the character
				// 3) Move back cursor again
				PUTC(BACKSPACE_REPLY_CHAR);
				PUTC(BACKSPACE_REPLACE_CHAR);
				PUTC(BACKSPACE_REPLY_CHAR);
				Flush();
			}

			// set that a command break was requested flag if we received a break character
			else if (c == BREAK_CHAR) {
				iter_break_req = true;
			}

			// No special character, add it to the buffer and return it to
			// the terminal to be visible.
			else {
				rxBuffer[rxIndex++] = c;
				PUTC(c);
				Flush();
			}
		}
	}
}

void Console::TXCallback(void)
{
	NVIC_DisableIRQ(UART0_IRQn);
	//handle transmission interrupts if necessary here
	NVIC_EnableIRQ(UART0_IRQn);
}

void Console::ConComCheck(void)
{
	/*
	 * Currently no way to check if a vbus has been disconnected or
	 * reconnected. Will continue to keep track of a beta library located
	 * here: http://developer.mbed.org/handbook/USBHostSerial
	 * Source here: http://developer.mbed.org/users/mbed_official/code/USBHost/file/607951c26872/USBHostSerial/USBHostSerial.cpp
	 * It's not present in our currently library, and is not working
	 * for most people. It could however, greatly reduce what we have to
	 * implement while adding more functionality.
	 */

	/*
	 * Note for the above ^. The vbus can be monitored through ADC0, input 4.
	 * This is possible when bits 29..28 of the PINSEL3 register are set to 0b01.
	 *
	 * 		- Jon
	 */
	return;
}

void Console::RequestSystemStop(void)
{
	Instance()->sysStopReq = true;
	instance.reset();
}

bool Console::IsSystemStopRequested(void)
{
	return Instance()->sysStopReq;
}

bool Console::IterCmdBreakReq(void)
{
	return iter_break_req;
}

void Console::IterCmdBreakReq(bool newState)
{
	iter_break_req = newState;

	// Print out the header if an iterating command is stopped
	if (newState == false) {
		instance->PRINTF("%s", COMMAND_BREAK_MSG.c_str());
		PrintHeader();
	}
}

char* Console::rxBufferPtr(void)
{
	return instance->rxBuffer;
}

bool Console::CommandReady(void)
{
	return command_ready;
}

void Console::CommandHandled(bool cmdDoneState)
{
	// update the class's flag for if a command was handled or not
	command_handled = cmdDoneState;

	// Clean up after command execution
	instance->rxIndex = 0;

	// reset our outgoing flag saying if there's a valid command sequence in the RX buffer or now
	command_ready = false;

	// print out the header without a newline first
	if (iter_break_req == false)
		instance->PRINTF("%s", instance->CONSOLE_HEADER.c_str());
}

void Console::changeHostname(const std::string& hostname)
{
	instance->CONSOLE_HOSTNAME = hostname;
}

void Console::changeUser(const std::string& user)
{
	instance->CONSOLE_USER = user;
}

void Console::Baudrate(uint16_t baud)
{
	instance->baudrate = baud;
	instance->pc.baud(instance->baudrate);
}

uint16_t Console::Baudrate(void)
{
	return instance->baudrate;
}
