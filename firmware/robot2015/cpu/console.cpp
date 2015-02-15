#include "robot.hpp"
#include "console.hpp"
#include "commands.hpp"

using namespace std;

/**
 * max buffer length. Default 252 (three lines)
 */
const uint8_t BUFFER_LENGTH = 252;

/**
 * new line character. Default '\r'
 */
const char NEW_LINE_CHAR = 13;  //ASCII CR (\r) (0x0D)

/**
 * backspace flag char. (What char does the console send when the backspace key
 * is pressed?). Default DEL
 */
const char BACKSPACE_FLAG_CHAR = 127; //ASCII DEL (0x7F)

/**
 * backspace reply char. (What char causes screen to delete the last 
 * character?). Default '\b'
 */
const char BACKSPACE_REPLY_CHAR = 8;   //ASCII BK (\b) (0x08)

/**
 * when the console backspaces, what does the last character become. Default ' '
 */
const char BACKSPACE_REPLACE_CHAR = 32;

/**
 * default ETX (0x3)
 */
const char BREAK_CHAR = 3;

/**
 * define the sequence for arrow key flags
 */
const char ARROW_KEY_SEQUENCE_ONE = 27;
const char ARROW_KEY_SEQUENCE_TWO = 91;

/**
 * arrow up value, following arrow key flag
 */
const char ARROW_UP_KEY = 65;

/**
 * arrow down value, following arrow key flag
 */
const char ARROW_DOWN_KEY = 66;

/**
 * console header string.
 */
const string CONSOLE_HEADER = "> ";

/**
 * receice buffer full error message
 */
const string RX_BUFFER_FULL_MSG = "RX BUFFER FULL";

/**
 * break message
 */
const string COMMAND_BREAK_MSG = "*BREAK*";

/**
 * Serial (over USB) baud rate. Default 9600. Screen default 9600
 */
const uint16_t BAUD_RATE = 9600;

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

/**
 * receive buffer
 */
char rxBuffer[BUFFER_LENGTH];

/**
 * transmission buffer
 */
char txBuffer[BUFFER_LENGTH];

/**
 * serial connection
 */
Serial pc(USBTX, USBRX);

/**
 * debug leds
 */
#if defined(DEBUG)
DigitalOut ledThree(LED3);
DigitalOut ledFour(LED4);
#endif

/**
 * clear recieve buffer
 */
void clearRXBuffer(void)
{
	memset(rxBuffer, '\0', BUFFER_LENGTH);
}

/**
 * clear transmission buffer
 */
void clearTXBuffer(void)
{
	memset(txBuffer, '\0', BUFFER_LENGTH);
}

/**
 * flushes stdout. Should be called after every putc or printf block.
 */
#if defined(_cplusplus)
extern "C"
#endif
void flush(void)
{
	fflush(stdout);
}

/**
 * Serial (over USB) receive interrupt handler.
 */
#if defined(_cplusplus)
extern "C"
#endif
void rx(void)
{
	//flash receive led
	#if defined(DEBUG)
	ledThree = !ledThree;
	#endif

	//if for some reason more than one character is in the buffer when the
	//interrupt is called, handle them all.
	while (pc.readable())
	{
		//read the char that caused the interrupt
		char c = pc.getc();		

		//clear flags if the sequence is broken
		if (flagOne && !flagTwo && c != ARROW_KEY_SEQUENCE_TWO)
		{
			flagOne = false;
			flagTwo = false;
		}

		//clear flags if the sequence is broken
		if (flagOne && flagTwo && !(c == ARROW_UP_KEY || ARROW_DOWN_KEY))
		{
			flagOne = false;
			flagTwo = false;
		}

		//if the buffer is full, ignore the chracter and print a
		//warning to the console
		if (rxIndex >= BUFFER_LENGTH - 1 && c != BACKSPACE_FLAG_CHAR)
		{
			pc.printf("%s\r\n", RX_BUFFER_FULL_MSG.c_str());
			flush();
		}
 		//if a new line character is sent, process the current buffer
		else if (c == NEW_LINE_CHAR)
		{
			//print command prior to executing
			pc.printf("\r\n");
			flush();
			rxBuffer[rxIndex] = '\0';

			#if not defined(DISABLE_CMD_FEEDBACK)
			pc.printf("%s\r\n", rxBuffer);
			flush();
			#endif

			//execute rx buffer as command line
			NVIC_DisableIRQ(UART0_IRQn);
			executeCommand(rxBuffer);
			NVIC_EnableIRQ(UART0_IRQn);

			//clean up after command execution
			rxIndex = 0;
			pc.printf(CONSOLE_HEADER.c_str());
			flush();
		}
		//if a backspace is requested, handle it.
		else if (c == BACKSPACE_FLAG_CHAR && rxIndex > 0)
		{
			//re-terminate the string
			rxBuffer[--rxIndex] = '\0';

			//move cursor back, write a space to clear the character
			//move back cursor again
			pc.putc(BACKSPACE_REPLY_CHAR);
			pc.putc(BACKSPACE_REPLACE_CHAR);
			pc.putc(BACKSPACE_REPLY_CHAR);
			flush();
		}
		//if a break is requested, cancel iterative commands
		else if (c == BREAK_CHAR)
		{
			if (isExecutingIterativeCommand())
			{
				cancelIterativeCommand();
				pc.printf("%s\r\n", COMMAND_BREAK_MSG.c_str());
				pc.printf(CONSOLE_HEADER.c_str());
				flush();
			}
		}
		//flag the start of an arrow key sequence
		else if (c == ARROW_KEY_SEQUENCE_ONE)
		{
			flagOne = true;
		}
		//continue arrow sequence
		else if (flagOne && c == ARROW_KEY_SEQUENCE_TWO)
		{
			flagTwo = true;
		}
		//process arrow key sequence
		else if (flagOne && flagTwo)
		{
			//process keys
			if (c == ARROW_UP_KEY)
			{
				printf("\033M");
			}
			else if (c == ARROW_DOWN_KEY)
			{
				printf("\033D");
			}
			flush();
	
			flagOne = false;
			flagTwo = false;
		}
 		//no special character, add it to the buffer and return it to
		//to the terminal to be visible
		else
		{
			rxBuffer[rxIndex++] = c;
			pc.putc(c);
			flush();
		}
	}	
}

/**
 * Serial (over USB) transmission interrupt.
 */
#if defined(_cplusplus)
extern "C"
#endif
void tx(void)
{
	#if defined(DEBUG)
	ledFour = !ledFour;
	#endif

	NVIC_DisableIRQ(UART0_IRQn);
	//handle transmission interrupts if necessary here
	NVIC_EnableIRQ(UART0_IRQn);
}

/**
 * Console initialization routine. Attaches interrupt handlers and clears the 
 * buffers.
 */
void initConsole(void)
{
	flush();
	printf(ENABLE_SCROLL_SEQ.c_str());
	printf(CLEAR_SCREEN_SEQ.c_str());
	flush();

	//clear buffers
	clearRXBuffer();
	clearTXBuffer();
	
	//set baud rate
	pc.baud(BAUD_RATE);
	//attach interrupt handlers
	pc.attach(&rx, Serial::RxIrq);
	pc.attach(&tx, Serial::TxIrq);

	//print OK.
	pc.printf("OK.\r\n");
	flush();

	//reset indicces
	rxIndex = 0;
	txIndex = 0;

	//print header
	pc.printf(CONSOLE_HEADER.c_str());
	flush();
}

/**
 * console communications check. should be called in the main loop.
 */
void conComCheck(void)
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
	return;		
}

/**
 * requests the main loop break
 */
void reqSysStop(void)
{
	sysStopReq = true;
}

/**
 * returns if the main loop should break
 */
bool isSysStopReq(void)
{
	return sysStopReq;
}

