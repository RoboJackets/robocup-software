#include "console.hpp"

using namespace std;

const string Console::CONSOLE_HEADER = "user@robot $ ";

const string Console::RX_BUFFER_FULL_MSG = "RX BUFFER FULL";

const string Console::COMMAND_BREAK_MSG = "*BREAK*";

shared_ptr<Console> Console::instance;

Console::Console() : pc(USBTX, USBRX) { }

shared_ptr<Console>& Console::Instance()
{
	if(instance.get() == nullptr)
		instance.reset(new Console);
	return instance;
}

void Console::Init()
{
	auto instance = Instance();

	Flush();
	printf(ENABLE_SCROLL_SEQ.c_str());
	printf(CLEAR_SCREEN_SEQ.c_str());
	Flush();

	//clear buffers
	instance->ClearRXBuffer();
	instance->ClearTXBuffer();

	//set baud rate

	instance->pc.baud(BAUD_RATE);
	//attach interrupt handlers
	instance->pc.attach(instance.get(), &Console::RXCallback, Serial::RxIrq);
	instance->pc.attach(instance.get(), &Console::TXCallback, Serial::TxIrq);

	//print OK.
	instance->pc.printf("OK.\r\n");
	Flush();

	//reset indicces
	instance->rxIndex = 0;
	instance->txIndex = 0;

	//print header
	instance->pc.printf(CONSOLE_HEADER.c_str());
	Flush();
}

void Console::ClearRXBuffer()
{
	memset(rxBuffer, '\0', BUFFER_LENGTH);
}

void Console::ClearTXBuffer()
{
	memset(txBuffer, '\0', BUFFER_LENGTH);
}

void Console::Flush()
{
	fflush(stdout);
}

void Console::RXCallback()
{
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
			Flush();
		}
			//if a new line character is sent, process the current buffer
		else if (c == NEW_LINE_CHAR)
		{
			//print command prior to executing
			pc.printf("\r\n");
			Flush();
			rxBuffer[rxIndex] = '\0';

			//execute rx buffer as command line
			NVIC_DisableIRQ(UART0_IRQn);
			executeCommand(rxBuffer);
			NVIC_EnableIRQ(UART0_IRQn);

			//clean up after command execution
			rxIndex = 0;
			pc.printf(CONSOLE_HEADER.c_str());
			Flush();
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
			Flush();
		}
			//if a break is requested, cancel iterative commands
		else if (c == BREAK_CHAR)
		{
			if (isExecutingIterativeCommand())
			{
				cancelIterativeCommand();
				pc.printf("%s\r\n", COMMAND_BREAK_MSG.c_str());
				pc.printf(CONSOLE_HEADER.c_str());
				Flush();
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
			Flush();

			flagOne = false;
			flagTwo = false;
		}
			//no special character, add it to the buffer and return it to
			//to the terminal to be visible
		else
		{
			rxBuffer[rxIndex++] = c;
			pc.putc(c);
			Flush();
		}
	}
}

void Console::TXCallback()
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
	return;		
}

void Console::RequestSystemStop()
{
		Instance()->sysStopReq = true;
}

bool Console::IsSystemStopRequested()
{
	return Instance()->sysStopReq;
}
