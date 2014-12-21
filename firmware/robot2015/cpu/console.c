uint8_t BUFFER_LENGTH = 255;
uint8_t commandLength = 0;
uint8_t rx_buffer[BUFFER_LENGTH];
uint8_t tx_buffer[BUFFER_LENGTH];

bool hasConsole()
{
	return true;
}

void clearRXBuffer()
{
	for (int index = 0; index < BUFFER_LENGTH; index++)
		rx_buffer[index] = '\0';
}

void clearTXBuffer()
{
	for (int index = 0; index < BUFFER_LENGTH; index++)
		tx_buffer[index] = '\0';
}

void consoleInit()
{
	clearRXBuffer();
	clearTXBuffer();
	commandLength = 0;	
}

void consoleRun()
{
	while (pc.readable() && commandLength < BUFFER_LENGTH)
	{
		
	}	
}
