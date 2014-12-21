#include "mbed.h"
#include "console.h"

DigitalOut ledOne(LED1);
DigitalOut ledTwo(LED2);
Serial pc(USBTX, USBRX);


char txBuf[84];
char rxBuf[84];

void rx()
{
	ledOne = 1;
	NVIC_DisableIRQ(UART1_IRQn);
	//pc.getc();
	pc.printf("Echo: %c\n", pc.getc());
	NVIC_EnableIRQ(UART1_IRQn);
	ledOne = 0;	
}

void tx()
{
	ledTwo = 1;
	ledTwo = 0;
}

int main() 
{
	//pc.attach(&cmdHandler);
	//pc.attach(&tx, Serial::TxIrq);
	pc.attach(&rx, Serial::RxIrq);
	//myled = 1;

    	for (;;) 
	{
		//wait(0.5);
		//myled = !myled;

		/*
		while (pc.readable())
		{
			myled = !myled;

			pc.printf("echo: %s\n", pc.getc());
		}
		*/
    	}
}
