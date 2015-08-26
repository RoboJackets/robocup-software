#ifndef _SERIAL_H_
#define _SERIAL_H_

// Sets up UART1 for use with stdio.
// After this returns, printf() and friends work.
void ser_init(void);

// Returns nonzero if the TX ring buffer is empty
// (a write of that size will not block).
char ser_tx_empty(void);

#endif  // _SERIAL_H_
