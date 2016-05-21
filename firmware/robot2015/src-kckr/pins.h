/* Inputs */
#define V_MONITOR PA2 // pin 11
#define N_KICK_CS PA7 // pin 6
// buttons - these are pull downs
#define DB_KICK PB0 // should send a normal kick commmand
#define DB_CHIP PB1 // should send a normal chip command
#define DB_CHG PB2 // should send an enable charging command

/* Interrupts for PCMASK0 or PCMASK1 */
#define INT_N_KICK_CS PCINT7
#define INT_DB_KICK PCINT8
#define INT_DB_CHIP PCINT9
#define INT_DB_CHG PCINT10

/* Outputs */
#define KICK PA0 // pin 13
#define CHIP PA1 // pin 12
#define MISO PA5 // pin 8
#define CHARGE PA3 // pin 10
