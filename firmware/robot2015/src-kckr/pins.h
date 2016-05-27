/* Inputs */
#define V_MONITOR_PIN PA2  // pin 11
#define N_KICK_CS_PIN PA7  // pin 6
#define MOSI_PIN PA6       // pin 7, not actually used in code
// buttons - these are active low
#define DB_KICK_PIN PB0  // should send a normal kick commmand
#define DB_CHIP_PIN PB1  // should send a normal chip command
#define DB_CHG_PIN PB2   // should send an enable charging command

/* Interrupts for PCMASK0 or PCMASK1 */
#define INT_N_KICK_CS PCINT7
#define INT_DB_KICK PCINT8
#define INT_DB_CHIP PCINT9
#define INT_DB_CHG PCINT10

/* Outputs */
#define KICK_PIN PA0    // pin 13
#define CHIP_PIN PA1    // pin 12
#define MISO_PIN PA5    // pin 8
#define CHARGE_PIN PA3  // pin 10
