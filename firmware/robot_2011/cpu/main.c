#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <board.h>
#include <stdio.h>

#include "timer.h"
#include "command.h"
#include "sound.h"
#include "write.h"
#include "spi.h"
#include "tools/reflash.h"
#include "cc1101.h"
#include "status.h"
#include "radio.h"

//FIXME - Precision reference
#define VBATT_RH	90000
#define VBATT_RL	10000

// Numerator and denominator for Vbatt raw-to-millivolts conversion ratio
#define VBATT_NUM	(33 * (VBATT_RL + VBATT_RH))
#define VBATT_DIV	(10230 * VBATT_RL / 1000)

#if VBATT_NUM >= (1 << 32) || VBATT_DIV >= (1 << 32)
#error VBATT_NUM out of range
#endif

#define MV_TO_RAW(v) (v * VBATT_DIV / VBATT_NUM)

// Supply voltage below which we believe that the motor fuse is blown.
#define FUSE_BLOWN_RAW	MV_TO_RAW(2500)

// Minimum supply voltage, converted from millivolts.
// This is determined by the minimum safe discharge voltage of the LiPo pack.
#define MIN_SUPPLY_RAW	MV_TO_RAW(10000)

// Maximum supply voltage, converted from millivolts.
// This is determined by the maximum voltage that the TC4428's can tolerate.
// This has a large safety margin because we may not detect spikes from regenerative
// braking quickly enough to protect the circuitry.
#define MAX_SUPPLY_RAW	MV_TO_RAW(16000)

extern const char git_version[];

unsigned int run_robot;
unsigned int robot_id;
unsigned int failures;
uint8_t motor_faults;

// Last forward packet
uint8_t forward_packet[Forward_Size];

// Last reverse packet
uint8_t reverse_packet[Reverse_Size];

int8_t wheel_command[4];
uint8_t dribble;

uint32_t rx_lost_time;
int8_t last_rssi;

// Time the motor outputs were last updated
unsigned int motor_time;

// Raw power supply voltage measurements:
// Last, minimum since reset, maximum since reset
int supply_raw, supply_min, supply_max;

// Last time the supply voltage was above MIN_SUPPLY_RAW
int supply_good_time;

int disable_power_music;

int hack;

int fpga_init();

static void cmd_help(int argc, const char *argv[], void *arg)
{
	printf("Commands:\n");
	for (int i = 0; commands[i].name; ++i)
	{
		printf("  %s\n", commands[i].name);
	}
}

static void cmd_reflash(int argc, const char *argv[], void *arg)
{
	unsigned int len;
	
	if (argc != 1)
	{
		printf("Use the host-side reflash script\n");
		return;
	}
	
	music_stop();
	
	len = parse_uint32(argv[0]);
	
	// Set FMR for timing and auto-erase
	AT91C_BASE_MC->MC_FMR = 0x00340100;
	
	printf("GO %08x\n", len);
	
	// Disable interrupts
	AT91C_BASE_AIC->AIC_IDCR = ~0;
	
	// After this point, we're committed to running the reflasher.
	// RAM contents are forfeit.  Only the stack should be considered usable.
	
	//FIXME - Copy reflash to its final location in SRAM.
	//  For now, it's always there thanks to the relocate section, but this wastes memory.
	//  This would be unnecessary if we are running from SRAM to begin with.
	
	// This never returns
	reflash_main(len);
}

//FIXME - This can be smaller
const char *const motor_names[5] =
{
	"BL",
	"FL",
	"FR",
	"BR",
	"DR"
};

// Prints a power supply measurement with a label
static void print_supply(const char *label, int raw)
{
	int supply_mv = raw * VBATT_NUM / VBATT_DIV;
	printf("%s: %d.%03dV\n", label, supply_mv / 1000, supply_mv % 1000);
}

static void cmd_status(int argc, const char *argv[], void *arg)
{
	printf("Robot ");
	if (!run_robot)
	{
		printf("NOT ");
	}
	printf("running\n");
	
	printf("Robot ID %X\n", robot_id);
	printf("Reset type %x\n", (AT91C_BASE_SYS->RSTC_RSR >> 8) & 7);
	
	printf("Failures: 0x%08x", failures);
	if (failures & Fail_FPGA)
	{
		printf(" FPGA");
	}
	if (failures & Fail_Radio)
	{
		printf(" Radio");
	}
	if (failures & Fail_Power)
	{
		printf(" Power");
	}
	putchar('\n');
	
	printf("Power:\n");
	print_supply("  Now", supply_raw);
	print_supply("  Min", supply_min);
	print_supply("  Max", supply_max);
	
	printf("Motor faults: 0x%02x", motor_faults);
	if (motor_faults)
	{
		for (int i = 0; i < 5; ++i)
		{
			if (motor_faults & (1 << i))
			{
				putchar(' ');
				printf(motor_names[i]);
			}
		}
	} else {
		printf(" None");
	}
	putchar('\n');
	
	printf("GIT version: %s\n", git_version);
}

static void cmd_timers(int argc, const char *argv[], void *arg)
{
	printf("timer_t    time       period\n");
	//      0x01234567 0x01234567 0x01234567
	for (timer_t *t = first_timer; t; t = t->next)
	{
		printf("%p 0x%08x 0x%08x\n", t, t->time, t->period);
	}
}

static void cmd_print_uint32(int argc, const char *argv[], void *arg)
{
	printf("0x%08x\n", *(unsigned int *)arg);
}

static void cmd_spi_test(int argc, const char *argv[], void *arg)
{
	spi_select(NPCS_FLASH);

	spi_xfer(0xab);
	spi_xfer(0);
	spi_xfer(0);
	spi_xfer(0);
	printf("Signature: 0x%02x\n", spi_xfer(0));
	spi_deselect();
	
	spi_xfer(0x05);
	printf("Status:    0x%02x\n", spi_xfer(0));
	spi_deselect();
}

static int spi_wait(int max)
{
	spi_xfer(0x05);
	for (unsigned int  start_time = current_time; (current_time - start_time) < max;)
	{
		uint8_t status = spi_xfer(0);
		if (!(status & 1))
		{
			spi_deselect();
			return 1;
		}
	}
	printf("*** Timeout!\n");
	spi_deselect();
	return 0;
}

static void cmd_spi_erase(int argc, const char *argv[], void *arg)
{
	spi_select(NPCS_FLASH);
	
	// Bulk erase
	// WREN
	spi_xfer(0x06);
	spi_deselect();

	// BE
	spi_xfer(0xc7);
	spi_deselect();

	// Wait for completion
	spi_wait(4000);
}

static void cmd_spi_write(int argc, const char *argv[], void *arg)
{
	uint32_t addr, len;
	int rx_pos;
	
	if (argc != 2)
	{
		printf("spi_write <address> <length>\n");
		return;
	}
	
	addr = parse_uint32(argv[0]);
	len = parse_uint32(argv[1]);
	
	usb_rx_len = 0;
	rx_pos = 0;
	
	// Program one page at a time
	spi_select(NPCS_FLASH);
	while (len)
	{
		// WREN
		spi_xfer(0x06);
		spi_deselect();
		
		// PP
		spi_xfer(0x02);
		spi_xfer(addr >> 16);
		spi_xfer(addr >> 8);
		spi_xfer(addr);
		
		// Program up to the end of the page or the end of the data
		do
		{
			// Wait for more data if necessary
			if (rx_pos == usb_rx_len)
			{
				usb_rx_start();
				while (!usb_rx_len);
				rx_pos = 0;
			}
			
			spi_xfer(usb_rx_buffer[rx_pos++]);
			++addr;
			--len;
		} while ((addr & 0x7f) && len);
		
		spi_deselect();
		
		if (!spi_wait(5))
		{
			return;
		}
	}
	//FIXME - I don't like how this fucntion handles input.
	//  There should be an input buffer.  Currently we depend on there not being extra input.
	usb_rx_len = 0;
	printf("OK\n");
}

static void cmd_spi_read(int argc, const char *argv[], void *arg)
{
	uint32_t addr, len;
	
	if (argc != 2)
	{
		printf("spi_read <address> <length>\n");
		return;
	}
	
	addr = parse_uint32(argv[0]);
	len = parse_uint32(argv[1]);
	
	spi_select(NPCS_FLASH);
	spi_xfer(0x03);
	spi_xfer(addr >> 16);
	spi_xfer(addr >> 8);
	spi_xfer(addr);
	
	for (; len; --len)
	{
		putchar_raw(spi_xfer(0));
	}
	flush_stdout();
	
	spi_deselect();
}

static void cmd_fpga_test(int argc, const char *argv[], void *arg)
{
	spi_select(NPCS_FPGA);
	
	for (int i = 0; i < argc; ++i)
	{
		uint8_t byte = parse_uint32(argv[i]);
		printf("0x%02x\n", spi_xfer(byte));
	}
	
	spi_deselect();
}

static void cmd_fpga_on(int argc, const char *argv[], void *arg)
{
	switch (fpga_init())
	{
		case 0:
			music_start(song_failure);
			printf("*** Failed\n");
			break;
		
		case 1:
			music_start(song_startup);
			printf("Configured\n");
			break;
		
		case 2:
			printf("Already configured\n");
			break;
	}
}

static void cmd_fpga_reset(int argc, const char *argv[], void *arg)
{
	AT91C_BASE_PIOA->PIO_CODR = MCU_PROGB;
	delay_ms(1);
	cmd_fpga_on(0, 0, 0);
}

static void cmd_read_word(int argc, const char *argv[], void *arg)
{
	if (argc != 1)
	{
		printf("rw <addr>\n");
		return;
	}
	
	uint32_t addr = parse_uint32(argv[0]);
	printf("0x%08x\n", *(unsigned int *)addr);
}

static void cmd_write_word(int argc, const char *argv[], void *arg)
{
	if (argc != 2)
	{
		printf("ww <addr> <value>\n");
		return;
	}
	
	uint32_t addr = parse_uint32(argv[0]);
	uint32_t value = parse_uint32(argv[1]);
	*(uint32_t *)addr = value;
}

static void cmd_delay(int argc, const char *argv[], void *arg)
{
	if (argc != 1)
	{
		printf("delay <ms>\n");
		return;
	}
	delay_ms(parse_uint32(argv[0]));
}

static void cmd_radio_start(int argc, const char *argv[], void *arg)
{
	radio_command(SIDLE);
	radio_command(SFRX);
	radio_command(SRX);
}

static void cmd_last_rx(int argc, const char *argv[], void *arg)
{
// 	printf("RSSI %02x\n", (uint8_t)last_rssi);
 	printf("RSSI %d dBm\n", (int)last_rssi / 2 - 74);
	
	printf("%02x %02x %02x\n", forward_packet[0], forward_packet[1], forward_packet[2]);
	for (int i = 0; i < 5; ++i)
	{
		int off = 3 + i * 5;
		printf("%02x %02x %02x %02x %02x\n",
			   forward_packet[off],
			   forward_packet[off + 1],
			   forward_packet[off + 2],
			   forward_packet[off + 3],
			   forward_packet[off + 4]);
	}
	putchar('\n');
}

static void cmd_stfu(int argc, const char *argv[], void *arg)
{
	if (argc)
	{
		disable_power_music = parse_uint32(argv[0]);
	} else {
		disable_power_music = 1;
	}
	
	// Stop any music that may be playing
	music_stop();
}

static const note_t *const test_songs[] = {song_startup, song_failure, song_overvoltage, song_undervoltage, song_fuse_blown};
#define NUM_TEST_SONGS (sizeof(test_songs) / sizeof(test_songs[0]))

static void cmd_music(int argc, const char *argv[], void *arg)
{
	int n;
	if (argc != 1 || (n = parse_uint32(argv[0])) >= NUM_TEST_SONGS)
	{
		printf("music <0-%d>\n", (int)NUM_TEST_SONGS - 1);
		return;
	}
	
	music_start(test_songs[n]);
}

static void cmd_tone(int argc, const char *argv[], void *arg)
{
	AT91C_BASE_PWMC->PWMC_DIS = 1 << 3;
	if (argc == 1)
	{
		int period = PERIOD(parse_uint32(argv[0]));
		AT91C_BASE_PWMC->PWMC_CH[3].PWMC_CPRDR = period;
		AT91C_BASE_PWMC->PWMC_CH[3].PWMC_CDTYR = period / 2;
		AT91C_BASE_PWMC->PWMC_ENA = 1 << 3;
		printf("%d\n", period);
	}
}

static void cmd_fail(int argc, const char *argv[], void *arg)
{
	if (argc != 1)
	{
		printf("fail <flags>\n");
		return;
	}
	
	failures = parse_uint32(argv[0]);
}

static const write_int_t write_fpga_off = {&AT91C_BASE_PIOA->PIO_CODR, MCU_PROGB};
static const write_int_t write_reset = {AT91C_RSTC_RCR, 0xa5000005};
static const write_int_t write_run = {&run_robot, 1};

const command_t commands[] =
{
	{"help", cmd_help},
	{"status", cmd_status},
	{"reflash", cmd_reflash},
	{"reset", cmd_write_int, (void *)&write_reset},
	{"rw", cmd_read_word},
	{"ww", cmd_write_word},
	{"stfu", cmd_stfu},
	{"run", cmd_write_int, (void *)&write_run},
	{"delay", cmd_delay},
	{"time", cmd_print_uint32, (void *)&current_time},
	{"chip_id", cmd_print_uint32, (void *)AT91C_DBGU_CIDR},
	{"hack", cmd_print_uint32, (void *)&hack},
	{"inputs", cmd_print_uint32, (void *)&AT91C_BASE_PIOA->PIO_PDSR},
	{"timers", cmd_timers},
	{"fpga_reset", cmd_fpga_reset},
	{"fpga_off", cmd_write_int, (void *)&write_fpga_off},
	{"fpga_on", cmd_fpga_on},
	{"fpga_test", cmd_fpga_test},
	{"spi_test", cmd_spi_test},
	{"spi_erase", cmd_spi_erase},
	{"spi_write", cmd_spi_write},
	{"spi_read", cmd_spi_read},
	{"radio_configure", (void *)radio_configure},
	{"radio_start", cmd_radio_start},
	{"last_rx", cmd_last_rx},
	{"music", cmd_music},
	{"tone", cmd_tone},
	{"fail", cmd_fail},

	// End of list placeholder
	{0, 0}
};

// Initializes and tests the FPGA.
// This does not force the FPGA to reconfigure, but waits for it to finish.
//
// Returns 0 on failure, 1 on successful configuration, or 2 if the FPGA was already configured.
int fpga_init()
{
	int ret;
	
	failures &= ~Fail_FPGA;
	
	// Disable SPI drivers so the FPGA can reconfigure
	spi_shutdown();
	
	// Release PROGB so the FPGA can start configuring
	AT91C_BASE_PIOA->PIO_SODR = MCU_PROGB;
	
	// Wait for the FPGA to start configuring
	delay_ms(5);
	if (!(AT91C_BASE_PIOA->PIO_PDSR & FLASH_NCS))
	{
		// FLASH_NCS is low: the FPGA is reading
		//
		// Wait for the FPGA to finish configuring
		for (int i = 0; i < 100; ++i)
		{
			delay_ms(10);
			if (AT91C_BASE_PIOA->PIO_PDSR & FLASH_NCS)
			{
				// FLASH_NCS is high: the FPGA is done
				//FIXME - Read version register
				ret = 1;
				goto good;
			}
		}
		
		// The FPGA took too long to configure.
		// Configuration memory is probably empty/corrupt.
		// Shut down the FPGA, since the MCU needs to become the SPI master
		AT91C_BASE_PIOA->PIO_CODR = MCU_PROGB;
		
		failures |= Fail_FPGA_Config;
		
		// Become the SPI master
		spi_init();
		
		return 0;
	} else {
		// FPGA did not start reading SPI flash - already configured?
		ret = 2;
	}
good:
	// Become the SPI master
	spi_init();

	//FIXME - Read version
	spi_select(NPCS_FPGA);
	uint8_t s0 = spi_xfer(0);
	uint8_t s1 = spi_xfer(0);
	spi_deselect();
	
	if (s0 != 0xc9 || s1 != 0xa5)
	{
		failures |= Fail_FPGA_Logic;
		failures |= Fail_FPGA_Version;
	}
	
	return ret;
}

static int forward_packet_received()
{
	uint8_t bytes = radio_read(RXBYTES);
	AT91C_BASE_PIOA->PIO_ODSR ^= LED_RY;
	
	if (bytes != (Forward_Size + 2))
	{
		// Bad CRC, so the packet was flushed (or the radio got misconfigured).
		radio_command(SFRX);
		radio_command(SRX);
		return 0;
	}
	
	// Read the packet from the radio
	radio_select();
	spi_xfer(RXFIFO | CC_READ | CC_BURST);
	for (int i = 0; i < Forward_Size; ++i)
	{
		forward_packet[i] = spi_xfer(SNOP);
	}
	
	// Read status bytes
	last_rssi = spi_xfer(SNOP);
	uint8_t status = spi_xfer(SNOP);
	radio_deselect();
	
	if (!(status & 0x80))
	{
		// Bad CRC
		//
		// Autoflush is supposed to be on so this should never happen.
		// If we get here and autoflush is on, this means some bytes have been lost
		// and the status byte isn't really the status byte.
		radio_command(SFRX);
		radio_command(SRX);
		return 0;
	}
	
	rx_lost_time = current_time;
	AT91C_BASE_PIOA->PIO_ODSR ^= LED_RG;
	AT91C_BASE_PIOA->PIO_SODR = LED_RR;
	
// 	uint8_t reverse_id = forward_packet[0] & 15;
	
	// Update sequence number history
// 	uint8_t sequence = forward_packet[0] >> 4;
	
	// Kicking
	uint8_t kick_id = forward_packet[1] & 15;
	if (kick_id == robot_id && forward_packet[2])
	{
// 		AT91C_BASE_PIOA->PIO_CODR = LED_RY;
	} else {
// 		AT91C_BASE_PIOA->PIO_SODR = LED_RY;
	}
	
	// Kick/chip selection
/*	kicker_control &= ~0x20;
	if (forward_packet[1] & 0x10)
	{
	    kicker_control |= 0x20;
	}
	fpga_write(FPGA_Kicker_Status, kicker_control);*/

#if 0
	// Clear history bits for missed packets
	for (int i = (last_sequence + 1) & 15; i != sequence; i = (i + 1) & 15)
	{
		sequence_history &= ~(1 << i);
	}
	
	// Set the history bit for this packet
	sequence_history |= 1 << sequence;
	
	// Save this packet's sequence number for next time
	last_sequence = sequence;
	
	// Count lost packets
	int lost_packets = 0;
	for (int i = 0; i < 16; ++i)
	{
		if (!(sequence_history & (1 << i)))
		{
			++lost_packets;
		}
	}
#endif
	
	// Clear motor commands in case this robot's ID does not appear in the packet
	for (int i = 0; i < 4; ++i)
	{
		wheel_command[i] = 0;
	}
	dribble = 0;

	// Get motor commands from the packet
	int offset = 3;
	for (int slot = 0; slot < 5; ++slot)
	{
		if ((forward_packet[offset + 4] & 0x0f) == robot_id)
		{
			for (int i = 0; i < 4; ++i)
			{
				wheel_command[i] = (int8_t)forward_packet[offset + i];
			}
			dribble = forward_packet[offset + 4] >> 4;
		}
		offset += 5;
	}

#if 0
	if (reverse_id == board_id)
	{
		// Build and send a reverse packet
		radio_write(PKTLEN, Reverse_Size);
		radio_command(SFTX);
		
		uint8_t fault = fpga_read(FPGA_Fault) & 0x1f;
		
		// Clear fault bits
		fpga_write(FPGA_Fault, fault);
		
		reverse_packet[0] = (lost_packets << 4) | board_id;
		reverse_packet[1] = last_rssi;
		reverse_packet[2] = 0x00;
		reverse_packet[3] = battery >> 2;
		reverse_packet[4] = fpga_read(FPGA_Kicker_Status);
		
		// Fault bits
		reverse_packet[5] = fault;
		
		if (ball_present)
		{
			reverse_packet[5] |= 1 << 5;
		}
		
		if (ball_sensor_fail)
		{
			reverse_packet[5] |= 1 << 6;
		}
		
		reverse_packet[10] = 0;
		for (int i = 0; i < 4; ++i)
		{
		    reverse_packet[6 + i] = last_tick[i];
		    reverse_packet[10] |= (last_tick[i] & 0x300) >> (8 - i * 2);
		}

		radio_select();
		spi_xfer(TXFIFO | CC_BURST);
		for (int i = 0; i < Reverse_Size; ++i)
		{
			spi_xfer(reverse_packet[i]);
		}
		radio_deselect();

		// Start transmitting.  When this finishes, the radio will automatically switch to RX
		// without calibrating (because it doesn't go through IDLE).
		radio_command(STX);
		
		in_reverse = 1;
	} else {
		// Get ready to receive another forward packet
		radio_command(SRX);
	}
#else
	radio_command(SRX);
#endif
	
	return 1;
}

// One main loop iteration for robot operations
static void robot_main(int have_forward)
{
}

// Detects and handles USB connection/disconnection
enum
{
	USB_Disconnected,
	USB_Connected,
	USB_Working
} usb_state = USB_Disconnected;

static void check_usb_connection()
{
	int vbus = AT91C_BASE_PIOA->PIO_PDSR & VBUS;
	if (vbus && usb_state == USB_Disconnected)
	{
		// Initialize the USB controller.
		// This will disconnect from the host (important if we have just reset after reprogramming).
		// The host will notice disconnection while we check the FPGA below.
		CDCDSerialDriver_Initialize();
		USBD_Connect();
		usb_state = USB_Connected;
	}
	if (!vbus)
	{
		usb_state = USB_Disconnected;
	}
	
	if (usb_state == USB_Connected && USBD_GetState() >= USBD_STATE_CONFIGURED)
	{
		command_init();
		usb_state = USB_Working;
	}
	
	if (usb_state == USB_Working)
	{
		if (!command_run())
		{
			run_robot = 0;
		}
	}
}

// Called every update cycle to check for power failures
static void check_power()
{
	// Power failures are latched until reset.
	// Undervoltage could disappear briefly when load is removed, but the battery is still weak.
	// Overvoltage indicates a hardware failure  (caps, TVS, battery resistance, etc.) or a design flaw that
	// seriously needs to be fixed.
	
	supply_raw = AT91C_BASE_ADC->ADC_CDR5;
	
	if (supply_raw < FUSE_BLOWN_RAW)
	{
		// If the battery voltage is very low, the motor fuse is probably blown and we can't measure
		// the real battery voltage.
		//
		// We expect that it will not take long for the voltage to drop from MIN_SUPPLY_RAW to FUSE_BLOWN_RAW,
		// or else we will get a spurious undervoltage indication.
		if ((current_time - supply_good_time) >= 1000)
		{
			failures |= Fail_Fuse;
		}
	} else if (supply_raw < MIN_SUPPLY_RAW)
	{
		// If the battery voltage is too low for a long time, indicate an undervoltage failure.
		// The time limit prevents beeping when the power switch is turned off or when
		// accelerating with a weak but safe battery.
		if ((current_time - supply_good_time) >= 1000)
		{
			failures |= Fail_Undervoltage;
		}
	} else {
		supply_good_time = current_time;
		
		// If the supply voltage ever gets too high, even briefly, there is a potential for the
		// motor drivers to fail short.
		if (supply_raw > MAX_SUPPLY_RAW)
		{
			failures |= Fail_Overvoltage;
		}
	}
	
	// Keep track of minimum and maximum voltages since reset.
	if (supply_max == 0)
	{
		// First supply measurement
		supply_min = supply_raw;
		supply_max = supply_raw;
	} else {
		if (supply_raw < supply_min)
		{
			supply_min = supply_raw;
		}
		if (supply_raw > supply_max)
		{
			supply_max = supply_raw;
		}
	}
}

// Called every main loop iteration to restart power-failure music
static void power_fail_music()
{
	if (!music_playing && !disable_power_music)
	{
		// The order that these are checked indicates the priority of the failures:
		//   Overvoltage can damage the control electronics.
		//   A blown fuse indicates damage may have already occurred, and undervoltage can't be detected.
		//   Undervoltage indicates impending damage to the battery.
		if (failures & Fail_Overvoltage)
		{
			music_start(song_overvoltage);
		} else if (failures & Fail_Fuse)
		{
			music_start(song_fuse_blown);
		} else if (failures & Fail_Undervoltage)
		{
			music_start(song_undervoltage);
		}
	}
}

#if 0
// Use this main() to debug startup code, IRQ, or linker script problems.
// You can change SConstruct to build for SRAM because linker garbage collection
// will remove all the large code.
int main()
{
	// If no LEDs turn on, we didn't reach this point.
	// Either startup code is broken, built for the wrong memory (flash but running in SRAM?),
	// or LowLevelInit failed (failed to return properly do to interworking?).
	
	// Turn on all LEDs
	AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOA;
	AT91C_BASE_PIOA->PIO_CODR = LED_ALL;
	AT91C_BASE_PIOA->PIO_OER = LED_ALL;
	
	// Start PIT.  An IRQ will occur in about 1ms.
	timer_init();
	delay_ms(500);
	
	// Turn off all LEDs
	AT91C_BASE_PIOA->PIO_SODR = LED_ALL;
	
	// If the LEDs turn on and stay on, the delay failed (IRQ crashed or did not trigger).
	// If the LEDs turn off, IRQs and startup code are working.
	
	while (1);
}
#else

int main()
{
	// Set up watchdog timer
	AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDRSTEN | AT91C_WDTC_WDDBGHLT | (0xfff << 16) | 0x0ff;
	
	// Enable user reset (reset button)
	AT91C_BASE_SYS->RSTC_RMR = 0xa5000000 | AT91C_RSTC_URSTEN;
	
	timer_init();
	
	// Set up PIOs
	// Initially, FLASH_NCS is a PIO because the FPGA will be driving it.
	// After the FPGA is configured (or we give up on it), FLASH_NCS is assigned to SPI.  This happens in spi_init().
	AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOA;	// Turn on PIO clock
	AT91C_BASE_PIOA->PIO_OWER = LED_ALL;			// Allow LED states to be written directly
	AT91C_BASE_PIOA->PIO_ODR = ~0;					// Disable all outputs
	AT91C_BASE_PIOA->PIO_CODR = LED_ALL;			// Turn all LEDs on
	AT91C_BASE_PIOA->PIO_OER = LED_ALL | BUZZ;		// Enable LED outputs
	// Connect some pins to the PIO controller
	AT91C_BASE_PIOA->PIO_PER = LED_ALL | MCU_PROGB | FLASH_NCS | RADIO_INT | VBUS;
	// Enable and disable pullups
	AT91C_BASE_PIOA->PIO_PPUER = RADIO_INT | FLASH_NCS | MISO | ID0 | ID1 | ID2 | ID3 | DP0 | DP1 | DP2;
	AT91C_BASE_PIOA->PIO_PPUDR = VBUS | M2DIV | M3DIV | M5DIV | BUZZ;
	
	// Set up MCU_PROGB as an open-drain output, initially high
	AT91C_BASE_PIOA->PIO_SODR = MCU_PROGB;
	AT91C_BASE_PIOA->PIO_MDER = MCU_PROGB;
	AT91C_BASE_PIOA->PIO_OER = MCU_PROGB;
	
	// At this point, the FPGA is presumed to be the SPI master.
	// Wait for it to configure and determine if it works.
	// If not, it must be disabled.
	// This calls spi_init.
	fpga_init();
	
	// Find out if the radio works.
	// This tests SPI communications with the radio and tests if
	// the interrupt line is working.
	radio_init();
	
	// Set up the ADC
	//FIXME - Justify these numbers
	AT91C_BASE_ADC->ADC_MR = AT91C_ADC_SLEEP | (0x3f << 8) | (4 << 16) | (2 << 24);
	// Use all channels except 3
	AT91C_BASE_ADC->ADC_CHER = 0xf7;
	// Start the first conversion
	AT91C_BASE_ADC->ADC_CR = AT91C_ADC_START;
	
	// Check for low/high supply voltage
	while (!(AT91C_BASE_ADC->ADC_SR & AT91C_ADC_EOC5));
	check_power();
	
	// Turn off LEDs
	AT91C_BASE_PIOA->PIO_SODR = LED_ALL;
	
	if (failures == 0)
	{
		music_start(song_startup);
		
		// Enough hardware is working, so act like a robot
		run_robot = 1;
	} else if (failures & Fail_Power)
	{
		// Dead battery (probably)
		power_fail_music();
	} else {
		// We're a paperweight
		music_start(song_failure);
	}
	
	// Set up the radio.  After this, it will be able to transmit and receive.
	//FIXME - Multiple channels
	if (!(failures & Fail_Radio))
	{
		radio_configure();
	}
	
	rx_lost_time = current_time;
	
	// Main loop
	while (1)
	{
		// Reset the watchdog timer
		AT91C_BASE_WDTC->WDTC_WDCR = 0xa5000001;
		
		// Handle USB connect/disconnect
		check_usb_connection();
		
		// Read robot ID
		uint32_t inputs = AT91C_BASE_PIOA->PIO_PDSR;
		robot_id = 0;
		if (!(inputs & ID0))
		{
			robot_id = 1;
		}
		if (!(inputs & ID1))
		{
			robot_id |= 2;
		}
		if (!(inputs & ID2))
		{
			robot_id |= 4;
		}
		if (!(inputs & ID3))
		{
			robot_id |= 8;
		}
		
		if (!(failures & Fail_Radio))
		{
			// Flash LED and recalibrate radio when signal is lost
			if ((current_time - rx_lost_time) > 250)
			{
				rx_lost_time = current_time;
				AT91C_BASE_PIOA->PIO_SODR = LED_RG;
				AT91C_BASE_PIOA->PIO_ODSR ^= LED_RR;
				radio_command(SIDLE);
				radio_command(SFRX);
				radio_command(SRX);
				++hack;
			}
			
			// Check for radiio packets
			int have_forward = radio_gdo2() && forward_packet_received();
			
			// Run robot operations
			if (run_robot)
			{
				robot_main(have_forward);
			} else {
				for (int i = 0; i < 4; ++i)
				{
					wheel_command[i] = 0;
				}
				dribble = 0;
			}
		}
		
		// Periodic activities: motor and ADC updates
		if ((current_time - motor_time) >= 5)
		{
			// Send motor speeds to the FPGA
			spi_select(NPCS_FPGA);
			spi_xfer(0x01);
			for (int i = 0; i < 4; ++i)
			{
				int8_t cmd = wheel_command[i];
				
				//FIXME - Select 2008/2010 mechanical base with switch DP0
				cmd = -cmd;
				
				// Convert from 2's-complement to sign-magnitude for FPGA
				uint8_t out;
				if (cmd < 0)
				{
					out = -cmd | 0x80;
				} else {
					out = cmd;
				}
				spi_xfer(out);
			}
			spi_xfer(0x80 | (dribble << 3));
			motor_faults = spi_xfer(0);
			spi_deselect();
			
			motor_time = current_time;
			
			check_power();
			
			// Stop driving if there is a power supply problem
			if (failures & Fail_Power)
			{
				run_robot = 0;
			}
			
			// Start a new set of ADC conversions
			AT91C_BASE_ADC->ADC_CR = AT91C_ADC_START;
		}
		
		// Keep power failure music playing continuously
		power_fail_music();
	}
}

#endif
