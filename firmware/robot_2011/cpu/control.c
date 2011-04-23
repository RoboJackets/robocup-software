#include <stdio.h>

#include "control.h"
#include "fpga.h"
#include "main.h"
#include "console.h"
#include "power.h"

////////

static void dumb_update()
{
	for (int i = 0; i < 4; ++i)
	{
		wheel_out[i] = -wheel_command[i];
	}
	dribble_out = dribble_command;
}

////////

static int step_motor;
static int step_level = 0;

static void step_init(int argc, const char *argv[])
{
	if (argc == 2)
	{
		step_motor = parse_uint32(argv[0]);
		step_level = parse_uint32(argv[1]);
	}
	
	if (argc != 2 || step_motor < 0 || step_motor > 3 || step_level < 0 || step_level > 127)
	{
		printf("Usage: run step <motor 0..3> <level 0..127>\n");
		controller = 0;
	}
}

static void step_update()
{
	int supply_mv = supply_raw * VBATT_NUM / VBATT_DIV;
	printf("%2d.%03d %5d\n", supply_mv / 1000, supply_mv % 1000, encoder_delta[2]);
	wheel_out[step_motor] = step_level;
}

////////

#define Log_Count 100
static int log_pos;
static int log_level;
static struct
{
	int16_t delta[4];
} log_data[Log_Count];
static uint8_t log_started;

static void log_init(int argc, const char *argv[])
{
	log_pos = 0;
	log_started = 0;
	if (argc)
	{
		log_level = parse_uint32(argv[0]);
	} else {
		printf("Usage: run log <-127..127> [start]\n");
		controller = 0;
	}
	
	if (argc > 1)
	{
		log_started = 1;
	}
}

static void log_update()
{
	if (log_pos == Log_Count)
	{
		log_started = 0;
	} else if (kick_strength)
	{
		log_started = 1;
	}
	
	if (log_started)
	{
		for (int i = 0; i < 4; ++i)
		{
			log_data[log_pos].delta[i] = encoder_delta[i];
		}
		++log_pos;
		
		wheel_out[0] = -log_level;
		wheel_out[1] = -log_level;
		wheel_out[2] = log_level;
		wheel_out[3] = log_level;
	}
}

static void log_print()
{
	controller = 0;
	printf("\n");
	for (int i = 0; i < Log_Count; ++i)
	{
		printf("%3d", i);
		for (int j = 0; j < 4; ++j)
		{
			printf(" %5d", log_data[i].delta[j]);
		}
		printf("\n");
	}
	printf("DONE\n");
}

const controller_info_t controllers[] =
{
	{"dumb", 0, 0, dumb_update},
	{"step", step_init, 0, step_update},
	{"log", log_init, 0, log_update},
	{"log_print", 0, 0, log_print},
	
	// End of table
	{0, 0}
};

const controller_info_t *controller = 0;
