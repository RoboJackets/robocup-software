#include <stdio.h>

#include "control.h"
#include "fpga.h"
#include "main.h"
#include "console.h"
#include "power.h"

const controller_info_t *controller = 0;

static void dumb_update()
{
	for (int i = 0; i < 4; ++i)
	{
		wheel_out[i] = -wheel_command[i];
	}
	dribble_out = dribble_command;
}

static int step_level = 0;

static void step_init(int argc, const char *argv[])
{
	if (argc)
	{
		step_level = parse_uint32(argv[0]);
	} else {
		printf("Usage: run step <0..127>\n");
		controller = 0;
	}
}

static void step_update()
{
	int supply_mv = supply_raw * VBATT_NUM / VBATT_DIV;
	printf("%2d.%03d %5d\n", supply_mv / 1000, supply_mv % 1000, encoder_delta[2]);
	wheel_out[2] = step_level;
}

const controller_info_t controllers[] =
{
	{"dumb", 0, 0, dumb_update, 0},
	{"step", step_init, 0, step_update, 0},
	
	// End of table
	{0, 0}
};
