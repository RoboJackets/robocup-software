#include <stdio.h>
#include <stdlib.h>

#include "control.h"
#include "fpga.h"
#include "main.h"
#include "console.h"
#include "power.h"
#include "status.h"
#include "stall.h"
#include "radio_protocol.h"
#include "encoder_monitor.h"
#include "stall.h"
#include "imu.h"
#include "timer.h"
#include "invensense/imuFIFO.h"

static const int Command_Rate_Limit = 40;
static int last_out[4];

static const int w0x8 = 136733; //these integers come from the trig functions of the angles of the robots' wheels, multiplied by 100000
static const int w0y8 = 146628;
static const int w1x8 = 166164;
static const int w1y8 = 125214;
static const int w2x8 = 155572;
static const int w2y8 = 130541;
static const int w3x8 = 143956;
static const int w3y8 = 139016;
static const int wfx11 = 155572;
static const int wfy11 = 130541;
static const int wrx11 = 122077;
static const int wry11 = 174345;

const controller_info_t *default_controller;

////////

static void dumb_update()
{
	int wheel_command[4];
	if (base2008) { //this is because integer math is faster than simply handling doubles
		wheel_command[0] = (-cmd_body_x * w0x8 / 100000 - cmd_body_y * w0y8 / 100000) * 100000 / (w0x8 + w0y8) + cmd_body_w;
		wheel_command[1] = (-cmd_body_x * w1x8 / 100000 + cmd_body_y * w1y8 / 100000) * 100000 / (w1x8 + w1y8) + cmd_body_w;
		wheel_command[2] = (cmd_body_x * w2x8 / 100000 + cmd_body_y * w2y8 / 100000) * 100000 / (w2x8 + w2y8) + cmd_body_w;
		wheel_command[3] = (cmd_body_x * w3x8 / 100000 - cmd_body_y * w3y8 / 100000) * 100000 / (w3x8 + w3y8) + cmd_body_w;
	} else {
		wheel_command[0] = (-cmd_body_x * wrx11 / 100000 - cmd_body_y * wry11 / 100000) * 100000 / (wrx11 + wry11) + cmd_body_w;
		wheel_command[1] = (-cmd_body_x * wfx11 / 100000 + cmd_body_y * wfy11 / 100000) * 100000 / (wfx11 + wfy11) + cmd_body_w;
		wheel_command[2] = (cmd_body_x * wfx11 / 100000 + cmd_body_y * wfy11 / 100000) * 100000 / (wfx11 + wfy11) + cmd_body_w;
		wheel_command[3] = (cmd_body_x * wrx11 / 100000 - cmd_body_y * wry11 / 100000) * 100000 / (wrx11 + wry11) + cmd_body_w;
	}

	for (int i = 0; i < 5; ++i)
	{
		drive_mode[i] = DRIVE_SLOW_DECAY;
	}

	for (int i = 0; i < 4; ++i)
	{
		//FIXME - Pretend to drive the right speed
		int new_out = wheel_command[i];
		if (new_out > MOTOR_MAX)
		{
			new_out = MOTOR_MAX;
		} else if (new_out < -MOTOR_MAX)
		{
			new_out = -MOTOR_MAX;
		}

		int delta = new_out - last_out[i];
		if (delta > Command_Rate_Limit)
		{
			delta = Command_Rate_Limit;
		} else if (delta < -Command_Rate_Limit)
		{
			delta = -Command_Rate_Limit;
		}
		motor_out[i] = last_out[i] + delta;
		last_out[i] = motor_out[i];
	}
	motor_out[4] = dribble_command >> 1;
}

////////

static int step_motor;
static int step_level = 0;
static int step_threshold = 0;
static int step_holdoff = 0;
static int step_last_time = 0;

static void step_debug()
{
	int supply_mv = supply_raw * VBATT_NUM / VBATT_DIV;
	int dtime = current_time - step_last_time;
	step_last_time = current_time;
	printf("%2d.%03d %4d %3d %3d %2d", supply_mv / 1000, supply_mv % 1000, encoder_delta[step_motor], hall_delta[step_motor], motor_out[step_motor], dtime);
}

static void step_init(int argc, const char *argv[])
{
	if (argc >= 2)
	{
		step_motor = parse_int(argv[0]);
		step_level = parse_int(argv[1]);
	}

	if (argc >= 3)
	{
		step_threshold = parse_int(argv[2]);
	} else {
		step_threshold = 0;
	}

	if (argc < 2 || step_motor < 0 || step_motor > 4 || step_level < -MOTOR_MAX || step_level > MOTOR_MAX)
	{
		printf("Usage: run step <motor 0..3> <level> [<low threshold>]\n");
		controller = 0;
	}

	step_holdoff = 0;
	step_last_time = current_time;
	debug_update = step_debug;
}

static void step_update()
{
	int failed = (encoder_faults | motor_stall) & (1 << step_motor);
	if (failed)
	{
		printf(" *");
	}
	printf("\n");

	if (step_holdoff < 100)
	{
		++step_holdoff;
	} else {
		if (encoder_delta[step_motor] < step_threshold)
		{
			printf("Below %d\n", step_threshold);
			step_level = 0;
		}
	}

	if (failed && encoder_delta[step_motor] == 0 && hall_delta[step_motor] == 0)
	{
		// Motor died
		printf("Motor failed\n");
		controller = 0;
	}

	motor_out[step_motor] = step_level;
	drive_mode[step_motor] = DRIVE_SLOW_DECAY;
}

////////

static int kpid = 0;
static int kp2d = 0;
static int kd = 0;
static int error1[4], error2[4];
static int pd_debug = -1;

static void pid_init(int argc, const char *argv[])
{
	for (int i = 0; i < 4; ++i)
	{
		last_out[i] = 0;
	}

	// First three parameters are coefficients
	int kp = 160;
	int ki = 160;
	if (argc >= 3)
	{
		kp = parse_int(argv[0]);
		ki = parse_int(argv[1]);
		kd = parse_int(argv[2]);
	}

	// Find coefficients for simplified differential form
	kpid = kp + ki + kd;
	kp2d = kp + 2 * kd;

	// Next parameter is which motor to print
	if (argc >= 4)
	{
		pd_debug = parse_int(argv[3]);
	} else {
		pd_debug = -1;
	}
}

static void pid_update()
{
	if (base2008)
	{
		// Nope
		for (int i = 0; i < 5; ++i)
		{
			drive_mode[i] = DRIVE_OFF;
		}
		return;
	}

	for (int i = 0; i < 5; ++i)
	{
		drive_mode[i] = DRIVE_SLOW_DECAY;
	}

	//	FIXME: this assumes wheels are all at 45degrees
	int wheel_command[4] =
	{
		-cmd_body_x - cmd_body_y + cmd_body_w,
		-cmd_body_x + cmd_body_y + cmd_body_w,
		cmd_body_x + cmd_body_y + cmd_body_w,
		cmd_body_x - cmd_body_y + cmd_body_w
	};

	const int Scale = 256;
	const int Max_Command = MOTOR_MAX * Scale;

	for (int i = 0; i < 4; ++i)
	{
		int speed = encoder_delta[i];
		int error = wheel_command[i] - speed;

		if (abs(error) <= 1)
		{
				error = 0;
		}

		// Differential form: find the derivative of the controller's output:
		//  delta = kp * delta_error + ki * error + kd * (delta_error - last_delta[i]);
		// but simplified so that only the error history is stored.
		// Limit it (to prevent excessive motor current), integrate it,
		// and then limit the final output.
		// This results in a natural and correct limit on the error integral
		// and is simpler to implement than the more direct form:
		//    out = kp * error + ki * error_integral[i] + kd * delta_error
		int delta = kpid * error - kp2d * error1[i] + kd * error2[i];

		// Limit the change between consecutive cycles to prevent excessive current
		if (delta > Command_Rate_Limit * Scale)
		{
			delta = Command_Rate_Limit * Scale;
		} else if (delta < -Command_Rate_Limit * Scale)
		{
			delta = -Command_Rate_Limit * Scale;
		}
		last_out[i] += delta;

		// Don't accumulate output for broken motors, in case they start working
		if ((motor_stall) & (1 << i))
		{
			last_out[i] = 0;
		}

		// Clip to output limits
		if (last_out[i] > Max_Command)
		{
			last_out[i] = Max_Command;
		} else if (last_out[i] < -Max_Command)
		{
			last_out[i] = -Max_Command;
		}

		if (i == pd_debug)
		{
			printf("%04x %04x -> %08x %08x %08x -> %08x\n", wheel_command[i], speed, error, error1[i], error2[i], last_out[i]);
		}

		// Shift error history
		error2[i] = error1[i];
		error1[i] = error;

		// Convert the fixed point command to the final motor command
		motor_out[i] = (last_out[i] + Scale / 2) / Scale;
	}

	//FIXME - Do we need speed control on the dribbler?
	motor_out[4] = -dribble_command;
}

////////

static int tg_kp = 1;
static int tg_kd = 5;
static int tg_out = 0;
static int tg_last_error = 0;

static void test_gyro_init(int argc, const char *argv[])
{
	// First two parameters are coefficients
	if (argc != 2)
	{
		printf("Usage: run test_gyro <kp> <kd>\n");
		controller = 0;
	}

	tg_kp = parse_int(argv[0]);
	tg_kd = parse_int(argv[1]);
	tg_out = 0;
	tg_last_error = 0;
}

static void test_gyro_update()
{
	for (int i = 0; i < 4; ++i)
	{
		drive_mode[i] = DRIVE_SLOW_DECAY;
	}

	long gyro[3] = {0};
	if (imu_aligned)
	{
		IMUgetGyro(gyro);
	}

	int setpoint = 0;//-wheel_command[i] * 3;
	int speed = gyro[2] >> 12;
	int error = setpoint - speed;

	if (abs(error) <= 1)
	{
		error = 0;
	}

	int delta_error = error - tg_last_error;
	tg_last_error = error;
	int delta = error * tg_kp + delta_error * tg_kd;

	// Limit the change between consecutive cycles to prevent excessive current
	if (delta > Command_Rate_Limit * 256)
	{
		delta = Command_Rate_Limit * 256;
	} else if (delta < -Command_Rate_Limit * 256)
	{
		delta = -Command_Rate_Limit * 256;
	}
	tg_out += delta;

	// Don't accumulate output for broken motors, in case they start working
	if ((motor_stall) || kick_command == 0)
	{
		tg_out = 0;
	}

	// Clip to output limits
	static const int MAX = 40;
	if (tg_out > MAX * 256)
	{
		tg_out = MAX * 256;
	} else if (tg_out < -MAX * 256)
	{
		tg_out = -MAX * 256;
	}

// 	printf("%5d %4d\n", speed, tg_out);
	for (int i = 0; i < 4; ++i)
	{
		motor_out[i] = tg_out / 256;
	}
}


////////

const controller_info_t controllers[] =
{
	{"dumb", 0, 0, dumb_update},
	{"pid", pid_init, 0, pid_update},
	{"step", step_init, 0, step_update},
	{"test_gyro", test_gyro_init, 0, test_gyro_update},

	// End of table
	{0, 0}
};

const controller_info_t *controller = 0;
