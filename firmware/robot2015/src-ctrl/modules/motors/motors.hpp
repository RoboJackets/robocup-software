#pragma once

#include "robot.hpp"
#include "commands.hpp"
#include "param.hpp"

/* Any math using the delta values for the hall/encoder (we really
 * only care about encoder readings...) is best handled through a task
 * queue where the calculations can be managed on the kernel's scheduler.
 * So we'll leave the MOTOR_MAX_SAMPLES at a minimum in the struct.
 */
#define MOTOR_MAX_SAMPLES 2

typedef uint16_t motorVel_t;

typedef struct {
	/*
	Available Flags:
	=================
	GVDD_OV
	GVDD_UV
	PVDD_UV
	OTSD
	OTW
	FETHA_OC
	FETLA_OC
	FETHB_OC
	FETLB_OC
	FETHC_OC
	FETLC_OC
	*/
	bool encOK;
	bool hallOK;
	std::array<uint16_t, 2> drvStatus;
} motorErr_t;

typedef struct {
	motorVel_t 	targetVel;
	motorVel_t	adjVel;
	uint16_t	hallCount;
	std::array<uint32_t, MOTOR_MAX_SAMPLES> encCounts;
	motorErr_t status;
	std::string desc;
} motor_t;

void motors_Init(void);
void motors_PrintMotor(motor_t &);
void motors_cmdProcess(const std::vector<std::string> &args);
