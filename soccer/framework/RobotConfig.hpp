#pragma once

#include <Configuration.hpp>

/**
 * Configuration per robot model
 * includes movement, pid, kicker and status
 */
struct RobotConfig
{
	RobotConfig(Configuration *config, QString prefix);
	~RobotConfig();
	
	struct PID
	{
		PID(Configuration *config, QString prefix);
		
		ConfigDouble *p;
		ConfigDouble *i;
		ConfigDouble *d;
	};
	
	struct Dynamics
	{
		Dynamics(Configuration *config, QString prefix);
		
		ConfigDouble *velocity;
		ConfigDouble *acceleration;
// 		ConfigDouble *deceleration;
		ConfigDouble *predictTime;
		ConfigDouble *responseTime;
	};
	
	struct Kicker
	{
		Kicker(Configuration *config, QString prefix);
		
		ConfigDouble *m;
		ConfigDouble *b;
	};
	
	Dynamics trapTrans;
	Dynamics trapRot;
	PID translation;
	PID rotation;
	PID wheel;

	Kicker kicker;
};


/**
 * Provides per-robot overrides for a robot
 * Should be updated for hardware revision
 */
struct RobotStatus
{
	RobotStatus(Configuration *config, QString prefix);
	~RobotStatus() {}

	ConfigBool *chipper_enabled;
	ConfigBool *kicker_enabled;
	ConfigBool *ball_sense_enabled;
	ConfigBool *dribbler_enabled;
};
