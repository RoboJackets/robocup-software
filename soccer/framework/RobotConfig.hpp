#pragma once

#include <Configuration.hpp>

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
