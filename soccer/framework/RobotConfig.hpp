#pragma once

#include <Configuration.hpp>

struct RobotConfig
{
	RobotConfig(Configuration *config, QString prefix);
	~RobotConfig();
	
	struct PID
	{
		PID(Configuration *config, QString prefix);
		
		ConfigDouble p;
		ConfigDouble i;
		ConfigDouble d;
	};
	
	struct Dynamics
	{
		Dynamics(Configuration *config, QString prefix);
		
		ConfigDouble velocity;
		ConfigDouble acceleration;
		ConfigDouble deceleration;
	};
	
	struct Motion
	{
		Motion(Configuration *config, QString prefix);
		
		Dynamics deg0;
		Dynamics deg45;
		Dynamics rotation;
		
		PID angle;
		
		ConfigFloatVector output_coeffs;
		//FIXME - In config
// 		std::vector<float> output_coeffs;
	};
	
	struct Axle
	{
		Axle(Configuration *config, QString prefix);
		ConfigDouble x, y;
	};
	
	struct Kicker
	{
		Kicker(Configuration *config, QString prefix);
		
		ConfigDouble m;
		ConfigDouble b;
	};
	
	Motion motion;
	Kicker kicker;
	Axle *axles[4];
};
