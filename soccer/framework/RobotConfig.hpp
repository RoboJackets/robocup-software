#pragma once

#include <Configuration.hpp>

struct RobotConfig
{
	RobotConfig(Configuration *config, QString prefix);
	~RobotConfig();
	
	struct PID
	{
		PID(Configuration *config, QString prefix);
		
		ConfigDouble::shared_ptr p;
		ConfigDouble::shared_ptr i;
		ConfigDouble::shared_ptr d;
	};
	
	struct Dynamics
	{
		Dynamics(Configuration *config, QString prefix);
		
		ConfigDouble::shared_ptr velocity;
		ConfigDouble::shared_ptr acceleration;
		ConfigDouble::shared_ptr deceleration;
	};
	
	struct Kicker
	{
		Kicker(Configuration *config, QString prefix);
		
		ConfigDouble::shared_ptr m;
		ConfigDouble::shared_ptr b;
	};
	
	Dynamics trapTrans;
	Dynamics trapRot;
	PID translation;
	PID rotation;
	PID wheel;

	ConfigDouble::shared_ptr wheelAlpha;
	ConfigDouble::shared_ptr test;
	
	Kicker kicker;
};
