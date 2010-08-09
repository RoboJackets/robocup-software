#include "RobotConfig.hpp"

#include <Configuration.hpp>

RobotConfig::PID::PID(Configuration* config, QString prefix):
	p(config, QString("%1/p").arg(prefix)),
	i(config, QString("%1/i").arg(prefix)),
	d(config, QString("%1/d").arg(prefix))
{
}

RobotConfig::Dynamics::Dynamics(Configuration* config, QString prefix):
	velocity(config, QString("%1/velocity").arg(prefix)),
	acceleration(config, QString("%1/acceleration").arg(prefix)),
	deceleration(config, QString("%1/deceleration").arg(prefix))
{
}

RobotConfig::Motion::Motion(Configuration* config, QString prefix):
	deg0(config, QString("%1/deg0").arg(prefix)),
	deg45(config, QString("%1/deg45").arg(prefix)),
	rotation(config, QString("%1/rotation").arg(prefix)),
	angle(config, QString("%1/angle").arg(prefix)),
	output_coeffs(config, QString("%1/output_coeffs").arg(prefix))
{
}

RobotConfig::Axle::Axle(Configuration* config, QString prefix):
	x(config, QString("%1/x").arg(prefix)),
	y(config, QString("%1/y").arg(prefix))
{
}

RobotConfig::Kicker::Kicker(Configuration* config, QString prefix):
	m(config, QString("%1/m").arg(prefix), 1),
	b(config, QString("%1/b").arg(prefix))
{
}

RobotConfig::RobotConfig(Configuration* config, QString prefix):
	motion(config, QString("%1/motion").arg(prefix)),
	kicker(config, QString("%1/kicker").arg(prefix))
{
	for (int i = 0; i < 4; ++i)
	{
		axles[i] = new Axle(config, QString("%1/axle/%2").arg(prefix, QString::number(i)));
	}
}

RobotConfig::~RobotConfig()
{
	for (int i = 0; i < 4; ++i)
	{
		delete axles[i];
	}
}
