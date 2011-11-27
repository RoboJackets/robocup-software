#include "RobotConfig.hpp"

#include <Configuration.hpp>

///////    RobotConfig

RobotConfig::PID::PID(Configuration* config, QString prefix):
	p(new ConfigDouble(config, QString("%1/p").arg(prefix))),
	i(new ConfigDouble(config, QString("%1/i").arg(prefix))),
	d(new ConfigDouble(config, QString("%1/d").arg(prefix)))
{
}

RobotConfig::Dynamics::Dynamics(Configuration* config, QString prefix):
	velocity(new ConfigDouble(config, QString("%1/velocity").arg(prefix))),
	acceleration(new ConfigDouble(config, QString("%1/acceleration").arg(prefix))),
	predictTime(new ConfigDouble(config, QString("%1/predictTime").arg(prefix))),
	responseTime(new ConfigDouble(config, QString("%1/responseTime").arg(prefix)))
{
}

RobotConfig::Kicker::Kicker(Configuration* config, QString prefix):
	m(new ConfigDouble(config, QString("%1/m").arg(prefix), 1)),
	b(new ConfigDouble(config, QString("%1/b").arg(prefix)))
{
}

RobotConfig::RobotConfig(Configuration* config, QString prefix):
	trapTrans(config, QString("%1/trapTrans").arg(prefix)),
	trapRot(config, QString("%1/trapRot").arg(prefix)),
	translation(config, QString("%1/translation").arg(prefix)),
	rotation(config, QString("%1/rotation").arg(prefix)),
	wheel(config, QString("%1/wheel").arg(prefix)),
	kicker(config, QString("%1/kicker").arg(prefix))
{
}

RobotConfig::~RobotConfig()
{
}

///////    RobotStatus

RobotStatus::RobotStatus(Configuration *config, QString prefix):
	chipper_enabled(new ConfigBool(config, QString("%1/Chipper Enabled").arg(prefix), true)),
	kicker_enabled(new ConfigBool(config, QString("%1/Kicker Enabled").arg(prefix), true)),
	ball_sense_enabled(new ConfigBool(config, QString("%1/Ball Sense Enabled").arg(prefix), true)),
	dribbler_enabled(new ConfigBool(config, QString("%1/Dribbler Enabled").arg(prefix), true))
{
}
