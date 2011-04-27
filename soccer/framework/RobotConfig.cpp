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

RobotConfig::Kicker::Kicker(Configuration* config, QString prefix):
	m(config, QString("%1/m").arg(prefix), 1),
	b(config, QString("%1/b").arg(prefix))
{
}

RobotConfig::RobotConfig(Configuration* config, QString prefix):
	trapTrans(config, QString("%1/trapTrans").arg(prefix)),
	trapRot(config, QString("%1/trapRot").arg(prefix)),
	translation(config, QString("%1/translation").arg(prefix)),
	rotation(config, QString("%1/rotation").arg(prefix)),
	wheel(config, QString("%1/wheel").arg(prefix)),
	wheelAlpha(config, QString("%1/wheelAlpha").arg(prefix), 1),
	test(config, QString("%1/test").arg(prefix)),
	kicker(config, QString("%1/kicker").arg(prefix))
{
}

RobotConfig::~RobotConfig()
{
}
