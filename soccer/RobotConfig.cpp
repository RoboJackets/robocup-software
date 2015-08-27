#include "RobotConfig.hpp"

#include <Configuration.hpp>

///////    RobotConfig

RobotConfig::PID::PID(Configuration* config, QString prefix)
    : p(new ConfigDouble(config, QString("%1/p").arg(prefix))),
      i(new ConfigDouble(config, QString("%1/i").arg(prefix))),
      i_windup(new ConfigInt(config, QString("%1/i_windup").arg(prefix))),
      d(new ConfigDouble(config, QString("%1/d").arg(prefix))) {}

RobotConfig::Kicker::Kicker(Configuration* config, QString prefix)
    : maxKick(new ConfigDouble(config, QString("%1/maxKick").arg(prefix), 255)),
      maxChip(
          new ConfigDouble(config, QString("%1/maxChip").arg(prefix), 150)) {}

RobotConfig::Dribbler::Dribbler(Configuration* config, QString prefix)
    : multiplier(new ConfigDouble(config, QString("%1/multiplier").arg(prefix),
                                  1.0)) {}

RobotConfig::Chipper::Chipper(Configuration* config, QString prefix)
    : calibrationSlope(new ConfigDouble(
          config, QString("%1/calibSlope").arg(prefix), 2.20227272727)),
      calibrationOffset(new ConfigDouble(
          config, QString("%1/calibOffset").arg(prefix), 70.0727272727)) {}

RobotConfig::RobotConfig(Configuration* config, QString prefix)
    : translation(config, QString("%1/translation").arg(prefix)),
      rotation(config, QString("%1/rotation").arg(prefix)),
      kicker(config, QString("%1/kicker").arg(prefix)),
      dribbler(config, QString("%1/dribbler").arg(prefix)),
      chipper(config, QString("%1/chipper").arg(prefix)),
      pivotVelMultiplier(new ConfigDouble(
          config, QString("%1/pivot/velMultiplier").arg(prefix))),
      velMultiplier(new ConfigDouble(
          config, QString("%1/translation/velMultiplier").arg(prefix))),
      angleVelMultiplier(new ConfigDouble(
          config, QString("%1/rotation/velMultiplier").arg(prefix))),
      accelerationMultiplier(new ConfigDouble(
          config, QString("%1/translation/accelMultiplier").arg(prefix))),
      minEffectiveVelocity(new ConfigDouble(
          config, QString("%1/translation/minEffectiveVelocity").arg(prefix),
          0.3)),
      minEffectiveAngularSpeed(new ConfigDouble(
          config, QString("%1/rotation/minEffectiveAngularSpeed").arg(prefix),
          0.7)) {}

RobotConfig::~RobotConfig() {}

///////    RobotStatus

RobotStatus::RobotStatus(Configuration* config, QString prefix)
    : chipper_enabled(new ConfigBool(
          config, QString("%1/Chipper Enabled").arg(prefix), true)),
      kicker_enabled(new ConfigBool(
          config, QString("%1/Kicker Enabled").arg(prefix), true)),
      ball_sense_enabled(new ConfigBool(
          config, QString("%1/Ball Sense Enabled").arg(prefix), true)),
      dribbler_enabled(new ConfigBool(
          config, QString("%1/Dribbler Enabled").arg(prefix), true)) {}
