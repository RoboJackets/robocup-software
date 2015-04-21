#include "ChipCalibration.hpp"

using namespace std;
using namespace Geometry2d;

namespace Gameplay { 
	REGISTER_CONFIGURABLE(ChipCalibration) 
}

ConfigDouble *Gameplay::ChipCalibration::_a0;
ConfigDouble *Gameplay::ChipCalibration::_a1;
ConfigDouble *Gameplay::ChipCalibration::_a2;
ConfigDouble *Gameplay::ChipCalibration::_a3;
ConfigDouble *Gameplay::ChipCalibration::_max_chip_distance;
ConfigDouble *Gameplay::ChipCalibration::_dribble_speed;
ConfigDouble *Gameplay::ChipCalibration::_roll_distance;

Gameplay::ChipCalibration::ChipCalibration(GameplayModule* game)
	: SingleRobotBehavior(game) {
}

void Gameplay::ChipCalibration::createConfiguration(Configuration *cfg)
{
    _a0 = new ConfigDouble(cfg, "ChipCalibration/_a0", -78.3635);
    _a1 = new ConfigDouble(cfg, "ChipCalibration/_a1", 3.30802);
    _a2 = new ConfigDouble(cfg, "ChipCalibration/_a2", -0.0238479);
    _a3 = new ConfigDouble(cfg, "ChipCalibration/_a3", 0.000613888);
    _dribble_speed = new ConfigDouble(cfg, "ChipCalibration/Dribble Speed", 40);
    _max_chip_distance = new ConfigDouble(cfg, "ChipCalibration/Max Chip Distance", 2.5);
    _roll_distance = new ConfigDouble(cfg, "ChipCalibration/Roll Distance", 2.5);
}

int Gameplay::ChipCalibration::chipPowerForDistance(double distance)
{
	if(roll_to_target) {
		if(distance > *_roll_distance)
			distance -= *_roll_distance;
	}

	if(distance >= (*_max_chip_distance))
		return 255;
	int x = 0;
	double dist = (*_a0);
	while(dist < distance && x <= 255)
	{
		x += 1;
		dist = (*_a0) + (*_a1) * x + (*_a2) * x * x + (*_a3) * x * x * x;
	}
	return x;
}
