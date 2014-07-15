#pragma once
#include <gameplay/Behavior.hpp>

namespace Gameplay
{
	// FIXME: It's not a behavior...
	class ChipCalibration : public SingleRobotBehavior {
	public:
		ChipCalibration(GameplayModule* game);

		static void createConfiguration(Configuration *cfg);

		virtual bool run() { return false; } //< No purpose

		int chipPowerForDistance(double distance);

		double max_chip_distance() { return *_max_chip_distance; }
		double dribble_speed() { return *_dribble_speed; }

		bool roll_to_target; // toggles usage of roll distance

        static ConfigDouble *_a0, *_a1, *_a2, *_a3; // constants of the cubic
        static ConfigDouble *_max_chip_distance; // Maximum chip distance the robot is capable of
        static ConfigDouble *_dribble_speed; // dribble speed in order to map cubically
        static ConfigDouble *_roll_distance; // reduces chip distance to roll to target 
	};
}