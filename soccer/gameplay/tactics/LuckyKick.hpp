#pragma once
#include <gameplay/Behavior.hpp>
#include <gameplay/behaviors/PivotKick.hpp>
#include <gameplay/evaluation/WindowEvaluator.hpp>
#include <gameplay/tactics/LuckyKick.hpp>

namespace Gameplay {

	enum Luck_Status {
		NO_LUCK = 0,
		GETTING_LUCKY,
		DONE,
	};

	struct Luck_Options {
		int use_chance;
		int use_kick;
		int use_pivot;
		int use_chip;
		int use_ballsense;
		int use_approach;
		int debug;

		double approach_angle;
		double approach_dist;

		Luck_Options();
	};

	class LuckyKick : public SingleRobotBehavior {
	public:
		static void createConfiguration(Configuration *cfg);

 	public:
		LuckyKick(GameplayModule* gameplay);
		virtual ~LuckyKick() {}

		// Don't use run
		virtual bool run() { return true; }

		Luck_Options get_options();

		bool use_chance(OurRobot* robot, const Luck_Options& opt);
		bool has_chance(OurRobot* robot, const Luck_Options& opt);
		bool use_kick(OurRobot* robot, const Luck_Options& opt);
		bool use_pivot(OurRobot* robot, const Luck_Options& opt);
		bool use_chip(OurRobot* robot, const Luck_Options& opt);
		bool use_ballsense(OurRobot* robot, const Luck_Options& opt);
		bool use_approach(OurRobot* robot, const Luck_Options& opt);

		bool open_kick(OurRobot* robot, const Luck_Options& opt);
		bool open_pivot(OurRobot* robot, const Luck_Options& opt);
		bool open_chip(OurRobot* robot, const Luck_Options& opt);

		Luck_Status get_lucky();
		Luck_Status get_lucky(OurRobot* robot);

		Geometry2d::Segment get_goalline();
		bool get_kickray(OurRobot* robot, Geometry2d::Segment& kickray);
		std::vector<Robot*> get_others(Robot* robot);

	protected:
		Behaviors::PivotKick pivot_kicker;
		WindowEvaluator window_eval;

	protected:
		Luck_Options local;

		static ConfigBool *USE_CHANCE;
		static ConfigBool *USE_KICK;
		static ConfigBool *USE_PIVOT;
		static ConfigBool *USE_CHIP;
		static ConfigBool *USE_BALLSENSE;
		static ConfigBool *USE_APPROACH;
		static ConfigBool *DEBUG;

		static ConfigDouble *APPROACH_ANGLE;
		static ConfigDouble *APPROACH_DIST;
	};
}