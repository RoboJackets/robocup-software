#include "LuckyKick.hpp"
#include <gameplay/behaviors/PivotKick.hpp>
#include <Geometry2d/util.h>

//namespace Gameplay { namespace Behaviors { REGISTER_CONFIGURABLE(LuckyKick) } }
namespace Gameplay { REGISTER_CONFIGURABLE(LuckyKick); }

ConfigBool *Gameplay::LuckyKick::USE_CHANCE;
ConfigBool *Gameplay::LuckyKick::USE_KICK;
ConfigBool *Gameplay::LuckyKick::USE_PIVOT;
ConfigBool *Gameplay::LuckyKick::USE_CHIP;
ConfigBool *Gameplay::LuckyKick::USE_BALLSENSE;
ConfigBool *Gameplay::LuckyKick::USE_APPROACH;
ConfigBool *Gameplay::LuckyKick::DEBUG;

ConfigDouble *Gameplay::LuckyKick::APPROACH_ANGLE;
ConfigDouble *Gameplay::LuckyKick::APPROACH_DIST;

namespace Gameplay {
	void LuckyKick::createConfiguration(Configuration* cfg) {
		USE_CHANCE = new ConfigBool(cfg, "LuckyKick/Use Chance", true);
		USE_KICK = new ConfigBool(cfg, "LuckyKick/Use Kick", true);
		USE_PIVOT = new ConfigBool(cfg, "LuckyKick/Use Pivot", true);
		USE_CHIP = new ConfigBool(cfg, "LuckyKick/Use Chip", true);
		USE_BALLSENSE = new ConfigBool(cfg, "LuckyKick/Use Ballsense", true);
		USE_APPROACH = new ConfigBool(cfg, "LuckyKick/Use Approach", true);
		DEBUG = new ConfigBool(cfg, "LuckyKick/Debug", true);

		APPROACH_ANGLE = new ConfigDouble(cfg, "LuckyKick/Approach Angle", 40);
		APPROACH_DIST = new ConfigDouble(cfg, "LuckyKick/Approach Dist", 0.1);
	}
}

namespace Gameplay {

	Luck_Options::Luck_Options()
	: use_chance(-1),
	  use_kick(-1),
	  use_pivot(-1),
	  use_chip(-1),
	  debug(-1),
	  approach_angle(-1),
	  approach_dist(-1) {
	}

	LuckyKick::LuckyKick(GameplayModule* gameplay)
	: SingleRobotBehavior(gameplay),
	  pivot_kicker(gameplay),
	  window_eval(state()),
	  local() {
	}

	Luck_Status LuckyKick::get_lucky(OurRobot* robot) {
		Luck_Options opt = get_options();

		use_chance(robot, opt);
		use_kick(robot, opt);
		use_pivot(robot, opt);
		use_chip(robot, opt);
		use_ballsense(robot, opt);
		use_approach(robot, opt);

		open_kick(robot, opt);
		open_pivot(robot, opt);
		open_chip(robot, opt);

		if(robot)
			robot->addText("Getting Lucky", Qt::magenta, "LuckyKick");

		return NO_LUCK;
	}

	Luck_Status LuckyKick::get_lucky() {
		return get_lucky(robot);
	}

	bool LuckyKick::use_chance(OurRobot* robot, const Luck_Options& opt) {
		return robot && opt.use_chance > 0;
	}

	bool LuckyKick::has_chance(OurRobot* robot, const Luck_Options& opt) {
		return use_chance(robot, opt) && (use_ballsense(robot, opt) || use_approach(robot, opt));
	}

	bool LuckyKick::use_kick(OurRobot* robot, const Luck_Options& opt) {
		return has_chance(robot, opt) && opt.use_kick > 0;
	}

	bool LuckyKick::use_pivot(OurRobot *robot, const Luck_Options& opt) {
		return has_chance(robot, opt) && opt.use_pivot > 0;
	}

	bool LuckyKick::use_chip(OurRobot *robot, const Luck_Options& opt) {
		return has_chance(robot, opt) && robot->hardwareVersion() == Packet::RJ2011 && opt.use_chip;
	}

	bool LuckyKick::use_ballsense(OurRobot *robot, const Luck_Options& opt) {
		return opt.use_ballsense > 0 && robot->hasBall();
	}

	bool LuckyKick::use_approach(OurRobot *robot, const Luck_Options& opt) {
		if(!use_chance(robot, opt) && opt.use_approach > 0)
			return false;

		if(opt.approach_angle < 0 || opt.approach_dist < 0)
			return false;

		bool within_approach = false;
		Geometry2d::Line kick_line(robot->pos, robot->kickerBar().center());
		Geometry2d::Point p = kick_line.nearestPoint(ball().pos);
		double x = (robot->pos - p).mag();
		double y = (ball().pos - p).mag();
		double angle = atan2(y,x);
		double dist = (robot->pos - ball().pos).mag();

		if(dist <= Robot_Radius + opt.approach_dist && fabs(angle * RadiansToDegrees) <= opt.approach_angle)
			within_approach = true;

		if(opt.debug && within_approach) {
			state()->drawCircle(robot->pos, Robot_Radius + opt.approach_dist,
				                Qt::green, "LuckyKick");
			Geometry2d::Point bound = (Robot_Radius + opt.approach_dist) * kick_line.delta().normalized();
			state()->drawLine(robot->pos, robot->pos + bound.rotated( opt.approach_angle), 
				              Qt::green, "LuckyKick");
			state()->drawLine(robot->pos, robot->pos + bound.rotated(-opt.approach_angle), 
				              Qt::green, "LuckyKick");
		}

		return within_approach;
	}

	bool LuckyKick::open_kick(OurRobot *robot, const Luck_Options& opt) {
		if(!use_kick(robot, opt))
			return false;

		Geometry2d::Segment kick_ray;
		bool ok = get_kickray(robot, kick_ray);
		for (Robot* other :  get_others(robot)) {
			if(!other->visible)
				continue;
			Geometry2d::Circle obstacle(other->pos, Robot_Radius + Ball_Radius);
			bool open = !kick_ray.intersects(obstacle);
			ok &= open;

			if(opt.debug > 0 && !open) {
				state()->drawLine(other->pos, kick_ray.nearestPoint(other->pos),
					              Qt::red, "LuckyKick");
			}
		}
		if(opt.debug > 0) {
			state()->drawLine(kick_ray, ok ? Qt::green : Qt::red, "LuckyKick");
		}

		return ok && robot->hasBall();
	}

	bool LuckyKick::open_pivot(OurRobot *robot, const Luck_Options& opt) {
		if(!use_pivot(robot, opt))
			return false;

		window_eval.exclude.clear();
		window_eval.exclude.push_back(robot->pos);
		window_eval.run(robot->pos, get_goalline());

		bool open = false;
		for (Window* window :  window_eval.windows) {
			if(window->segment.length() > Ball_Radius) {
				open = true;
			}
		}

		return open;
	}

	bool LuckyKick::open_chip(OurRobot *robot, const Luck_Options& opt) {
		return false;
	}

	std::vector<Robot*> LuckyKick::get_others(Robot* robot) {
		std::vector<Robot*> others;
		std::vector<OpponentRobot*>& opp = state()->opp;
		std::vector<OurRobot*>& self = state()->self;
		for(int i=0; i < opp.size(); i++) {
			others.push_back(opp[i]);
		}
		for(int i=0; i < self.size(); i++) {
			if(robot->shell() == self[i]->shell())
				continue;
			others.push_back(self[i]);
		}
		return others;
	}

	Geometry2d::Segment LuckyKick::get_goalline() {
		return Geometry2d::Segment(Geometry2d::Point(-Field_GoalWidth/2 + Ball_Radius, Field_Length),
			                       Geometry2d::Point( Field_GoalWidth/2 - Ball_Radius, Field_Length));
	}

	bool LuckyKick::get_kickray(OurRobot* robot, Geometry2d::Segment& kickray) {
		Geometry2d::Segment goal_line = get_goalline();
		Geometry2d::Point kick_dir = (robot->kickerBar().center() - robot->pos).normalized();
		Geometry2d::Segment kick_line(robot->pos, robot->pos + Field_Length * kick_dir);
		Geometry2d::Point target;
		goal_line.Line::intersects(kick_line, &target);
		kickray = Geometry2d::Segment(robot->pos, target);
		return goal_line.intersects(kick_line, NULL);
	}

	Luck_Options LuckyKick::get_options() {
		Luck_Options opt;

		opt.use_chance = local.use_chance;
		if(opt.use_chance < 0)
			opt.use_chance = *USE_CHANCE;

		opt.use_kick = local.use_kick;
		if(opt.use_kick < 0)
			opt.use_kick = *USE_KICK;

		opt.use_pivot = local.use_pivot;
		if(opt.use_pivot < 0)
			opt.use_pivot = *USE_PIVOT;

		opt.use_chip = local.use_chip;
		if(opt.use_chip < 0)
			opt.use_chip = *USE_CHIP;

		opt.use_ballsense = local.use_ballsense;
		if(opt.use_ballsense < 0)
			opt.use_ballsense = *USE_BALLSENSE;

		opt.use_approach = local.use_approach;
		if(opt.use_approach < 0)
			opt.use_approach = *USE_APPROACH;

		opt.debug = local.debug;
		if(opt.debug < 0)
			opt.debug = *DEBUG;

		opt.approach_angle = local.approach_angle;
		if(opt.approach_angle < 0)
			opt.approach_angle = *APPROACH_ANGLE;

		opt.approach_dist = local.approach_dist;
		if(opt.approach_dist < 0)
			opt.approach_dist = *APPROACH_DIST;

		return opt;
	}
}