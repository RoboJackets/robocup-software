// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

#include <QMutex>
#include <QPainter>

#include <framework/SystemState.hpp>
#include <framework/Dynamics.hpp>

#include "Pid.hpp"
#include <Utils.hpp>

class Configuration;

namespace Motion
{
	class RobotController
	{

		public:
			RobotController(Configuration *config, SystemState *state, int shell);
			~RobotController();

			/** Process the command for a robot and prepare output */
			void proc();

			void setAngKp(double value);
			void setAngKi(double value);
			void setAngKd(double value);

			OurRobot* self() { return _self; }

			typedef boost::shared_ptr<RobotController> shared_ptr;

		private:
			Pid _anglePid;

			/**
			 * generate velocities based on path travel
			 * @param endBehavior determines how to end the path
			 */
			void genVelocity(MotionCmd::PathEndType ending);

			/**
			 * safety net against hitting other robots
			 * if the commanded velocity will cause us to hit something
			 * in N frames...we need to subtract from commanded velocity
			 */
			void sanityCheck(const unsigned int N);

			/** scales the velocity to a smaller value if commands require */
			void scaleVelocity();

			/** stop the robot by forcing velocities to zero */
			void stop(float dtime); // time in seconds for the framerate

			/** generate motor speeds */
			void genMotor();

			/** overall state **/
			SystemState* _state;

			/** state info for self */
			OurRobot* _self;

			QMutex _procMutex;

			/** planner flag - copied out for rendering */
			MotionCmd::PlannerType _plannerType;

			/** robot dynamics information */
			Planning::Dynamics _dynamics;

			uint64_t _lastTimestamp;

			/** commanded velocities */
			Geometry2d::Point _vel; /// translational velocity
			float _w; /// rotational velocity

			/** convenience functions for robot state */
			inline Geometry2d::Point pos() const { return _self->pos; }
			inline Geometry2d::Point vel() const { return _self->vel; }
			inline float angle() const { return _self->angle; }
			inline float angleVel() const { return _self->angleVel; }

			/** filters for output */
			Utils::FIRFilter<Geometry2d::Point> _velFilter;
			Utils::FIRFilter<double> _wFilter;
	};
}
