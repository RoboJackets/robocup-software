// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

#include <QMutex>
#include <QPainter>

#include <framework/ConfigFile.hpp>
#include <framework/SystemState.hpp>

#include "Pid.hpp"

#include "Dynamics.hpp"

#include "planning/rrt.hpp"

namespace Motion
{
	class Robot
	{
		private:
			/** information pertaining to a single robot axle */
			typedef struct Axle
			{
				Axle(const Geometry2d::Point axel = Geometry2d::Point())
				{
					motor = 0;
					lastWheelVel = 0;
					wheel = axel.perpCCW();
				}

				//motor value
				float motor;

				//weeel velocities in the last frame
				int8_t lastWheelVel;

				//axle vector
				Geometry2d::Point axle;
				Geometry2d::Point wheel;
			} Axle;

		public:
			Robot(const ConfigFile::MotionModule::Robot& cfg, unsigned int id);
			~Robot();

			void setSystemState(SystemState* state);

			/** Process the command for a robot and prepare output */
			void proc();

			void setAngKp(double value);
			void setAngKi(double value);
			void setAngKd(double value);

			SystemState::Robot* self() { return _self; }

			typedef boost::shared_ptr<Robot> shared_ptr;

		private:
			Pid _anglePid;

			/**
			 * generate velocities based on path travel
			 * @param endBehavior determines how to end the path
			 */
			void genVelocity(MotionCmd::PathEndType ending);

			/** generate velocities based on time-position control */
			void genTimePosVelocity();

			/** generate velocities based on bezier paths */
			void genBezierVelocity();

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
			void genMotorOld();

			/** calibration function
			 * only runs when compile flags are set
			 * FIXME: find a way to parameterize this properly
			 */
			void calib();

		private:
			/** flag determining motor speed generation */
			static const bool _useOldMotorGen = false;

			/** robot identification */
			const unsigned int _id;
			
			/** overall state **/
			SystemState* _state;

			/** state info for self */
			SystemState::Robot* _self;

			/** robot axles */
			QVector<Robot::Axle> _axles;
			QMutex _procMutex;

			/** planner flag - copied out for rendering */
			MotionCmd::PlannerType _plannerType;

			RRT::Planner _planner;
			/** robot dynamics information */
			Dynamics _dynamics;

			uint64_t _lastTimestamp;

			/** commanded velocities */
			Geometry2d::Point _vel; /// translational velocity
			float _w; /// rotational velocity

			/** convenience functions for robot state */
			inline Geometry2d::Point pos() const { return _self->pos; }
			inline Geometry2d::Point vel() const { return _self->vel; }
			inline float angle() const { return _self->angle; }
			inline float angleVel() const { return _self->angleVel; }

			/** convenience functions for ball state */
			Geometry2d::Point ballPos() const { return _state->ball.pos; }
			Geometry2d::Point ballVel() const { return _state->ball.vel; }

			/** latest path */
			Planning::Path _path;

			/// calibration things ///
			typedef enum
			{
				InitCalib,
				InitialPoint,
				Wait1,
				Travel,
				Decel,
				End
			} CalibrationStates;
			
			class CalibInfo
			{
				public:
					CalibInfo()
					{
						startTime = 0;
						pSum = 0;
						speed = 0;
						outSpeed = 0;
					}
					
					uint64_t startTime;
					float pSum;
					Geometry2d::Point lastPos;
					
					int speed;
					int8_t outSpeed;
					
					Geometry2d::Point startPos;
					Geometry2d::Point endPos;
			};
			
			CalibrationStates _calibState;
			CalibInfo _calibInfo;

			/** filters for output */
			Utils::FIRFilter<Geometry2d::Point> _velFilter;
			Utils::FIRFilter<double> _wFilter;

			/// store parameters for bezier curves for rendering
			std::vector<Geometry2d::Point> _bezierControls;
			float _bezierTotalLength; /// total length of current bezier curve
			float _bezierCurTime;     /// current time increment for curve progression

			/// evaluates a Bezier curve
			Geometry2d::Point evaluateBezier(float t,
					const std::vector<Geometry2d::Point>& controls,
					const std::vector<float>& coeffs) const;

			/// evaluates the derivative of a Bezier curve
			Geometry2d::Point
			evaluateBezierVelocity(float t,
					const std::vector<Geometry2d::Point>& controls,
					const std::vector<float>& coeffs) const;

			/// determines the length of a Bezier curve
			float bezierLength(const std::vector<Geometry2d::Point>& controls,
					const std::vector<float>& coeffs) const;

			/// calculates binomial coefficients
			int binomialCoefficient(int n, int k) const;

			int factorial(int n) const;
	};
}
