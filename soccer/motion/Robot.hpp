// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

#include <QMutex>
#include <QPainter>

#include <Team.h>
#include <framework/ConfigFile.hpp>
#include <framework/Module.hpp>

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

			/** slow processing for each robot */
			void slow();

			void drawPath(QPainter& p);
			void drawRRT(QPainter& p);

			void setPosKp(double value);
			void setPosKi(double value);
			void setPosKd(double value);

			void setAngKp(double value);
			void setAngKi(double value);
			void setAngKd(double value);

		private:
			Pid _posPid;
			Pid _anglePid;

			/** generate velocities based on path travel */
			void genVelocity();

			/** generate velocities based on time-position control */
			void genTimePosVelocity();

			/** stop the robot by forcing velocities to zero */
			void stop();

			/** generate motor speeds */
			void genMotor(bool old = false);
			void genMotorOld();

			/** calibration function
			 * only runs when compile flags are set
			 * FIXME: find a way to parameterize this properly
			 */
			void calib();

		private:
			/** robot identification */
			const unsigned int _id;
			
			/** overall state **/
			SystemState* _state;

			/** state info for self */
			SystemState::Robot* _self;

			/** robot axles */
			QVector<Robot::Axle> _axles;
			QMutex _procMutex;

			RRT::Planner _planner;
			/** robot dynamics information */
			Dynamics _dynamics;

			uint64_t _lastTimestamp;

			/** commanded velocities */
			Geometry2d::Point _vel; /// translational velocity
			float _w; /// rotational velocity

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
	};
}
