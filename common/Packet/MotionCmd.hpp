#ifndef MOTIONCMD_HPP_
#define MOTIONCMD_HPP_

#include "Ports.hpp"

#include <Geometry/Point2d.hpp>
#include <Geometry/Circle2d.hpp>
#include <stdint.h>

namespace Packet
{
	class MotionCmd
	{
		public:
			static const int Type = MotionCmdPort;

			typedef enum
			{
				Fast,
				Accurate
			} MotionStyle;
			
			MotionCmd()
			{
				baton = -1;
			}
			
			/** a goto command for the motion system */
			class Robot
			{
				public:
					Robot() :
						valid(false), style(Fast), avoid(false), spin(false),
						roller(false), kick(0), goalie(false)
					{}
					
					/** if true the motion command is valid for this robot */
					bool valid;
					
					/** the style of motion */
					MotionCmd::MotionStyle style;

					/** position to goto */
					Geometry::Point2d pos;
					/** point to face */
					Geometry::Point2d face;					
					
					/** if true, the robot will avoid the specified region */
					bool avoid;
					/** zone to avoid */
					Geometry::Circle2d avoidZone;
					
					/** if true the robot will spin in place */
					bool spin;
					
					/// other robot control ... not used by motion
					int8_t roller;
					uint8_t kick;
					
					/** if true, this robot is the goalie */
					bool goalie;
										
			} __attribute__((__packed__));

			/** vision parent for this motion command */
			uint64_t timestamp;

			/** max of 5 robots can be controlled at once */
			Robot robots[5];
			
			/** if id is in [0,4] then enable reverse channel for robot */
			int baton;
	} __attribute__((__packed__));
}

#endif /*MOTIONCMD_HPP_*/
