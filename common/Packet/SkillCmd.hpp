#ifndef SKILLCMD_HPP_
#define SKILLCMD_HPP_

#include "Ports.hpp"
#include "MotionCmd.hpp"

namespace Packet
{
	/** a skill cmd is a small wrapper around the motion packet
	 *  It adds extra members for performing specified skills. 
	 *  Depending on the skill, members of MotionCmd will be used/ignored */
	class SkillCmd
	{
		public:
			static const int Type = SkillCmdPort;

			typedef enum
			{
				/** no ball skills, avoid ball */
				None,
				
				/** move to a position with the ball */
				Dribble,
				
				/** face a position with the ball */
				Orient,

				/** ignores position and tries to aquire the ball */
				GotoBall,
				
				/** attempts a kick on goal */
				Kick,
				
				/** attempts a pass to a point */
				Pass,
				
				/** steal the ball from a robot */
				StealBall,
                
                /** receive a pass */
                Receive,
				
				/** Mark a player */
				Mark
			} Skill;

			class Robot
			{
				public:
					Robot() : valid(false), sequence(-1) {};
					
					/** if the skill is valid for the robot
					 *  valid in motion does NOT need to be set as well */
					bool valid;

					/** skill to perform if any */
					Skill skill;
					
					/** position to pass the ball to */
					Geometry::Point2d passPos;

					/** motion command for the robot */
					Packet::MotionCmd::Robot motion;
					
					/** id of the robot to mark, if skill == mark */
					unsigned int markID;
                    
                    int sequence;

			} __attribute__((__packed__));

			Robot robots[5];

	} __attribute__((__packed__));
}

#endif /* SKILLCMD_HPP_ */
