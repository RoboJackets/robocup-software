#ifndef ROBOT_HPP_
#define ROBOT_HPP_

//#include <Packet/SkillCmd.hpp>
//#include <Packet/SkillStatus.hpp>
#include <Packet/VisionData.hpp>
#include <Packet/RobotStatus.hpp>

#include <Geometry/Point2d.hpp>

#include "Skill.hpp"

class Robot
{		
	/// methods ///
	public:
		Robot(unsigned int id, Packet::VisionData& vd, Packet::RobotStatus::Robot& status);
		~Robot() {};
		
		/** return the id of the robot */
		unsigned int id() const { return _id; }
		
		/** process a skill command ... assumed to be valid
		 *  @return a robot motion command from the skill */
		Packet::MotionCmd::Robot proc(Packet::SkillCmd::Robot skillCmd);
		
		Packet::RobotStatus::Robot status() const { return _status; }
		
		Packet::SkillStatus::Status skillStatus() const;
		
		const Packet::VisionData& vision() const { return _vision; }
		const Packet::VisionData::Robot& self() const { return _self; }
		
		/** returns the robot id for the robot to enable reverse channel for */
		//static Robot const* baton() { return _baton; }
		
		/** request a baton, if available it will be given */
		void requestBaton() { _needBaton = true; }
		
		/** releases the baton from this robot, if it has it */
		void releaseBaton() { _needBaton = false; }
		
		/** returns if the robot needs the baton */
		bool needBaton() const { return _needBaton; }  
		
	/// members ///
	private:
		/** current active skill */
		Skill* _skill;
		
		/** robot id */
		unsigned int _id;
		
		Packet::VisionData& _vision;
		Packet::VisionData::Robot& _self;		
		Packet::RobotStatus::Robot& _status;
		
		bool _needBaton;
};

#endif /* ROBOT_HPP_ */
