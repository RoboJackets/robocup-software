#ifndef SKILL_HPP_
#define SKILL_HPP_

#include <Packet/SkillCmd.hpp>
#include <Packet/SkillStatus.hpp>

class Robot;

class Skill
{
	/// methods ///
	public:
		Skill(Packet::SkillCmd::Skill type);
		virtual ~Skill();
		
		/** returns the type of the skill */
		Packet::SkillCmd::Skill type() const { return _type; }
		
		/** returns the status of the skill
		 *  Default is to return no status */
		virtual	Packet::SkillStatus::Status status() const { return _status; }
		
		void setCmd(Packet::SkillCmd::Robot cmd) { _cmd = cmd; }
		
		/** called when the skill is first started */
		virtual void start() {};
		
		/** called each loop iteration if the skill is active
		 *  Return the motion command output from the skill */
		virtual Packet::MotionCmd::Robot run() = 0;
		
		/** get the robot for the skill */
		Robot* robot() const { return _robot; }
		/** set the robot for the skill */
		void robot(Robot* robot) { _robot = robot; }
		
	/// members ///
	protected:
		/** skill status holder */
		Packet::SkillStatus::Status _status;
		
		/** the skill command */
		Packet::SkillCmd::Robot _cmd;
		
	private:
		/** type of the skill */
		const Packet::SkillCmd::Skill _type;
		
		Robot* _robot;
};

#endif /* SKILL_HPP_ */
