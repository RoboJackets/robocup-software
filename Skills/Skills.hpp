#ifndef SKILLS_HPP_
#define SKILLS_HPP_

#include <QThread>

#include <Team.h>
#include <Geometry/Point2d.hpp>

#include <Packet/PacketReceiver.hpp>
#include <Packet/PacketSender.hpp>

#include <Packet/VisionData.hpp>
#include <Packet/SkillCmd.hpp>
#include <Packet/MotionCmd.hpp>
#include <Packet/SkillStatus.hpp>
#include <Packet/RobotStatus.hpp>

#include "Robot.hpp"

/** skills handler for a single team */
class Skills : public QThread
{
	/// methods ///
	public:
		Skills(Team t);
		~Skills();
		
	private:
		void visionHandler(const Packet::VisionData* data);
		void skillCmdHandler(const Packet::SkillCmd* data);
		void statusHandler(const Packet::RobotStatus* data);
		
		void procBaton();
		
		void run();
		
	/// members ///
	private:
		Team _team;
		bool _running;
		
		bool _newVisionData;
		
		Packet::VisionData _vision;
		Packet::SkillCmd _skillCmd;
		Packet::RobotStatus _status;
		
		Robot* _robots[5];
		
		/** robot with the baton */
		Robot* _baton;
};

#endif /* SKILLS_HPP_ */
