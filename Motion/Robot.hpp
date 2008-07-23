#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include <Packet/MotionCmd.hpp>
#include <Packet/VisionData.hpp>
#include <Packet/CommData.hpp>
#include <Packet/LogMotion.hpp>

#include "ConfigFile.hpp"
#include "Pid.hpp"
#include "PathPlanner.hpp"

class Robot
{
	private:
		/** robot velocity control information */
		typedef struct VelocityCmd
		{
			VelocityCmd() : w(0), maxWheelSpeed(127) {}
			
			/** velocity along x,y in team space */
			Geometry::Point2d vel;
			
			/** rotational velocity */
			float w;
			
			uint8_t maxWheelSpeed;
		} VelocityCmd;
	
	public:
		Robot(ConfigFile::RobotCfg cfg, 
				Packet::VisionData& vd, 
				Packet::CommData::Robot& cd,
				Packet::LogMotion::Robot& ld);
		~Robot();
		
		Packet::VisionData::Robot& self() const { return _self; }
		
		/** Process the command for a robot and prepare output */
		void proc(const Packet::MotionCmd::Robot& cmd);
		
		/** clear PID windup */
		void clearPid();
		
		PathPlanner* pathPlanner() const { return _pathPlanner; }
		
		template <typename T>
		void newPathPlanner()
		{
			_pathPlanner = new T(_vision);
		}
		
	private:
		/** given a created robot velocity, generate motor speeds */
		void genMotor(VelocityCmd velCmd);
		
		/** robot identification */
		const unsigned int _id;
		
		/** position pid */
		Pid* _posPID;
		/** angle pid */
		Pid* _anglePID;
		
		float* _motors;
		
		/** planner for this robot */
		PathPlanner* _pathPlanner;
		
		/** vision data for the robot */
		Packet::VisionData& _vision;
		/** vision data for the robot */
		Packet::VisionData::Robot& _self;
		/** comm data for the robot */
		Packet::CommData::Robot& _comm;
		/** logging information for robot */
		Packet::LogMotion::Robot& _log;
		
		/** robot axels */
		QVector<Geometry::Point2d> _axels;
};

#endif /* ROBOT_HPP_ */
