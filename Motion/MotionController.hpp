#ifndef MOTIONCONTROLLER_HPP_
#define MOTIONCONTROLLER_HPP_

#include <QThread>
#include <QTimer>
#include <QMutex>
#include <QWaitCondition>
#include <QMutexLocker>

#include <Packet/IO.hpp>
#include <Packet/VisionData.hpp>
#include <Packet/MotionCmd.hpp>
#include <Packet/Ref.hpp>
#include <Packet/LogMotion.hpp>
#include <Packet/CommData.hpp>

#include <Geometry/Point2d.hpp>

#include "ConfigFile.hpp"
#include "InputHandler.hpp"
#include "Robot.hpp"

/** NEVER make more than ONE motion controller in a single process!
 *  It isn't thread safe nor does it make any sense... */
class MotionController : public QThread
{
	Q_OBJECT;
	
	private:
		typedef enum
		{
			Manual = 0,
			Auto
		} ControlMode;
		
		typedef enum
		{
			Running = 0,
			Stopped
		} RunMode;
	
	public:
		/** if autoOn is true, motion will go into auto mode by default when 
		 * there is a controller, otherwise it does manual default */
		MotionController(Team t, ConfigFile& cfg, bool autoOn = false);
		~MotionController();
		
	protected Q_SLOTS:
		void playPause();
		void autoMan();
		
		void visionDataHandler(const Packet::VisionData* data);
		void motionCmdHandler(const Packet::MotionCmd* data);
		void refHandler(const Packet::Ref* data);
		
	protected:
		void run();
		
		/** process ref data.
		 *  Setup the path planner accordingly */
		void procRef();
		
	private:
		/** our current team */
		Team _team;
		
		//run and control states for motion
		ControlMode _controlState;
		RunMode _runState;
		
		/** Input device handler (Logitech) */
		InputHandler* _input;
		
		/** outgoing Log and comm data */
		Packet::LogMotion _logData;
		Packet::CommData _commData;
		
		/** thread state */
		bool _running;
		
		/** robot info */
		Robot* _robots[5];
		
		//Packet::PacketReceiver _packRecv;
		Packet::VisionData _visionData; //vision infomation
		Packet::MotionCmd _commands; //commands for robots
		Packet::Ref _ref;
		
		//set to true when there is new vision data
		bool _newVision;
		
		/** configuration file */
		ConfigFile& _cfg;
};

#endif /*MOTIONCONTROLLER_HPP_*/
