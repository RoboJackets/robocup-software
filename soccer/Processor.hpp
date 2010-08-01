// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

#include <QThread>
#include <QMutex>
#include <QMutexLocker>
#include <QUdpSocket>

#include <boost/shared_ptr.hpp>

#include <protobuf/LogFrame.pb.h>
#include <Network.hpp>
#include <Logger.hpp>
#include <Geometry2d/TransformMatrix.hpp>

#include <gameplay/GameplayModule.hpp>
#include <motion/MotionModule.hpp>
#include <stateID/StateIDModule.hpp>
#include <modeling/WorldModel.hpp>
#include <motion/PointController.hpp>
#include <motion/WheelController.hpp>

#include <framework/ConfigFile.hpp>

#include "RefereeModule.hpp"

class Joystick;

/** handles processing for a team */
class Processor: public QThread
{
	public:
		struct Status
		{
			Status()
			{
				lastLoopTime = 0;
				lastVisionTime = 0;
				lastRefereeTime = 0;
				lastRadioRxTime = 0;
			}
			
			uint64_t lastLoopTime;
			uint64_t lastVisionTime;
			uint64_t lastRefereeTime;
			uint64_t lastRadioRxTime;
		};
		
		Processor(QString filename, bool sim, int radio);
		~Processor();
		
		void stop();
		
		int radio() const
		{
			return _radio;
		}
		
		bool autonomous();
		bool joystickValid();
		
		void externalReferee(bool value)
		{
			_externalReferee = value;
		}
		
		bool externalReferee() const
		{
			return _externalReferee;
		}
		
		void manualID(int value);
		int manualID()
		{
			QMutexLocker locker(&_loopMutex);
			return _manualID;
		}
		
		void blueTeam(bool value);
		bool blueTeam() const
		{
			return _blueTeam;
		}
		
		boost::shared_ptr<Gameplay::GameplayModule> gameplayModule() const
		{
			return _gameplayModule;
		}
		
		boost::shared_ptr<RefereeModule> refereeModule() const
		{
			return _refereeModule;
		}
		
		SystemState *state()
		{
			return &_state;
		}
		
		boost::shared_ptr<ConfigFile> configFile()
		{
			return _config;
		}
		
		bool simulation() const
		{
			return _simulation;
		}

		Logger logger;
		
		// LogFrame being built for the current frame
		Packet::LogFrame logFrame;
		
		void defendPlusX(bool value);
		
		Status status()
		{
			QMutexLocker locker(&_statusMutex);
			return _status;
		}
		
		// Simulates a command from the referee
		void internalRefCommand(char ch);
		
		float framerate()
		{
			return _framerate;
		}
		
		// Time of the first LogFrame
		uint64_t firstLogTime;
		
	protected:
		void run();
		
	private:
		// Adds motor values to a RadioTx::Robot
		void addMotors(Packet::RadioTx::Robot *robot);
		
		/** send out the radio data for the radio program */
		void sendRadioData();

		/** handle incoming vision packet */
		void visionPacket(const SSL_WrapperPacket &wrapper, std::vector<SSL_DetectionFrame> &teamVision);
		
		/** Used to start and stop the thread **/
		volatile bool _running;

		// True if we are running with a simulator.
		// This changes network communications.
		bool _simulation;
		
		// If true, the processing loop waits to run until a vision packet is received.
		bool _syncToVision;
		
		// True if we are blue.
		// False if we are yellow.
		bool _blueTeam;
		
		// Locked when processing loop stuff is happening (not when blocked for timing or I/O).
		// This is public so the GUI thread can lock it to access SystemState, etc.
		QMutex _loopMutex;
		
		/** global system state */
		SystemState _state;

		// Transformation from world space to team space.
		// This depends on which goal we're defending.
		//
		// _teamTrans is used for positions, not angles.
		// _teamAngle is used for angles.
		Geometry2d::TransformMatrix _worldToTeam;
		float _teamAngle;
		
		// Board ID of the robot to manually control or -1 if none
		int _manualID;
		
		bool _defendPlusX;
		
		/** Which robot will next send reverse data */
		int _reverseId;
		
		// Processing period in microseconds
		int _framePeriod;
		
		// Which radio channel we're using
		int _radio;
		
		// True if we are using external referee packets
		bool _externalReferee;
		
		// Measured framerate
		float _framerate;
		
		// This is used by the GUI to indicate status of the processing loop and network
		QMutex _statusMutex;
		Status _status;
		
		// Network sockets
		QUdpSocket _visionSocket;
		QUdpSocket _refereeSocket;
		// _radioSocket is used for both sending (RadioTx) and receiving (RadioRx).
		QUdpSocket _radioSocket;

		//modules
		boost::shared_ptr<Modeling::WorldModel> _modelingModule;
		boost::shared_ptr<RefereeModule> _refereeModule;
		boost::shared_ptr<Gameplay::GameplayModule> _gameplayModule;
//		boost::shared_ptr<Motion::MotionModule> _motionModule; //FIXME: this should go away
		boost::shared_ptr<Motion::PointController> _pointControlModule;
		boost::shared_ptr<Motion::WheelController> _wheelControlModule;

		boost::shared_ptr<StateIdentification::StateIDModule> _stateIDModule;
		Joystick *_joystick;
		
		boost::shared_ptr<ConfigFile> _config;
};
