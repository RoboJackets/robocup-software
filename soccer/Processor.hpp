// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

//FIXME - Move a lot of stuff like blueTeam and worldToTeam to a globally accessible place

#pragma once

#include <QThread>
#include <QMutex>
#include <QMutexLocker>
#include <QUdpSocket>

#include <protobuf/LogFrame.pb.h>
#include <Network.hpp>
#include <Logger.hpp>
#include <Geometry2d/TransformMatrix.hpp>
#include <framework/SystemState.hpp>
#include <modeling/RobotFilter.hpp>

#include <boost/shared_ptr.hpp>

class Configuration;
class Joystick;
class RefereeModule;
class Radio;
class BallTracker;

namespace StateIdentification
{
	class StateIDModule;
}

namespace Gameplay
{
	class GameplayModule;
}

namespace Motion
{
	class RobotController;
}

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
		
		Processor(Configuration *config, bool sim);
		virtual ~Processor();
		
		void stop();
		
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
			QMutexLocker lock(&_loopMutex);
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
		
		bool simulation() const
		{
			return _simulation;
		}

		void defendPlusX(bool value);
		
		Status status()
		{
			QMutexLocker lock(&_statusMutex);
			return _status;
		}
		
		// Simulates a command from the referee
		void internalRefCommand(char ch);
		
		float framerate()
		{
			return _framerate;
		}
		
		const Logger &logger() const
		{
			return _logger;
		}
		
		bool openLog(const QString &filename)
		{
			return _logger.open(filename);
		}
		
		void closeLog()
		{
			_logger.close();
		}
		
		// Use all/part of the field
		void useOurHalf(bool value)
		{
			_useOurHalf = value;
		}
		
		void useOpponentHalf(bool value)
		{
			_useOpponentHalf = value;
		}
		
		QMutex &loopMutex()
		{
			return _loopMutex;
		}
		
		////////
		
		// Time of the first LogFrame
		uint64_t firstLogTime;
		
		// Access to RobotConfigs
		RobotConfig * robotConfig2008;
		RobotConfig * robotConfig2011;

	protected:
		void run();
		
	private:
		// Adds motor values to a RadioTx::Robot
		void addMotors(Packet::RadioTx::Robot *robot);
		
		/** send out the radio data for the radio program */
		void sendRadioData();

		void runModels(const std::vector<const SSL_DetectionFrame *> &detectionFrames);
		
		/** Used to start and stop the thread **/
		volatile bool _running;

		Logger _logger;
		
		Radio *_radio;
		
		bool _useOurHalf, _useOpponentHalf;
		
		// True if we are running with a simulator.
		// This changes network communications.
		bool _simulation;
		
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
		
		// Processing period in microseconds
		int _framePeriod;
		
		// True if we are using external referee packets
		bool _externalReferee;
		
		// Measured framerate
		float _framerate;
		
		// This is used by the GUI to indicate status of the processing loop and network
		QMutex _statusMutex;
		Status _status;
		
		// Network sockets
		QUdpSocket *_refereeSocket;

		//modules
		boost::shared_ptr<RefereeModule> _refereeModule;
		boost::shared_ptr<Gameplay::GameplayModule> _gameplayModule;
		boost::shared_ptr<BallTracker> _ballTracker;

		Joystick *_joystick;
};
