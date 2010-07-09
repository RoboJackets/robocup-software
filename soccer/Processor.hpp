// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

#include <QThread>
#include <QMutex>
#include <QUdpSocket>

#include <list>

#include <boost/shared_ptr.hpp>

#include <Team.h>
#include <Geometry2d/TransformMatrix.hpp>
#include <Vision.hpp>

#include <gameplay/GameplayModule.hpp>
#include <motion/MotionModule.hpp>
#include <stateID/StateIDModule.hpp>

#include <framework/ConfigFile.hpp>

#include "JoystickInput.hpp"
#include "RefereeModule.hpp"

/** handles processing for a team */
class Processor: public QThread
{	
	Q_OBJECT;
	
	public:
		Processor(Team t, QString filename, QObject *mainWindow);
		~Processor();
		
		boost::shared_ptr<Gameplay::GameplayModule> gameplayModule() const
		{
			return _gameplayModule;
		}
		
		SystemState& state()
		{
			return _state;
		}
		
		//return a list of all the modules */
		const QVector<boost::shared_ptr<Module> >& modules()
		{
			return _modules;
		}
		
		boost::shared_ptr<ConfigFile> configFile()
		{
			return _config;
		}

		Packet::LogFrame lastFrame();

		/** Multicast address for vision */
		QString vision_addr;
		
	public Q_SLOTS:
		void flipField(bool flip);
		
	protected:
		void run();

	private:
		/** convert all coords to team space */
		void toTeamSpace(Packet::Vision& vision);
		
		/** send out the radio data for the radio program */
		void sendRadioData();

		/** handle incoming vision packet */
		void visionPacket(const std::vector<uint8_t>* packet);
		
		/** handle incoming radio packet */
		void radioHandler(const Packet::RadioRx* packet);
	
		/** Used to start and stop the thread **/
		volatile bool _running;

		/** if true, the system should run */
		bool _trigger;

		/** team we are running as */
		Team _team;
		
		/** global system state */
		SystemState _state;

		Geometry2d::TransformMatrix _teamTrans;
		float _teamAngle;
		
		bool _flipField;
		
		/** Which robot will next send reverse data */
		int _reverseId;
		
		// Time taken to run the last frame in microseconds
		int _lastFrameTime;
		
		// Processing period in microseconds
		int _framePeriod;
		
		QMutex _frameMutex;
		Packet::LogFrame _lastFrame;
		QObject *_mainWindow;
		void captureState();
		
		QUdpSocket _visionSocket;
		QUdpSocket _radioSocket;

		//modules
		QMutex _modulesMutex;
		
		boost::shared_ptr<Module> _modelingModule;
		boost::shared_ptr<RefereeModule> _refereeModule;
		boost::shared_ptr<Gameplay::GameplayModule> _gameplayModule;
		boost::shared_ptr<Module> _motionModule;
		boost::shared_ptr<StateIdentification::StateIDModule> _stateIDModule;
		
		boost::shared_ptr<JoystickInput> _joystick;
		
		boost::shared_ptr<ConfigFile> _config;
		
		/** list of all modules */
		QVector<boost::shared_ptr<Module> > _modules;
};
