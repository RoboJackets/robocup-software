// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

#include <QThread>
#include <QMutex>

#include <list>

#include <Network/Sender.hpp>
#include <Team.h>
#include <Geometry2d/TransformMatrix.hpp>
#include <Vision.hpp>
#include <RadioRx.hpp>

#include <gameplay/GameplayModule.hpp>
#include <motion/MotionModule.hpp>
#include <log/LogModule.hpp>
#include <stateID/StateIDModule.hpp>

#include <framework/ConfigFile.hpp>

#include "InputHandler.hpp"
#include "RefereeModule.hpp"

/** handles processing for a team */
class Processor: public QThread
{	
	Q_OBJECT;
	
	class SlowThread : public QThread
	{
		public:
			SlowThread(Processor* p) : proc(p) {}
			
			void run();
			
			Processor* proc;
			
			//the copy of the system state for the slow thread
			//this will be the lastest state available
			SystemState _state;
	};
	
	public:
		Processor(Team t, QString filename);
		~Processor();
		
		void start();
		
		Gameplay::GameplayModule *gameplayModule() const
		{
			return _gameplayModule;
		}
		
		void setLogFile(Log::LogFile* lf);
		
		SystemState& state()
		{
			return _state;
		}
		
		//return a list of all the modules */
		const QVector<Module*>& modules()
		{
			return _modules;
		}
		
	public Q_SLOTS:
		void on_input_playPauseButton();
		void on_input_manualAutoButton();
		void on_input_selectRobot(int rid);
		void flip_field(bool flip);
		
	protected:
		void run();

		/** handle incoming vision packet */
		void visionHandler(const Packet::Vision* packet);
		/** handle incoming radio packet */
		void radioHandler(const Packet::RadioRx* packet);
	
	private:
		/** convert all coords to team space */
		void toTeamSpace(Packet::Vision& vision);
		
		/** reset certain system state variables
		 * NOTE This should ALWAYS happen after a send on radio */
		void clearState();
		
		/** send out the radio data for the radio program */
		void sendRadioData();

	private:
		
		/** Used to start and stop the thread **/
		volatile bool _running;

		/** trigger camera id, triggers syncronous processing */
		int _triggerId;

		/** if true, the system should run */
		bool _trigger;

		/** team we are running as */
		Team _team;
		
		/** global system state */
		SystemState _state;

		Geometry2d::TransformMatrix _teamTrans;
		float _teamAngle;
		
		bool _flipField;

		//modules
		QMutex _modulesMutex;
		
		Module* _modelingModule;
		RefereeModule *_refereeModule;
		Gameplay::GameplayModule *_gameplayModule;
		Module* _motionModule;
		Log::LogModule *_logModule;
		StateIdentification::StateIDModule *_stateIDModule;
		
		InputHandler *_inputHandler;
		
		Network::Sender _sender;
		
		/** Currently the configfile is for motion but others can add to it **/
		ConfigFile _config;
		
		/** list of all modules */
		QVector<Module*> _modules;
		
		SlowThread _slowThread;
};
