// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
#ifndef PROCESSOR_HPP
#define PROCESSOR_HPP

#include <QThread>
#include <QMutex>

#include <list>

#include <Network/Sender.hpp>
#include <Team.h>
#include <Geometry/TransformMatrix.hpp>
#include <Vision.hpp>
#include <RadioRx.hpp>

#include <log/LogModule.hpp>

#include <config/ConfigFile.hpp>

#include "InputHandler.hpp"
#include <framework/Module.hpp>

/** handles processing for a team */
class Processor: public QThread
{	
	Q_OBJECT;
	
	public:
		Processor(Team t, QString filename);
		~Processor();
		
		//RunState runState() const { return _runState; }
		//void runState(RunState s);
		
		//ControlState controlState() const { return _controlState; }
		//void controlState(ControlState s);
		
		void setLogFile(Log::LogFile* lf);
		
	public Q_SLOTS:
		void on_input_playPauseButton();
		void on_input_manualAutoButton();
		void on_input_changeRobot(int rid);
		
	protected:
		void run();

		/** handle incoming vision packet */
		void visionHandler(const Packet::Vision* packet);
		/** handle incoming radio packet */
		void radioHandler(const Packet::RadioRx* packet);
	
	private:
		/** clip angle to +/- 180 */
		static void trim(float& angle);
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

		Geometry::TransformMatrix _teamTrans;
		float _teamAngle;

		//modules
		QMutex _modulesMutex;
		
		Module* _motionModule;
		Module* _modelingModule;
		
		Log::LogModule* _logModule;
		
		InputHandler _inputHandler;
		
		Network::Sender _sender;
		
		/** Currently the configfile is for motion but others can add to it **/
		ConfigFile _config;
};

#endif // PROCESSOR_HPP
