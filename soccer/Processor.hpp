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

#include "InputHandler.hpp"
#include "framework/Module.hpp"

/** handles processing for a team */
class Processor: public QThread
{	
	Q_OBJECT;
	
	public:
		Processor(Team t);
		~Processor();

		//add a process to the queue
		//process are executed in the order they are placed
		//team handler is only responsible for triggering
		//this means that the SystemState is clear each time and
		//that only the vision information is fresh
		void addModule(Module* mod);
		
		//RunState runState() const { return _runState; }
		//void runState(RunState s);
		
		//ControlState controlState() const { return _controlState; }
		//void controlState(ControlState s);
		
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

	private:
		
		/** Used to start and stop the thread **/
		bool _running;

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
		std::list<Module*> _modules;
		
		InputHandler _inputHandler;
};

#endif // PROCESSOR_HPP
