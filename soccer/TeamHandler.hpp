#ifndef TEAMHANDLER_HPP_
#define TEAMHANDLER_HPP_

#include <QThread>
#include <QMutex>

#include <list>

#include <Team.h>
#include <Geometry/TransformMatrix.hpp>
#include <Vision.hpp>

#include "framework/Module.hpp"

/** handles processing for a team */
class TeamHandler : public QThread
{	
	public:
		TeamHandler(Team t);
		~TeamHandler();
		
		//add a process to the queue
		//process are executed in the order they are placed
		//team handler is only responsible for triggering
		//this means that the SystemState is clear each time and
		//that only the vision information is fresh
		void addModule(Module* mod);
		
	protected:
		void run();

		void visionHandler(const Packet::Vision* packet);
		//void radioHandler(const Packet::RadioRx* packet);
		
	private:
		/** clip angle to +/- 180 */
		static void trim(float& angle);

	/// members ///
	private:
		bool _running;
		
		/** system is clocked by one camera only, 
		 * the first camera packet received is the one that will do the clocking */
		int _camTrigId;
		
		/** if true, the system should run */
		bool _trigger;
		
		/** team we are running as */
		Team _team;
		
		/** global system state */
		SystemState _state;
		
		Geometry::TransformMatrix _teamTrans;
		float _teamAngle;
		
		//processes
		QMutex _modulesMutex;
		std::list<Module*> _modules;
		
};

#endif /* TEAMHANDLER_HPP_ */
