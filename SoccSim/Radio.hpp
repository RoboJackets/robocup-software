#pragma once

#include <QThread>

#include <Team.h>
#include "Physics/Env.hpp"

/** sends and receives radio data to/from the environment
 * Acts as the host radio program and the radio for the robots */
class Radio : public QThread
{
	public:
		Radio(Team t, Env& env);
		~Radio();
			
		void handler(const Packet::RadioTx* packet);
		
	protected:
		void run();
		
	private:
		Env& _env;
		
		bool _running;
		
		Team _team;
};
