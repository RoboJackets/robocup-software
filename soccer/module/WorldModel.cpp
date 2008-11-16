#include "WorldModel.hpp"

#include <QObject>

WorldModel::WorldModel() :
	Module("World Model")
{
	
}

void WorldModel::run()
{
	//calculate velocities for now
	
	/*
	SystemState::WorldModelOut& state = _state->worldModelOut;
	
	Q_FOREACH (const Packet::LocVision::Robot& r, _state->vision.self)
	{
		SystemState::WorldModelOut::Robot& rb = state.robots[r.shell % 5];
		rb.valid = true;
		rb.pos = r.pos;
	}
	
	for (unsigned int i=0 ; i<5 ; ++i)
	{
		state.robots[i].vel = state.robots[i].pos - _old.robots[i].pos;
		
		if (i == 0)
		{
			printf("speed: %f\n", state.robots[i].vel.mag());
		}
	}
	
	_old = state;
	*/
}
