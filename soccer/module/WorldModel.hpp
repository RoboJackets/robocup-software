#ifndef WORLDMODEL_HPP_
#define WORLDMODEL_HPP_

#include "framework/Module.hpp"

/** process raw vision data and creates system sanitized data */
class WorldModel : public Module
{
	public:
		WorldModel();
	
		virtual void run();
		
	private:
		SystemState::WorldModelOut _old;
};

#endif /* WORLDMODEL_HPP_ */
