#ifndef _WORLD_MODEL_HPP_
#define _WORLD_MODEL_HPP_

#include "structures.hpp"
#include "Soccer.hpp"

class WorldModel
{

    public:

	//Generic Constructor
	WorldModel();

	//Links to other modules:
	void setSoc(Soccer *);

	//Update due to inputs
	void updateFromInput(const state_p *);
	void updateFromMotion(const robot_control_set *);

    private:

	//Links to subscribers
	Soccer * soc;
	Motion * mot;

	//primary data structure
	const state_pv model;

};

#endif //_WORLD_MODEL_HPP_
