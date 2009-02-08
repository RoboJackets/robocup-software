#ifndef SYSTEMSTATE_HPP_
#define SYSTEMSTATE_HPP_

#include <LogFrame.hpp>

/** The system state is an aggregate data structure for storing current system
 * state information through a loop of the system. A state variable is not valid
 * until that module has run */
class SystemState: public Packet::LogFrame
{
	public:
		
	private:
};

#endif /* SYSTEMSTATE_HPP_ */
