#include <framework/Module.hpp>

#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

namespace Motion
{
	class Controller : public Module
	{
		public:
			Controller();
			
			virtual void run();
			
		private:
	};
}

#endif /* CONTROLLER_HPP_ */
