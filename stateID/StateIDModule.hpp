// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et
#pragma once

#include <framework/Module.hpp>

/** Semantic State Identification system */
namespace StateIdentification
{
	class StateIDModule : public Module
	{
		public:
			StateIDModule();
			~StateIDModule();
			
			virtual void run();
	};
}
