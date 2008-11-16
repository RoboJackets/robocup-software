#ifndef MODULE_HPP_
#define MODULE_HPP_

#include "SystemState.hpp"
#include <string>

/** a module is called by the system trigger to process its data */
class Module
{
	public:
		Module(std::string name);
		virtual ~Module() {};
		
		void setSystemState(SystemState* state);
		
		std::string name() const { return _name; }
		
		/** run the code for the module */
		virtual void run() = 0;
		
	private:
		Module(Module&);
		Module& operator&=(Module&);
		
	protected:
		/** this WILL BE SET before the module is run */
		SystemState* _state;
		
		std::string _name;
		
		//static std::vector<Module*> _modules;
};

#endif /* MODULE_HPP_ */
