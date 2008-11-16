#ifndef MODULE_HPP_
#define MODULE_HPP_

#include "SystemState.hpp"

#include <string>
#include <map>

/** a module is called by the system trigger to process its data */
class Module
{
	public:
		virtual ~Module() {};
		
		void setSystemState(SystemState* state);
		
		std::string name() const { return _name; }
		
		/** run the code for the module */
		virtual void run() = 0;
		
		/** return the module with the given name */
		static Module* module(std::string name);
		
	private:
		Module(Module&);
		Module& operator&=(Module&);
		
	protected:
		Module(std::string name);
		
		/** this WILL BE SET before the module is run */
		SystemState* _state;
		
		std::string _name;
		
		static std::map<std::string, Module*> _modules;
};

#endif /* MODULE_HPP_ */
