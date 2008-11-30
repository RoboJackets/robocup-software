#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <QString>
#include <framework/Module.hpp>

#include "Robot.hpp"
#include "ConfigFile.hpp"

namespace Motion
{
	class Controller : public Module
	{
		public:
		    Controller(QString filename);
                    ~Controller();

		    virtual void run();


		private:
                    Robot* _robots[5];
                    ConfigFile _config;
	};
}

#endif /* CONTROLLER_HPP_ */
