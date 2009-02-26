#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <QString>
#include <framework/Module.hpp>
#include <RadioTx.hpp>

#include "Robot.hpp"
#include "ConfigFile.hpp"

namespace Motion
{
    class Controller : public Module
    {
        public:

	public:
	    Controller(QString filename);
	    ~Controller();

	    virtual void run();


	private:
            /** Robots **/
	    Robot* _robots[5];

            /** Config file from command line**/
	    ConfigFile _config;
    };
}

#endif /* CONTROLLER_HPP_ */
