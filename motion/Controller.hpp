#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <QString>
#include <framework/Module.hpp>
#include <RadioTx.hpp>

#include "Robot.hpp"
#include "ConfigFile.hpp"
#include "Gamepad.hpp"

namespace Motion
{
    class Controller : public Module
    {
        public:
            enum CmdMode
            {
                AUTO,
                MANUAL
            };

	public:
	    Controller(QString filename);
	    ~Controller();

	    virtual void run();


	private:
	    Robot* _robots[5];
	    ConfigFile _config;

            /** Defaults to Man if the controller is plugged in **/
            CmdMode _mode;

	    /** Gamepad **/
	    Gamepad* _gamepad;

    };
}

#endif /* CONTROLLER_HPP_ */
