#ifndef _MOTION_HPP_
#define _MOTION_HPP_

#include "Robot.hpp"
#include "ConfigFile.hpp"
#include <Team.h>
#include <framework/Module.hpp>

#include <QString>

class Motion : public Module
{
    public:
        Motion(QString filename);
        ~Motion();

        virtual void run();
    private:
        Robot* _robots[5];
        ConfigFile _config;
};
#endif
