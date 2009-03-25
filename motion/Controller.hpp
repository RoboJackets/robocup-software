#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <QObject>
#include <QString>
#include <framework/Module.hpp>
#include <RadioTx.hpp>

#include "Robot.hpp"
#include <config/ConfigFile.hpp>

namespace Motion
{
    class Controller : public QObject, public Module
    {
        Q_OBJECT;

        public:
            Controller(ConfigFile::RobotCfg cfg, unsigned int id[]);
            ~Controller();

            virtual void run();

        public Q_SLOTS:
            void setKpGains(double value);
            void setKdGains(double value);

        private:
            /** Robots **/
            Robot* _robots[5];
    };
}

#endif /* CONTROLLER_HPP_ */
