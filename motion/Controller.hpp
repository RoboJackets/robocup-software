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
            Controller(QString filename);
            ~Controller();

            virtual void run();

        public Q_SLOTS:
            void setKpGains(double value);
            void setKdGains(double value);
            //TODO This isn't right. I need to load the config file here so that mainwindow can access it
            void saveGains();

        private:
                /** Robots **/
            Robot* _robots[5];

                /** Config file from command line**/
            ConfigFile _config;
    };
}

#endif /* CONTROLLER_HPP_ */
