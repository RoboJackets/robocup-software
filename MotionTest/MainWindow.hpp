#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <Geometry/Point2d.hpp>
#include "RobotPath.hpp"
#include "../soccer/Processor.hpp"
#include <log/LogFile.hpp>
#include "log/LogControl.hpp"

#include <QMainWindow>
#include <QMouseEvent>
#include <QTimer>
#include <QPointF>
#include <ui_motion.h>
#include <QString>
#include <config/ConfigFile.hpp>

class MainWindow : public QMainWindow, Ui::MainWindow
{
    Q_OBJECT;

    public:
        MainWindow(Team t, QString filename);
        ~MainWindow();

    public Q_SLOTS:
        void on_erase_clicked();
        void on_beizerCurve_clicked();
        void on_erasePath_clicked();
        void on_run_clicked();
        void on_stop_clicked();
        void on_startPoint_clicked();
        void on_close_clicked();
        void on_kp_valueChanged(double value);
        void on_kd_valueChanged(double value);
        void on_saveGains_clicked();

    Q_SIGNALS:
        void runTrajectoryGen();
        void stopTrajectoryGen();
        void setPaths(QVector<RobotPath::Path> paths);
        void kpChanged(double kp);
        void kdChanged(double kd);

    private:
        Ui::MainWindow ui;

        RobotPath* _rp;

        Processor _processor;

        Log::LogFile* _logFile;
        /** This controls the data flow from logger to visualizers **/
        Log::LogControl* _logControl;

        /** Filename for the configfile  **/
        QString _configFile;

        /** Currently the configfile is for motion but others can add to it **/
        ConfigFile _config;

};

#endif // MAIN_WINDOW_HPP
