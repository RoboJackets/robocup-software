#include "MainWindow.hpp"
#include "log/LogModule.hpp"
#include "motion/Controller.hpp"
#include "modeling/WorldModel.hpp"
#include "motion/Trajectory.hpp"
#include <QFileDialog>
#include <ui_motion.h>
#include <Constants.hpp>

//TODO flicker is still an issue (though it only happens when you re-draw)

MainWindow::MainWindow(Team t, QString filename)
    :QMainWindow(), _processor(t, filename), _logFile(0), _config(filename)
{
    setupUi(this);

    this->setCentralWidget(centralwidget);

    _rp = new RobotPath(t,this);
    _rp->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    _rp->setAttribute(Qt::WA_OpaquePaintEvent);
    _rp->setAutoFillBackground(false);
    _logControl = new Log::LogControl(this);

    gridLayout->addWidget(_rp,0,0);
    gridLayout->addWidget(_logControl,1,0);

    qRegisterMetaType<Geometry::Point2d>("Geometry::Point2d");
    qRegisterMetaType<QVector<RobotPath::Path> >("QVector<RobotPath::Path>");

    _processor.start();

    _logFile = new Log::LogFile(Log::LogFile::genFilename());
    _processor.setLogFile(_logFile);

    try
    {
        _config.load();
    }
    catch (std::runtime_error& re)
    {
        printf("Config Load Error: %s\n", re.what());
    }

    kp->setValue(_config.robotConfig().posCntrlr.Kp);
    kd->setValue(_config.robotConfig().posCntrlr.Kd);

    _logControl->setLogFile(_logFile);
    connect(_logControl, SIGNAL(newFrame(Packet::LogFrame*)), _rp, SLOT(frame(Packet::LogFrame*)));
}

MainWindow::~MainWindow()
{
    _logControl->setLogFile(0);

    _processor.terminate();
    _processor.wait();

    //cleanup modules

    if (_logFile)
    {
        delete _logFile;
        _logFile = 0;
    }
}

void MainWindow::on_erase_clicked()
{
    _rp->erase();
}


void MainWindow::on_beizerCurve_clicked()
{
    _rp->addPath(RobotPath::BezierCurve);
}

void MainWindow::on_erasePath_clicked()
{
    _rp->eraseAllPaths();
}

void MainWindow::on_startPoint_clicked()
{
    _rp->addPath(RobotPath::Start);
}

void MainWindow::on_run_clicked()
{
    runTrajectoryGen();
    setPaths(_rp->getPaths());
}

void MainWindow::on_stop_clicked()
{
    stopTrajectoryGen();
}

void MainWindow::on_close_clicked()
{
    this->QWidget::close();
}

void MainWindow::on_kp_valueChanged(double value)
{
    kpChanged(value);
    _config.setElement("linearController,kp",value);
}

void MainWindow::on_kd_valueChanged(double value)
{
    kdChanged(value);
    _config.setElement("linearController,kd",value);
}

void MainWindow::on_saveGains_clicked()
{
    _config.save(QFileDialog::getSaveFileName(this, tr("Save File"),"../config/",tr("XML files (*.xml)")));
}

/*
Point2d MainWindow::convertPoint(QPointF point)
{
    Point2d retPoint;
    retPoint.y = (point.x() * (Constants::Floor::Length/802.0)) - Constants::Field::Border;
    retPoint.x = (point.y() * (Constants::Floor::Width/556.0)) - Constants::Field::Border;

    retPoint.x = retPoint.x - Constants::Field::Width/2;

    if(_team == Blue)
    {
        retPoint.y = Constants::Field::Length - retPoint.y;
        retPoint.x = -retPoint.x;
    }

    return retPoint;
}
*/