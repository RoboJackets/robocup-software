#include "MainWindow.hpp"
#include "log/LogModule.hpp"
#include "motion/Controller.hpp"
#include "modeling/WorldModel.hpp"
#include "TrajectoryGen.hpp"
#include <QFileDialog>
#include <ui_motion.h>

//TODO flicker is still an issue (though it only happens when you re-draw)

MainWindow::MainWindow(Team team, QString filename)
    :QMainWindow(), _team(team), _processor(team), _logFile(0), _configFile(filename), _config(filename)
{
    setupUi(this);

    this->setCentralWidget(centralwidget);

    _rp = new RobotPath(team,this);
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

    setupModules();

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

void MainWindow::setupModules()
{
    Modeling::WorldModel* wm = new Modeling::WorldModel(_configFile);

    //TODO Get this out from vision don't make assumptions!!!!!
    unsigned int id[5];
    for(int i = 0; i<5; i++)
    {
        id[i] = i;
    }
    Motion::Controller* motion = new Motion::Controller(_config.robotConfig(), id);
    connect(this, SIGNAL(kpChanged(double)), motion, SLOT(setKpGains(double)), Qt::QueuedConnection);
    connect(this, SIGNAL(kdChanged(double)), motion, SLOT(setKdGains(double)), Qt::QueuedConnection);

    Trajectory::TrajectoryGen* trajGen = new Trajectory::TrajectoryGen();
    connect(this, SIGNAL(runTrajectoryGen()), trajGen, SLOT(runModule()), Qt::QueuedConnection);
    connect(this, SIGNAL(stopTrajectoryGen()), trajGen, SLOT(stopModule()), Qt::QueuedConnection);
    connect(this, SIGNAL(setPaths(QVector<RobotPath::Path>)), trajGen, SLOT(setPaths(QVector<RobotPath::Path>)), Qt::QueuedConnection);

    Log::LogModule* lm = new Log::LogModule();
    lm->setLogFile(_logFile);

    //add the modules....ORDER MATTERS!!
    _processor.addModule(wm);
    _processor.addModule(trajGen);
    _processor.addModule(motion);
    _processor.addModule(lm);
}

void MainWindow::on_erase_clicked()
{
    _rp->erase();
}

void MainWindow::on_line_clicked()
{
    _rp->addPath(RobotPath::Line);
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

void MainWindow::on_arc_clicked()
{
    _rp->addPath(RobotPath::Arc);
}

void MainWindow::on_closePath_clicked()
{
    _rp->closePath();
}

void MainWindow::on_run_clicked()
{
    _mode = RunMode;
    runTrajectoryGen();
    setPaths(_rp->getPaths());
}

void MainWindow::on_stop_clicked()
{
    _mode = DrawingMode;
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
