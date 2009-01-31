#include "MainWindow.hpp"
// #include <module/modules.hpp>
#include "log/LogModule.hpp"
#include "motion/Controller.hpp"
//TODO  Get Alex to make world model a module
#include "../soccer/module/WorldModel.hpp"
#include "TrajectoryGen.hpp"

#include <ui_motion.h>

MainWindow::MainWindow(Team team, QString filename)
	   :QMainWindow(), _team(team), _processor(team), _logFile(0), _configFile(filename)
{
    setupUi(this);

    this->setCentralWidget(centralwidget);

    //RobotPath draws the path and also extends FieldView to draw the field, robots, ball, etc
    _rp = new RobotPath(team,this);
    _logControl = new Log::LogControl(this);
    gridLayout1->addWidget(_rp,0,0);
    gridLayout1->addWidget(_logControl, 1, 0);

    connect(&_timer, SIGNAL(timeout()), this, SLOT(redraw()));

    _timer.start(30);

    _processor.start();
    _logFile = new Log::LogFile(Log::LogFile::genFilename());
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

void MainWindow::redraw()
{
    _rp->update();
}

void MainWindow::setupModules()
{
	WorldModel* wm = new WorldModel();

	Motion::Controller* motion = new Motion::Controller(_configFile);

        Trajectory::TrajectoryGen* trajGen = new Trajectory::TrajectoryGen();

        //Thread-safe slots and signals require the use of a queue
        connect(this, SIGNAL(setModuleToRun()), trajGen, SLOT(runModule()), Qt::QueuedConnection);
        connect(this, SIGNAL(setModuleToStop()), trajGen, SLOT(stopModule()), Qt::QueuedConnection);

	Log::LogModule* lm = new Log::LogModule();
	lm->setLogFile(_logFile);

	//add the modules....ORDER MATTERS!!
	_processor.addModule(wm);
//         _processor.addModule(trajGen);
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
    setModuleToRun();
}

void MainWindow::on_stop_clicked()
{
    _mode = DrawingMode;
    setModuleToStop();
}

void MainWindow::on_close_clicked()
{
    printf("Close\n");
}
