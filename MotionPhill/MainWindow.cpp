#include "MainWindow.hpp"
// #include <module/modules.hpp>
#include "log/LogModule.hpp"
#include "motion/Controller.hpp"
//TODO  Get Alex to make world model a module
#include "../soccer/module/WorldModel.hpp"

#include <ui_motion.h>

MainWindow::MainWindow(Team team, QString filename)
	   :QMainWindow(), _team(team), _processor(team), _logFile(0), _configFile(filename)
{
    setupUi(this);

    this->setCentralWidget(centralwidget);

    _rp = new RobotPath(team,this);

    gridLayout1->addWidget(_rp,0,0);

    _mode = DrawingMode;

    _logFile = new Log::LogFile(Log::LogFile::genFilename());

    connect(&_timer, SIGNAL(timeout()), this, SLOT(redraw()));
    _timer.start(30);

    _processor.start();
//     _logFile = new Log::LogFile(LogFile::genFilename());
    setupModules();

}

MainWindow::~MainWindow()
{
	//remove the log file from the things that use it
// 	_logControl->setLogFile(0);

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
    if(_mode == DrawingMode)
    {
        _rp->update();
    }
    else if(_mode == RunMode)
    {
    }
}

void MainWindow::setupModules()
{
	WorldModel* wm = new WorldModel();

	Motion::Controller* motion = new Motion::Controller(_configFile);

	Log::LogModule* lm = new Log::LogModule();
	lm->setLogFile(_logFile);

	//add the modules....ORDER MATTERS!!
	_processor.addModule(wm);
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
    //reset state
}

void MainWindow::on_stop_clicked()
{
    _mode = DrawingMode;
    //run calcs on data
    //switch focus to graphs
}

void MainWindow::on_close_clicked()
{
    printf("Close\n");
}


// void MainWindow::mouseMoveEvent(QMouseEvent* me)
// {
//     QString xlabel = "X = ";
//     QString ylabel = "Y =";
//
//     xlabel.append(QString::number(me->x(),'f', 3));
//     ylabel.append(QString::number(me->y(),'f', 3));
//     MainWindow::xPos->setText(xlabel);
//     MainWindow::yPos->setText(ylabel);
// }

