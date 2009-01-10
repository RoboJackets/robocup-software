#include "MainWindow.hpp"

#include <ui_motion.h>
#include <QString>

MainWindow::MainWindow(Team team)
	   :QMainWindow(), _team(team)
{
        setupUi(this);

	this->setCentralWidget(centralwidget);

        _fieldView = new FieldView(team,this);
        _rp = new RobotPath(team,this);
        _rp->addPath(RobotPath::BezierCurve);

        gridLayout1->addWidget(_fieldView,0,0);
        gridLayout1->addWidget(_rp,0,0);

        _mode = DrawingMode;

        connect(&_timer, SIGNAL(timeout()), this, SLOT(redraw()));
        _timer.start(30);
}

MainWindow::~MainWindow()
{
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
    //start processor
}

void MainWindow::on_stop_clicked()
{
    _mode = DrawingMode;
    //run calcs on data
    //switch focus to graphs
}

void MainWindow::on_close_clicked()
{

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

