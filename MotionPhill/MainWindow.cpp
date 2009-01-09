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
        _rp->setPath(RobotPath::BezierCurve);

        gridLayout1->addWidget(_fieldView,0,0);
        gridLayout1->addWidget(_rp,0,0);

        connect(&_timer, SIGNAL(timeout()), this, SLOT(redraw()));
        _timer.start(30);
}

MainWindow::~MainWindow()
{
}

void MainWindow::redraw()
{
//     _fieldView->update();
    _rp->update();
}

void MainWindow::on_point_clicked()
{
    _rp->setPath(RobotPath::Point);
}

void MainWindow::on_line_clicked()
{
    _rp->setPath(RobotPath::Line);
}

void MainWindow::on_beizerCurve_clicked()
{
    _rp->setPath(RobotPath::BezierCurve);
}

void MainWindow::mouseMoveEvent(QMouseEvent* me)
{
//     QString xlabel = "X = ";
//     QString ylabel = "Y =";
//
//     xlabel.append(QString::number(me->x(),'f', 3));
//     ylabel.append(QString::number(me->y(),'f', 3));
//     MainWindow::xPos->setText(xlabel);
//     MainWindow::yPos->setText(ylabel);
}

