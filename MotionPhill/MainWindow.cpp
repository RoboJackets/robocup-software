#include "MainWindow.hpp"

#include <ui_motion.h>
#include <QMouseEvent>
#include <QString>
#include <QPointF>
#include <QTimer>
#include "FieldDisplay.hpp"
#include "RobotPath.hpp"

MainWindow::MainWindow(Team team)
	   :QMainWindow()
{
        setupUi(this);
        field->setTeam(team);
        connect(field,SIGNAL(newPosition(float, float, float, float, QMouseEvent)),this,SLOT(cursorPosition(float, float, float, float, QMouseEvent)));

        MainWindow::rp.setPath(QPointF(2,2),QPointF(2,2),QPointF(2,2));
        field->setRobotPath(rp);

        connect(&_timer, SIGNAL(timeout()), SLOT(redraw()));
	_timer.start(30);
}

MainWindow::~MainWindow()
{
}

void MainWindow::redraw()
{
    field->update();
}

void MainWindow::cursorPosition(float x, float y, float wx, float wy, QMouseEvent me)
{
    QString xlabel = "X = ";
    QString ylabel = "Y =";

    xlabel.append(QString::number((double)x,'f', 3));
    ylabel.append(QString::number((double)y,'f', 3));
    MainWindow::xPos->setText(xlabel);
    MainWindow::yPos->setText(ylabel);

    MainWindow::mousePoint.setX(x);
    MainWindow::mousePoint.setY(y);
}

