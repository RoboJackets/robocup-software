#include "MainWindow.hpp"

#include <QGLFormat>
#include <ui_motion.h>
#include <QString>
#include <QGridLayout>
using namespace Log;

MainWindow::MainWindow(Team team)
	   :QMainWindow(), _team(team)
{
        setupUi(this);

        //TODO I should be able to add a QTGLwidget to a layout in the designer
	this->setCentralWidget(centralwidget);

	_fieldView = new FieldView(_team, this);
        gridLayout1->addWidget(_fieldView, 0, 0);


        _rp = new RobotPath(this,_fieldView);
        //_rp->setParent(_fieldView);
        _rp->setPath(QPointF(2,1),QPointF(2,2),QPointF(500,10));
	_rp->setAttribute(Qt::WA_NoSystemBackground);


        QGridLayout* gridLayout2 = new QGridLayout(_fieldView);
        gridLayout2->setContentsMargins(0, 0, 0, 0);
        gridLayout2->addWidget(_rp,0,0);

        connect(&_timer, SIGNAL(timeout()), SLOT(redraw()));
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

