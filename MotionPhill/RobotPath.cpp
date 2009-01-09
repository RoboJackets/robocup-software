#include "RobotPath.hpp"

#include <Geometry/Point2d.hpp>
#include <Constants.hpp>

#include <QPainter>
#include <QPainterPath>
#include <QPointF>

using namespace Constants;
RobotPath::RobotPath(Team team, QWidget* parent) :
    QWidget(parent), _team(team)
{
    for(int i = 0; i<5; i++)
    {
        _pathParamPoints[i] = QPointF(0,0);
    }

    _numPathPoints = 0;
    _pathPointInterator = 0;

}

void RobotPath::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    QPainterPath path;

    painter.setPen(Qt::red);

    switch(currPath.type)
    {
        case Line:
            path.lineTo(currPath.points[1]);
        case Circle:
	case Ellipse:
        case Start:
            path.moveTo(currPath.points[1]);
        case Point:
	    path.moveTo(currPath.points[1]);
        case BezierCurve:
            path.cubicTo(currPath.points[1], currPath.points[2], currPath.points[0]);
    }
    painter.drawPath(path);
}

void RobotPath::setPath(PathType pathType)
{
    currPath.type = pathType;
    printf("this should be called once\n");

    switch(currPath.type)
    {
        case Line:
            printf("Line\n");
            currPath.numPoints = 1;
        case Circle:
            currPath.numPoints = 2;
	case Ellipse:
            currPath.numPoints = 3;
        case Start:
            printf("Start\n");
            currPath.numPoints = 1;
        case Point:
            printf("Point\n");
            currPath.numPoints = 1;
        case BezierCurve:
            printf("BeizerCurve\n");
            currPath.numPoints = 3;

    }
}

void RobotPath::mousePressEvent(QMouseEvent* me)
{
    if(_pathPointInterator % currPath.numPoints == 0)
    {
        currPath.points[2] = QPointF(0,0);
        currPath.points[1] = QPointF(0,0);
    }
}

void RobotPath::mouseReleaseEvent(QMouseEvent* me)
{
    currPath.points[_pathPointInterator % currPath.numPoints] = me->pos();
    _pathPointInterator++;
    printf("_pathPointInterator %d \n", currPath.type);

}

void RobotPath::mouseMoveEvent(QMouseEvent* me)
{
    currPath.points[_pathPointInterator % currPath.numPoints] = me->pos();
}

void RobotPath::mouseDoubleClickEvent(QMouseEvent* me)
{
}
