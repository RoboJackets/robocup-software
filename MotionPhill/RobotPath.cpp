#include "RobotPath.hpp"

#include <Geometry/Point2d.hpp>
#include <Constants.hpp>

#include <QVector>
#include <QPainter>
#include <QPainterPath>
#include <QPointF>
#include <log/FieldView.hpp>

using namespace Constants;
RobotPath::RobotPath(Team team, QWidget* parent) :
    Log::FieldView(team, parent), _team(team)
{
    QPointF* initPoint = new QPointF(0,0);
    _numPathPoints = 0;
    _pathPointInterator = 0;

    for(int i = 0; i<4; i++)
    {
        currPath.points[i] = *initPoint;
    }

    setAutoFillBackground(false);

}

void RobotPath::paintEvent(QPaintEvent* event)
{

    Log::FieldView::paintEvent(event);
    QPainter painter(this);
    QPainterPath* painterPath = new QPainterPath(QPointF(0,0));

    painter.setPen(Qt::green);
    Q_FOREACH(RobotPath::Path p, _paths)
    {
	switch(p.type)
	{
	    case Line:
		painterPath->lineTo(p.points[0]);
                break;
            case Arc:
                //Arc parameters
		painterPath->quadTo(p.points[1],p.points[0]);
                break;
	    case Circle:
                break;
	    case Ellipse:
                break;
	    case Start:
		painterPath = new QPainterPath(p.points[0]);
                painterPath->addEllipse(p.points[0].x(),p.points[0].y(),1,1);
                painterPath->addEllipse(p.points[0].x() - 10,p.points[0].y() - 10,20,20);
                painterPath->moveTo(p.points[0]);
                break;
            case Close:
                painterPath->closeSubpath();
                break;
	    case BezierCurve:
		painterPath->cubicTo(p.points[1], p.points[2], p.points[0]);
                break;
	}
    }
    painter.drawPath(*painterPath);
}

void RobotPath::addPath(PathType pathType)
{
    currPath.type = pathType;
    switch(currPath.type)
    {
        case Line:
            currPath.numPoints = 1;
            break;
        case Arc:
            currPath.numPoints = 2;
            break;
        case Circle:
            currPath.numPoints = 2;
            break;
	case Ellipse:
            currPath.numPoints = 3;
            break;
        case Start:
            currPath.numPoints = 1;
            break;
        case BezierCurve:
            currPath.numPoints = 3;
            break;
        case Close:
            break;
    }

    currPath.points[1] = currPath.points[0];
    currPath.points[2] = currPath.points[0];

    _pathPointInterator = 0;

   _paths.append(currPath);
}

void RobotPath::eraseAllPaths()
{
    _paths.clear();

    for(int i = 0; i<4; i++)
    {
        currPath.points[i] = QPointF(0,0);
    }
}

void RobotPath::erase()
{
    if(!_paths.isEmpty())
    {
        _paths.remove(_paths.size()-1);
    }
    else
    {
        for(int i = 0; i<4; i++)
        {
            currPath.points[i] = QPointF(0,0);
        }
    }
}

void RobotPath::closePath()
{
    currPath.type = Close;

    currPath.numPoints = 0;
    currPath.points[1] = currPath.points[0];
    currPath.points[2] = currPath.points[0];

    _pathPointInterator = 0;

   _paths.append(currPath);
}

void RobotPath::mousePressEvent(QMouseEvent* me)
{
    if(_pathPointInterator % currPath.numPoints == 0)
    {
        currPath.points[2] = currPath.points[0];
        currPath.points[1] = currPath.points[0];
    }
}

void RobotPath::mouseReleaseEvent(QMouseEvent* me)
{
    currPath.points[_pathPointInterator % currPath.numPoints] = me->pos();
    _pathPointInterator++;
    _paths[_paths.size()-1] = currPath;
}

void RobotPath::mouseMoveEvent(QMouseEvent* me)
{
    currPath.points[_pathPointInterator % currPath.numPoints] = me->pos();
    _paths[_paths.size()-1] = currPath;
}

void RobotPath::mouseDoubleClickEvent(QMouseEvent* me)
{
}
