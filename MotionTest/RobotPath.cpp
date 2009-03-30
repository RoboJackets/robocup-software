#include "RobotPath.hpp"

#include <Constants.hpp>
#include <log/FieldView.hpp>

#include <QVector>
#include <QPainter>
#include <QPainterPath>
#include <QPointF>
#include <boost/foreach.hpp>

using namespace Constants;

RobotPath::RobotPath(Team team, QWidget* parent) :
    Log::FieldView(parent), _team(team)
{
    _numPathPoints = 0;
    _pathPointInterator = 0;

    this->team(team);

    //Prevent the background from being re-draw prior to painting
//     setAutoFillBackground(false);

    _lastPoint = QPointF(0,0);
}

void RobotPath::paintEvent(QPaintEvent* event)
{
    //Paint the field
    Log::FieldView::paintEvent(event);

    QPainter painter(this);

    QPainterPath* painterPath = new QPainterPath(QPointF(0,0));

    painter.setPen(Qt::green);
    BOOST_FOREACH(RobotPath::Path p, _paths)
    {
        switch(p.type)
        {
            case Line:
                painterPath->lineTo(p.points[0]);
                break;
            case Arc:
                painterPath->quadTo(p.points[1],p.points[0]);
                break;
            case Start:
                painterPath = new QPainterPath(p.points[0]);
                painter.drawEllipse(p.points[0].x(),p.points[0].y(),1,1);
                painter.drawEllipse(p.points[0].x() - 10,p.points[0].y() - 10,20,20);
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
    free(painterPath);
    painter.end();
}

void RobotPath::addPath(PathType pathType)
{
    Path path;
    path.type = pathType;
    switch(path.type)
    {
        case Line:
            path.numPoints = 1;
            break;
        case Arc:
            path.numPoints = 2;
            break;
        case Start:
            path.numPoints = 1;
            break;
        case BezierCurve:
            path.numPoints = 3;
            break;
        case Close:
            break;
    }

    for(int i = 0; i<path.numPoints+1; i++)
    {
        path.points.append(_lastPoint);
    }

    _pathPointInterator = 0;

   _paths.append(path);
}

void RobotPath::eraseAllPaths()
{
    _paths.clear();
    update();
}

void RobotPath::erase()
{
    if(!_paths.isEmpty())
    {
        _paths.remove(_paths.size()-1);
    }

    update();
}

void RobotPath::closePath()
{
    Path path;
    path.type = Close;

    path.numPoints = 0;

   _paths.append(path);
   update();
}

void RobotPath::mousePressEvent(QMouseEvent* me)
{
    if(!_paths.isEmpty())
    {
        Q_FOREACH(QPointF p, _paths.last().points)
        {
            p = _lastPoint;
        }
        drawing = true;
    }
}

void RobotPath::mouseReleaseEvent(QMouseEvent* me)
{
    if(drawing)
    {
        //Move to the next point
        _pathPointInterator++;
        drawing = false;
    }
}

void RobotPath::mouseMoveEvent(QMouseEvent* me)
{
    if(!_paths.isEmpty() && drawing)
    {
        _paths.last().points[_pathPointInterator % _paths.last().numPoints] = me->pos();
        _lastPoint = me->pos();
        update();
    }
}
