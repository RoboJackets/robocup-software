#include "RobotPath.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QWidget>
#include <QPointF>
#include <Geometry/Point2d.hpp>

RobotPath::RobotPath()
{

}

void RobotPath::display(QPainter& p)
{
    QPainterPath path;

    p.setPen(Qt::black);

    switch(pathType)
    {
        case Line:
        case Circle:
	case Ellipse:
        case Polygon:
        case RoundedRectangle:
        case BezierCurve:
            path.cubicTo(_c1, _c2, _endpoint);

    }
    p.drawPath(path);
}

void RobotPath::setPath(QPointF c1, QPointF c2, QPointF endpoint)
{
    _c1 = c1;
    _c2 = c2;
    _endpoint = endpoint;
    pathType = BezierCurve;
}
