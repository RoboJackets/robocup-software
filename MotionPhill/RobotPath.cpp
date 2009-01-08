#include "RobotPath.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QPointF>
#include <Geometry/Point2d.hpp>

RobotPath::RobotPath(QWidget* parent, QGLWidget* share) :
    QGLWidget(parent,share)
{
}

void RobotPath::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    QPainterPath path;

    painter.setPen(Qt::red);

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
    painter.setBackgroundMode(Qt::TransparentMode);
    painter.drawPath(path);
}

void RobotPath::setPath(QPointF c1, QPointF c2, QPointF endpoint)
{
    _c1 = c1;
    _c2 = c2;
    _endpoint = endpoint;
    pathType = BezierCurve;
}
