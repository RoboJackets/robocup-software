#ifndef _ROBOTPATH_HPP_
#define _ROBOTPATH_HPP_

#include <Geometry/Point2d.hpp>
#include <QGLWidget>
#include <QPainter>
#include <QPointF>

using namespace Geometry;


class RobotPath : public QGLWidget
{
    public:
	typedef enum
	{
	    Line = 0,
	    Circle = 1,
	    Ellipse = 2,
	    Polygon = 3,
	    RoundedRectangle = 4,
	    BezierCurve = 5,
	} PathType;

    public:
        RobotPath(QWidget* parent = 0, QGLWidget* share = 0);
        void paintEvent(QPaintEvent* event);
        /**Bezier Curves using cubic**/
        void setPath(QPointF c1, QPointF c2, QPointF endpoint);
        /**Circles**/
        //void setPath(Circle2d circle);
        /** Lines **/
        //void setPath(Point2d endpoint1, Point2d endpoint2);
        /** Polygons **/
        //void setPath(QPolygonF poly);

    private:
        //TODO make it such that I don't need specific variables for all types of paths
        QPointF _c1;
        QPointF _c2;
        QPointF _endpoint;

        QPointF _currPos;

        PathType pathType;

};

#endif
