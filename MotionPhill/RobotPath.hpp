#ifndef _ROBOTPATH_HPP_
#define _ROBOTPATH_HPP_

#include <Geometry/Point2d.hpp>
#include <Team.h>

#include <QVector>
#include <QWidget>
#include <QMouseEvent>
#include <QPainter>
#include <QPointF>

using namespace Geometry;


class RobotPath : public QWidget
{
    Q_OBJECT;

    public:
        typedef enum
	{
	    Line,
	    Circle,
	    Ellipse,
            Start,
            Point,
	    BezierCurve,
	} PathType;

        typedef struct
        {
            int numPoints;
	    PathType type;
            QPointF points[4];
        } Path;



    public:
        RobotPath(Team team, QWidget* parent = 0);
        /**Bezier Curves using cubic**/
        void setPath(PathType pathType);
        /**Circles**/
        //void setPath(Circle2d circle);
        /** Lines **/
        //void setPath(Point2d endpoint1, Point2d endpoint2);
        /** Polygons **/
        //void setPath(QPolygonF poly);

    protected:
        void paintEvent(QPaintEvent* event);
        void mousePressEvent(QMouseEvent* me);
        void mouseReleaseEvent(QMouseEvent* me);
        void mouseMoveEvent(QMouseEvent* me);
        void mouseDoubleClickEvent(QMouseEvent* me);


    private:
        //TODO make it such that I don't need specific variables for all types of paths
        QPointF _pathParamPoints[4];
        QPointF _c1;
        QPointF _c2;
        QPointF _endpoint;

        QPointF _currPos;

        /** The current path (temp)**/
        Path currPath;

        Team _team;

        int _numPathPoints;

        int _pathPointInterator;

};

#endif
