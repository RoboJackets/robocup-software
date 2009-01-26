#ifndef _ROBOTPATH_HPP_
#define _ROBOTPATH_HPP_

#include <Geometry/Point2d.hpp>
#include <Team.h>
#include <log/FieldView.hpp>

#include <QVector>
#include <QWidget>
#include <QMouseEvent>
#include <QPainter>
#include <QPointF>

using namespace Geometry;


class RobotPath : public Log::FieldView
{
    Q_OBJECT;

    public:
        typedef enum
	{
	    Line,
            Arc,
	    Circle,
	    Ellipse,
            Start,
	    BezierCurve,
            Close,
	} PathType;

        typedef struct
        {
            int numPoints;
	    PathType type;
            QPointF points[4];
        } Path;



    public:
        RobotPath(Team team, QWidget* parent = 0);
        void addPath(PathType pathType);

        void eraseAllPaths();
        void erase();
        void closePath();

    protected:
        void paintEvent(QPaintEvent* event);
        void mousePressEvent(QMouseEvent* me);
        void mouseReleaseEvent(QMouseEvent* me);
        void mouseMoveEvent(QMouseEvent* me);
        void mouseDoubleClickEvent(QMouseEvent* me);


    private:
        QVector<Path> _paths;
//         QPointF _c1;
//         QPointF _c2;
//         QPointF _endpoint;

        QPointF _currPos;

        /** The current path (temp)**/
        Path currPath;

        Team _team;

        int _numPathPoints;

        int _pathPointInterator;

};

#endif
