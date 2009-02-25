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
#include <QObject>

using namespace Geometry;

class RobotPath : public Log::FieldView
{
    Q_OBJECT;

    public:
        typedef enum
        {
            Line,
            Arc,
            Start,
            BezierCurve,
            Close,
        }PathType;

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

        QVector<Path> getPaths()
        {
            return _paths;
        }

        Point2d getSize()
        {
            return Point2d((float)this->height(),(float)this->width());
        }

    protected:
        void paintEvent(QPaintEvent* event);
        void mousePressEvent(QMouseEvent* me);
        void mouseReleaseEvent(QMouseEvent* me);
        void mouseMoveEvent(QMouseEvent* me);

    private:
        QVector<Path> _paths;

        QPointF _lastPoint;

        /** The current path (temp)**/
        Path currPath;

        Team _team;

        int _numPathPoints;

        int _pathPointInterator;

        bool drawing;

};

#endif
