#ifndef _PATH_PLANNER_
#define _PATH_PLANNER_

#include <Geometry/Point2d.hpp>
#include "framework/SystemState.hpp"

class PathPlanner
{
    public:
        typedef struct
        {
            Geometry::Point2d center;
        }Obstacle;

        typedef struct
        {
            int length;
            std::vector<Geometry::Point2d> waypoints;
        }Path;

    public:
        PathPlanner();
        ~PathPlanner();

        Path plan(Geometry::Point2d currPos, Geometry::Point2d goal);
        void genObstacles();
        void aStar();

        void setAvoidOppSide(bool avoid)
        { _avoidOppSide = avoid; }

        void setState(SystemState *state)
        { _state = state; }

    private:
        bool _avoidOppSide;
        std::vector<PathPlanner::Obstacle> _obstacles;

        SystemState *_state;
};
#endif