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

    public:
        PathPlanner();
        ~PathPlanner();

        std::vector<Geometry::Point2d> plan(Geometry::Point2d goal);
        void genObstacles();
        void aStar();

        void setAvoidOppSide(bool avoid)
        { avoidOppSide = avoid; }

        void setState(SystemState *state)
        { _state = state; }

    private:
        bool avoidOppSide;
        std::vector<PathPlanner::Obstacle> _obstacles;

        SystemState *_state;



};
#endif