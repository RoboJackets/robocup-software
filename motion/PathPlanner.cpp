#include "PathPlanner.hpp"

PathPlanner::PathPlanner()
{
}

PathPlanner::~PathPlanner()
{
}

PathPlanner::Path PathPlanner::plan(Geometry::Point2d currPos, Geometry::Point2d goal)
{
    std::vector<Geometry::Point2d> waypoints;
    Path p;
    if(!_obstacles.empty())
    {
    }
    else
    {
        waypoints.pushBack(goal);
        p.length =  currPos.distTo(goal);
    }

    p.waypoints = waypoints;
    return p;
}

void PathPlanner::genObstacles()
{
    //TODO - determine which robots in state and which no zones are relevant based on the current goal
}

void PathPlanner::aStar()
{
}