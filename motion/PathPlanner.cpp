#include "PathPlanner.hpp"

PathPlanner::PathPlanner()
{
}

PathPlanner::~PathPlanner()
{
}

std::vector<Geometry::Point2d> PathPlanner::plan(Geometry::Point2d goal)
{
    std::vector<Geometry::Point2d> waypoints;

    if(!_obstacles.empty())
    {
    }
    else
    {
        waypoints[0] = goal;
    }
    return waypoints;
}

void PathPlanner::genObstacles()
{
    //TODO - determine which robots in state and which no zones are relevant based on the current goal
}

void PathPlanner::aStar()
{
}