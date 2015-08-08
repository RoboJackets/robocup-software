#include "InterpolatedPath.hpp"

using namespace Planning;
using namespace std;


std::unique_ptr<Path> DirectPathPlanner::run(
        MotionInstant startInstant,
        MotionInstant endInstant,
        const MotionConstraints &motionConstraints,
        const Geometry2d::CompositeShape *obstacles)
{
    //return unique_ptr<Path>(new );
}