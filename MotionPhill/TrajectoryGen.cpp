#include "TrajectoryGen.hpp"

using namespace Trajectory;

TrajectoryGen::TrajectoryGen() :
    Module("TrajectoryGen"), _running(true)
{
    printf("TrajectoryGen is running\n");
}

TrajectoryGen::~TrajectoryGen()
{

}

void TrajectoryGen::run()
{
    while(_running)
    {
    }
}

void TrajectoryGen::runModule()
{
    _running = true;
    printf("Module State Set to Run\n");
}

void TrajectoryGen::stopModule()
{
    _running = false;
    printf("Module State Set to Stop\n");
}
