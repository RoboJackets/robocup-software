#include <gtest/gtest.h>
#include "planning/planner/PlanRequest.hpp"
#include "planning/planner/CapturePlanner.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "SystemState.hpp"
#include "Geometry2d/Pose.hpp"

using namespace Planning;
using namespace Geometry2d;

TEST(CapturePlanner, basic) {
    Context context;
    context.state.ball.pos = Point{1,1};
    context.state.ball.vel = Point{0,0};
    context.state.ball.time = RJ::now();
    PlanRequest request{&context, RobotInstant{{},{}, RJ::now()}, CaptureCommand{}, RobotConstraints{}, Trajectory{{}}, ShapeSet{}, {}, 2};
    CapturePlanner planner;
    Trajectory path = planner.plan(std::move(request));
    printf("produced path with %d instants of duration %.3f sec\n", path.num_instants(), path.duration().count());
    printf("iterating with TrajectoryIterator...\n");
    for(auto it = path.iterator(path.begin_time(), 100ms); it.hasValue(); ++it) {
        printf("(%.2f, %.2f)\n", (*it).pose.position().x(), (*it).pose.position().y());
    }
    printf("iterating with normal iterator...\n");
    for(auto it = path.instants_begin(); it != path.instants_end(); ++it) {
        printf("(%.3f, %.3f, %.3f) (%.3f, %.3f, %.3f) %.6f\n",
               (*it).pose.position().x(), (*it).pose.position().y(), (*it).pose.heading(),
               (*it).velocity.linear().x(), (*it).velocity.linear().y(), (*it).velocity.angular(),
               RJ::Seconds((*it).stamp-path.begin_time()).count()
        );
    }
}