#include <gtest/gtest.h>

#include <fstream>

#include "planning/low_level/AnglePlanning.hpp"

using namespace Planning;
using namespace Geometry2d;

TEST(AnglePlanning, FaceAngle) {
    Trajectory trajectory;
    trajectory.AppendInstant(RobotInstant{
        Pose(0, 0, 2),
        Twist(1, 0, 0),
        RJ::Time(0s)
    });
    trajectory.AppendInstant(RobotInstant{
        Pose(0.5, 0, 0),
        Twist(1, 0, 0),
        RJ::Time(500ms)
    });
    trajectory.AppendInstant(RobotInstant{
        Pose(1, 0, 0),
        Twist(1, 0, 0),
        RJ::Time(1000ms)
    });
    trajectory.AppendInstant(RobotInstant{
        Pose(1.5, 0, 0),
        Twist(1, 0, 0),
        RJ::Time(1500ms)
    });
    trajectory.AppendInstant(RobotInstant{
        Pose(2, 0, 0),
        Twist(1, 0, 0),
        RJ::Time(2000ms)
    });
    trajectory.AppendInstant(RobotInstant{
        Pose(2.5, 0, 0),
        Twist(1, 0, 0),
        RJ::Time(2500ms)
    });
    trajectory.AppendInstant(RobotInstant{
        Pose(3, 0, 0),
        Twist(1, 0, 0),
        RJ::Time(3000ms)
    });

    RotationConstraints constraints;
    constraints.maxAccel = 1.0;
    constraints.maxSpeed = 3.0;

    auto angle_fn = AngleFns::facePoint(Point(2, 0.5));
//    auto angle_fn = AngleFns::faceAngle(1.12);
//    auto angle_fn = AngleFns::tangent;
    PlanAngles(&trajectory,
               trajectory.first(),
               angle_fn,
               constraints);

    std::ofstream out("/tmp/out.csv");
    auto cursor = trajectory.cursor(trajectory.begin_time());
    for (; cursor.has_value(); cursor.advance(0.01s)) {
        RobotInstant instant = cursor.value();
        out << RJ::Seconds(instant.stamp - trajectory.begin_time()).count()
            << ", " << instant.heading()
            << ", " << instant.angular_velocity()
            << ", " << angle_fn(instant.linear_motion(), instant.heading(), nullptr)
            << std::endl;
    }
}
