#include <gtest/gtest.h>

#include <rj_vision_filter/robot/CameraRobot.hpp>

namespace vision_filter {
TEST(CameraRobot, get_time_captured) {
    RJ::Time t = RJ::now();
    Geometry2d::Pose pose(Geometry2d::Point(1, 1), 1);
    int id = 0;

    CameraRobot b = CameraRobot(t, pose, id);

    RJ::Time r = b.getTimeCaptured();

    EXPECT_EQ(t, r);
}

TEST(CameraRobot, get_pos) {
    RJ::Time t = RJ::now();
    Geometry2d::Pose pose(Geometry2d::Point(1, 1), 1);
    int id = 0;

    CameraRobot b = CameraRobot(t, pose, id);

    Geometry2d::Point r = b.getPos();

    EXPECT_EQ(pose.position().x(), r.x());
    EXPECT_EQ(pose.position().x(), r.x());
}

TEST(CameraRobot, get_theta) {
    RJ::Time t = RJ::now();
    Geometry2d::Pose pose(Geometry2d::Point(1, 1), 1);
    int id = 0;

    CameraRobot b = CameraRobot(t, pose, id);

    double r = b.getTheta();

    EXPECT_EQ(pose.heading(), r);
}

TEST(CameraRobot, get_robot_id) {
    RJ::Time t = RJ::now();
    Geometry2d::Pose pose(Geometry2d::Point(1, 1), 1);
    int id = 0;

    CameraRobot b = CameraRobot(t, pose, id);

    int r = b.getRobotID();

    EXPECT_EQ(id, r);
}

TEST(CameraRobot, combine_zero) {
    std::list<CameraRobot> robots;

    CameraRobot r = CameraRobot::CombineRobots(robots);

    EXPECT_EQ(r.getPos().x(), 0);
    EXPECT_EQ(r.getPos().y(), 0);
    EXPECT_EQ(r.getTheta(), 0);
    EXPECT_EQ(r.getRobotID(), -1);
}

TEST(CameraRobot, combine_one) {
    RJ::Time t = RJ::now();
    Geometry2d::Pose pose(Geometry2d::Point(1, 1), 1);
    int id = 0;

    std::list<CameraRobot> robots;
    robots.emplace_back(t, pose, id);

    CameraRobot r = CameraRobot::CombineRobots(robots);

    EXPECT_EQ(r.getPos().x(), pose.position().x());
    EXPECT_EQ(r.getPos().y(), pose.position().y());
    EXPECT_EQ(r.getTimeCaptured(), t);
    EXPECT_EQ(r.getTheta(), pose.heading());
    EXPECT_EQ(r.getRobotID(), id);
}

TEST(CameraRobot, combine_two) {
    RJ::Time t1 = RJ::now();
    RJ::Time t2 = t1;
    Geometry2d::Pose pose1(Geometry2d::Point(1, 1), 1);
    Geometry2d::Pose pose2(Geometry2d::Point(2, 2), 1.5);
    int id = 0;

    std::list<CameraRobot> robots;
    robots.emplace_back(t1, pose1, id);
    robots.emplace_back(t2, pose2, id);

    CameraRobot r = CameraRobot::CombineRobots(robots);

    EXPECT_EQ(r.getPos().x(), (pose1.position().x() + pose2.position().x()) / 2);
    EXPECT_EQ(r.getPos().y(), (pose1.position().y() + pose2.position().y()) / 2);
    EXPECT_EQ(r.getTimeCaptured(), t1);
    EXPECT_EQ(r.getTheta(), (pose1.heading() + pose2.heading()) / 2);
    EXPECT_EQ(r.getRobotID(), id);
}
}  // namespace vision_filter
