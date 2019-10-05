#include <gtest/gtest.h>
#include "vision/robot/CameraRobot.hpp"

TEST(CameraRobot, get_time_captured) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1,1);
    double th = 1;
    int id = 0;

    CameraRobot b = CameraRobot(t, p, th, id);

    RJ::Time r = b.getTimeCaptured();

    EXPECT_EQ(t, r);
}

TEST(CameraRobot, get_pos) {
RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1,1);
    double th = 1;
    int id = 0;

    CameraRobot b = CameraRobot(t, p, th, id);

    Geometry2d::Point r = b.getPos();

    EXPECT_EQ(p.x(), r.x());
    EXPECT_EQ(p.x(), r.x());
}

TEST(CameraRobot, get_theta) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1,1);
    double th = 1;
    int id = 0;

    CameraRobot b = CameraRobot(t, p, th, id);

    double r = b.getTheta();

    EXPECT_EQ(th, r);
}

TEST(CameraRobot, get_robot_id) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1,1);
    double th = 1;
    int id = 0;

    CameraRobot b = CameraRobot(t, p, th, id);

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
    Geometry2d::Point p = Geometry2d::Point(1,1);
    double th = 1;
    int id = 0;

    std::list<CameraRobot> robots;
    robots.emplace_back(t, p, th, id);

    CameraRobot r = CameraRobot::CombineRobots(robots);

    EXPECT_EQ(r.getPos().x(), p.x());
    EXPECT_EQ(r.getPos().y(), p.y());
    EXPECT_EQ(r.getTimeCaptured(), t);
    EXPECT_EQ(r.getTheta(), th);
    EXPECT_EQ(r.getRobotID(), id);
}

TEST(CameraRobot, combine_two) {
    RJ::Time t1 = RJ::now();
    RJ::Time t2 = t1;
    Geometry2d::Point p1 = Geometry2d::Point(1,1);
    Geometry2d::Point p2 = Geometry2d::Point(2,2);
    double th1 = 1;
    double th2 = 1.5;
    int id = 0;

    std::list<CameraRobot> robots;
    robots.emplace_back(t1, p1, th1, id);
    robots.emplace_back(t2, p2, th2, id);

    CameraRobot r = CameraRobot::CombineRobots(robots);

    EXPECT_EQ(r.getPos().x(), (p1.x() + p2.x()) / 2);
    EXPECT_EQ(r.getPos().y(), (p1.y() + p2.y()) / 2);
    EXPECT_EQ(r.getTimeCaptured(), t1);
    EXPECT_EQ(r.getTheta(), (th1 + th2) / 2);
    EXPECT_EQ(r.getRobotID(), id);
}