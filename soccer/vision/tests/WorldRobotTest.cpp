#include <gtest/gtest.h>
#include "vision/robot/WorldRobot.hpp"

TEST(WorldRobot, invalid) {
    WorldRobot wb;

    EXPECT_FALSE(wb.getIsValid());
}

TEST(WorldRobot, no_robot) {
    std::list<KalmanRobot> kbl;

    WorldRobot wb = WorldRobot(RJ::now(), WorldRobot::Team::BLUE, 1, kbl);

    EXPECT_FALSE(wb.getIsValid());
}

TEST(WorldRobot, one_robot) {
    RJ::Time t = RJ::now();
    geometry2d::Pose pose(geometry2d::Point(1, 1), 1);
    int rID = 1;
    CameraRobot b = CameraRobot(t, pose, rID);
    int cID = 1;
    WorldRobot w;

    KalmanRobot kb = KalmanRobot(cID, t, b, w);

    std::list<KalmanRobot> kbl;
    kbl.push_back(kb);

    WorldRobot wb = WorldRobot(t, WorldRobot::Team::BLUE, rID, kbl);

    geometry2d::Point rp = wb.getPos();
    double rt = wb.getTheta();
    geometry2d::Point rv = wb.getVel();
    double ro = wb.getOmega();

    double rpc = wb.getPosCov();
    double rvc = wb.getVelCov();

    std::list<KalmanRobot> list = wb.getRobotComponents();

    EXPECT_TRUE(wb.getIsValid());
    EXPECT_EQ(wb.getRobotID(), rID);
    EXPECT_EQ(rp.x(), pose.position().x());
    EXPECT_EQ(rp.y(), pose.position().y());
    EXPECT_EQ(rt, pose.heading());
    EXPECT_EQ(rv.x(), 0);
    EXPECT_EQ(rv.y(), 0);
    EXPECT_EQ(ro, 0);
    EXPECT_GT(rpc, 0);
    EXPECT_GT(rvc, 0);
    EXPECT_LT(rpc, 10000);
    EXPECT_LT(rvc, 10000);
    EXPECT_EQ(list.size(), 1);
}

TEST(WorldRobot, two_robot) {
    RJ::Time t = RJ::now();
    geometry2d::Pose pose1(geometry2d::Point(1, 1), 1);
    geometry2d::Pose pose2(geometry2d::Point(2, 2), 2);

    // geometry2d::Point p1 = geometry2d::Point(1,1);
    // geometry2d::Point p2 = geometry2d::Point(2,2);
    // double th1 = 1;
    // double th2 = 2;
    int rID = 1;
    CameraRobot b1 = CameraRobot(t, pose1, rID);
    CameraRobot b2 = CameraRobot(t, pose2, rID);
    int cID = 1;
    WorldRobot w;

    KalmanRobot kb1 = KalmanRobot(cID, t, b1, w);
    KalmanRobot kb2 = KalmanRobot(cID, t, b2, w);

    std::list<KalmanRobot> kbl;
    kbl.push_back(kb1);
    kbl.push_back(kb2);

    WorldRobot wb = WorldRobot(t, WorldRobot::Team::BLUE, rID, kbl);

    geometry2d::Point rp = wb.getPos();
    double rt = wb.getTheta();
    geometry2d::Point rv = wb.getVel();
    double ro = wb.getOmega();
    double rpc = wb.getPosCov();
    double rvc = wb.getVelCov();

    std::list<KalmanRobot> list = wb.getRobotComponents();

    EXPECT_TRUE(wb.getIsValid());
    EXPECT_EQ(rp.x(), (pose1.position().x() + pose2.position().x()) / 2);
    EXPECT_EQ(rp.y(), (pose1.position().y() + pose2.position().y()) / 2);
    EXPECT_EQ(rt, (pose1.heading() + pose2.heading()) / 2);
    EXPECT_EQ(rv.x(), 0);
    EXPECT_EQ(rv.y(), 0);
    EXPECT_EQ(ro, 0);
    EXPECT_GT(rpc, 0);
    EXPECT_GT(rvc, 0);
    EXPECT_LT(rpc, 10000);
    EXPECT_LT(rvc, 10000);
    EXPECT_EQ(list.size(), 2);
}
