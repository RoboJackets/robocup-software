#include <gtest/gtest.h>

#include <rj_vision_filter/robot/WorldRobot.hpp>

namespace vision_filter {
TEST(WorldRobot, invalid) {
    WorldRobot wb;

    EXPECT_FALSE(wb.get_is_valid());
}

TEST(WorldRobot, no_robot) {
    std::list<KalmanRobot> kbl;

    EXPECT_ANY_THROW(WorldRobot(RJ::now(), WorldRobot::Team::BLUE, 1, kbl));
}

TEST(WorldRobot, one_robot) {
    RJ::Time t = RJ::now();
    Geometry2d::Pose pose(Geometry2d::Point(1, 1), 1);
    int r_id = 1;
    CameraRobot b = CameraRobot(t, pose, r_id);
    int c_id = 1;
    WorldRobot w;

    KalmanRobot kb = KalmanRobot(c_id, t, b, w);

    std::list<KalmanRobot> kbl;
    kbl.push_back(kb);

    WorldRobot wb = WorldRobot(t, WorldRobot::Team::BLUE, r_id, kbl);

    Geometry2d::Point rp = wb.get_pos();
    double rt = wb.get_theta();
    Geometry2d::Point rv = wb.get_vel();
    double ro = wb.get_omega();

    double rpc = wb.get_pos_cov();
    double rvc = wb.get_vel_cov();

    std::list<KalmanRobot> list = wb.get_robot_components();

    EXPECT_TRUE(wb.get_is_valid());
    EXPECT_EQ(wb.get_robot_id(), r_id);
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
    Geometry2d::Pose pose1(Geometry2d::Point(1, 1), 1);
    Geometry2d::Pose pose2(Geometry2d::Point(2, 2), 2);

    // Geometry2d::Point p1 = Geometry2d::Point(1,1);
    // Geometry2d::Point p2 = Geometry2d::Point(2,2);
    // double th1 = 1;
    // double th2 = 2;
    int r_id = 1;
    CameraRobot b1 = CameraRobot(t, pose1, r_id);
    CameraRobot b2 = CameraRobot(t, pose2, r_id);
    int c_id = 1;
    WorldRobot w;

    KalmanRobot kb1 = KalmanRobot(c_id, t, b1, w);
    KalmanRobot kb2 = KalmanRobot(c_id, t, b2, w);

    std::list<KalmanRobot> kbl;
    kbl.push_back(kb1);
    kbl.push_back(kb2);

    WorldRobot wb = WorldRobot(t, WorldRobot::Team::BLUE, r_id, kbl);

    Geometry2d::Point rp = wb.get_pos();
    double rt = wb.get_theta();
    Geometry2d::Point rv = wb.get_vel();
    double ro = wb.get_omega();
    double rpc = wb.get_pos_cov();
    double rvc = wb.get_vel_cov();

    std::list<KalmanRobot> list = wb.get_robot_components();

    EXPECT_TRUE(wb.get_is_valid());
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
}  // namespace vision_filter