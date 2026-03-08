#include <gtest/gtest.h>

#include <rj_vision_filter/robot/world_robot.hpp>

namespace vision_filter {

//NOLINTNEXTLINE
TEST(WorldRobot, invalid) {
    WorldRobot wb; //NOLINT(readability-identifier-length)

    EXPECT_FALSE(wb.get_is_valid());
}

//NOLINTNEXTLINE
TEST(WorldRobot, no_robot) {
    std::list<KalmanRobot> kbl;
    
    //NOLINTNEXTLINE
    EXPECT_ANY_THROW(WorldRobot(RJ::now(), WorldRobot::Team::BLUE, 1, kbl));
}

//NOLINTNEXTLINE
TEST(WorldRobot, one_robot) {
    RJ::Time t = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 1);
    int r_id = 1;
    CameraRobot b = CameraRobot(t, pose, r_id); //NOLINT(readability-identifier-length)
    int c_id = 1;
    WorldRobot w; //NOLINT(readability-identifier-length)

    KalmanRobot kb = KalmanRobot(c_id, t, b, w); //NOLINT(readability-identifier-length)

    std::list<KalmanRobot> kbl;
    kbl.push_back(kb);

    WorldRobot wb = WorldRobot(t, WorldRobot::Team::BLUE, r_id, kbl); //NOLINT(readability-identifier-length)

    rj_geometry::Point rp = wb.get_pos(); //NOLINT(readability-identifier-length)
    double rt = wb.get_theta(); //NOLINT(readability-identifier-length)
    rj_geometry::Point rv = wb.get_vel(); //NOLINT(readability-identifier-length)
    double ro = wb.get_omega(); //NOLINT(readability-identifier-length)

    double rpc = wb.get_pos_cov();
    double rvc = wb.get_vel_cov();

    const std::list<KalmanRobot>& list = wb.get_robot_components();

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

//NOLINTNEXTLINE
TEST(WorldRobot, two_robot) {
    RJ::Time t = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Pose pose1(rj_geometry::Point(1, 1), 1);
    rj_geometry::Pose pose2(rj_geometry::Point(2, 2), 2);

    // rj_geometry::Point p1 = rj_geometry::Point(1,1);
    // rj_geometry::Point p2 = rj_geometry::Point(2,2);
    // double th1 = 1;
    // double th2 = 2;
    int r_id = 1;
    CameraRobot b1 = CameraRobot(t, pose1, r_id); //NOLINT(readability-identifier-length)
    CameraRobot b2 = CameraRobot(t, pose2, r_id); //NOLINT(readability-identifier-length)
    int c_id = 1;
    WorldRobot w; //NOLINT(readability-identifier-length)

    KalmanRobot kb1 = KalmanRobot(c_id, t, b1, w);
    KalmanRobot kb2 = KalmanRobot(c_id, t, b2, w);

    std::list<KalmanRobot> kbl;
    kbl.push_back(kb1);
    kbl.push_back(kb2);

    WorldRobot wb = WorldRobot(t, WorldRobot::Team::BLUE, r_id, kbl); //NOLINT(readability-identifier-length)

    rj_geometry::Point rp = wb.get_pos(); //NOLINT(readability-identifier-length)
    double rt = wb.get_theta(); //NOLINT(readability-identifier-length)
    rj_geometry::Point rv = wb.get_vel(); //NOLINT(readability-identifier-length)
    double ro = wb.get_omega(); //NOLINT(readability-identifier-length)
    double rpc = wb.get_pos_cov();
    double rvc = wb.get_vel_cov();

    const std::list<KalmanRobot>& list = wb.get_robot_components();

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