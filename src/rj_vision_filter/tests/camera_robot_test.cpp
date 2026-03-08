#include <gtest/gtest.h>

#include <rj_vision_filter/robot/camera_robot.hpp>

namespace vision_filter {

//NOLINTNEXTLINE
TEST(CameraRobot, get_time_captured) {
    RJ::Time t = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 1);
    int id = 0; //NOLINT(readability-identifier-length)

    CameraRobot b = CameraRobot(t, pose, id); //NOLINT(readability-identifier-length)

    RJ::Time r = b.get_time_captured(); //NOLINT(readability-identifier-length)

    EXPECT_EQ(t, r);
}

//NOLINTNEXTLINE
TEST(CameraRobot, get_pos) {
    RJ::Time t = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 1);
    int id = 0; //NOLINT(readability-identifier-length)

    CameraRobot b = CameraRobot(t, pose, id); //NOLINT(readability-identifier-length)

    rj_geometry::Point r = b.get_pos(); //NOLINT(readability-identifier-length)

    EXPECT_EQ(pose.position().x(), r.x());
    EXPECT_EQ(pose.position().x(), r.x());
}

//NOLINTNEXTLINE
TEST(CameraRobot, get_theta) {
    RJ::Time t = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 1);
    int id = 0; //NOLINT(readability-identifier-length)

    CameraRobot b = CameraRobot(t, pose, id); //NOLINT(readability-identifier-length)

    double r = b.get_theta(); //NOLINT(readability-identifier-length)

    EXPECT_EQ(pose.heading(), r);
}

//NOLINTNEXTLINE
TEST(CameraRobot, get_robot_id) {
    RJ::Time t = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 1);
    int id = 0; //NOLINT(readability-identifier-length)

    CameraRobot b = CameraRobot(t, pose, id); //NOLINT(readability-identifier-length)

    int r = b.get_robot_id(); //NOLINT(readability-identifier-length)

    EXPECT_EQ(id, r);
}

//NOLINTNEXTLINE
TEST(CameraRobot, combine_zero) {
    std::list<CameraRobot> robots;

    CameraRobot r = CameraRobot::combine_robots(robots); //NOLINT(readability-identifier-length)

    EXPECT_EQ(r.get_pos().x(), 0);
    EXPECT_EQ(r.get_pos().y(), 0);
    EXPECT_EQ(r.get_theta(), 0);
    EXPECT_EQ(r.get_robot_id(), -1);
}

//NOLINTNEXTLINE
TEST(CameraRobot, combine_one) {
    RJ::Time t = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 1);
    int id = 0; //NOLINT(readability-identifier-length)

    std::list<CameraRobot> robots;
    robots.emplace_back(t, pose, id);

    CameraRobot r = CameraRobot::combine_robots(robots); //NOLINT(readability-identifier-length)

    EXPECT_EQ(r.get_pos().x(), pose.position().x());
    EXPECT_EQ(r.get_pos().y(), pose.position().y());
    EXPECT_EQ(r.get_time_captured(), t);
    EXPECT_EQ(r.get_theta(), pose.heading());
    EXPECT_EQ(r.get_robot_id(), id);
}

//NOLINTNEXTLINE
TEST(CameraRobot, combine_two) {
    RJ::Time t1 = RJ::now(); //NOLINT(readability-identifier-length)
    RJ::Time t2 = t1; //NOLINT(readability-identifier-length)
    rj_geometry::Pose pose1(rj_geometry::Point(1, 1), 1);
    rj_geometry::Pose pose2(rj_geometry::Point(2, 2), 1.5);
    int id = 0; //NOLINT(readability-identifier-length)

    std::list<CameraRobot> robots;
    robots.emplace_back(t1, pose1, id);
    robots.emplace_back(t2, pose2, id);

    CameraRobot r = CameraRobot::combine_robots(robots); //NOLINT(readability-identifier-length)

    EXPECT_EQ(r.get_pos().x(), (pose1.position().x() + pose2.position().x()) / 2);
    EXPECT_EQ(r.get_pos().y(), (pose1.position().y() + pose2.position().y()) / 2);
    EXPECT_EQ(r.get_time_captured(), t1);
    EXPECT_EQ(r.get_theta(), (pose1.heading() + pose2.heading()) / 2);
    EXPECT_EQ(r.get_robot_id(), id);
}
}  // namespace vision_filter
