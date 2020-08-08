#include <gtest/gtest.h>

#include <rj_constants/constants.hpp>
#include <rj_vision_filter/camera/Camera.hpp>

namespace vision_filter {
TEST(Camera, invalid_camera) {
    Camera c = Camera();

    EXPECT_FALSE(c.get_is_valid());
}

TEST(Camera, valid_camera) {
    Camera c = Camera(1);

    std::list<KalmanBall> kb = c.get_kalman_balls();
    std::vector<std::list<KalmanRobot>> kry = c.get_kalman_robots_yellow();
    std::vector<std::list<KalmanRobot>> krb = c.get_kalman_robots_blue();

    EXPECT_TRUE(c.get_is_valid());
    EXPECT_EQ(kb.size(), 0);
    EXPECT_GT(kry.size(), 0);
    EXPECT_GT(krb.size(), 0);
    EXPECT_EQ(kry.at(0).size(), 0);
    EXPECT_EQ(krb.at(0).size(), 0);
}

TEST(Camera, update_no_frame_empty) {
    Camera c = Camera(1);
    c.update_without_frame(RJ::now());

    std::list<KalmanBall> kb = c.get_kalman_balls();
    std::vector<std::list<KalmanRobot>> kry = c.get_kalman_robots_yellow();
    std::vector<std::list<KalmanRobot>> krb = c.get_kalman_robots_blue();

    EXPECT_TRUE(c.get_is_valid());
    EXPECT_EQ(kb.size(), 0);
    EXPECT_GT(kry.size(), 0);
    EXPECT_GT(krb.size(), 0);
    EXPECT_EQ(kry.at(0).size(), 0);
    EXPECT_EQ(krb.at(0).size(), 0);
}

TEST(Camera, update_with_frame_empty) {
    Camera c = Camera(1);
    RJ::Time t = RJ::now();

    std::vector<CameraBall> b;
    std::vector<std::list<CameraRobot>> yr(kNumShells);
    std::vector<std::list<CameraRobot>> br(kNumShells);
    WorldBall wb;
    std::vector<WorldRobot> wry(kNumShells, WorldRobot());
    std::vector<WorldRobot> wrb(kNumShells, WorldRobot());

    c.update_with_frame(t, b, yr, br, wb, wry, wrb);

    std::list<KalmanBall> kb = c.get_kalman_balls();
    std::vector<std::list<KalmanRobot>> kry = c.get_kalman_robots_yellow();
    std::vector<std::list<KalmanRobot>> krb = c.get_kalman_robots_blue();

    EXPECT_TRUE(c.get_is_valid());
    EXPECT_EQ(kb.size(), 0);
    EXPECT_GT(kry.size(), 0);
    EXPECT_GT(krb.size(), 0);
    EXPECT_EQ(kry.at(0).size(), 0);
    EXPECT_EQ(krb.at(0).size(), 0);
}

TEST(Camera, update_with_single_frame) {
    Camera c = Camera(1);
    RJ::Time t = RJ::now();

    std::vector<CameraBall> b;
    std::vector<std::list<CameraRobot>> yr(kNumShells);
    std::vector<std::list<CameraRobot>> br(kNumShells);
    WorldBall wb;
    std::vector<WorldRobot> wry(kNumShells, WorldRobot());
    std::vector<WorldRobot> wrb(kNumShells, WorldRobot());

    b.emplace_back(RJ::now(), Geometry2d::Point(0, 0));
    b.emplace_back(RJ::now(), Geometry2d::Point(0.5, 0.5));

    yr.at(0).emplace_back(RJ::now(), Geometry2d::Pose(Geometry2d::Point(1, 1), 0), 0);
    yr.at(0).emplace_back(RJ::now(), Geometry2d::Pose(Geometry2d::Point(1.5, 1.5), 0.5), 0);

    br.at(0).emplace_back(RJ::now(), Geometry2d::Pose(Geometry2d::Point(1.5, 1.5), 0.5), 0);

    c.update_with_frame(t, b, yr, br, wb, wry, wrb);

    std::list<KalmanBall> kb = c.get_kalman_balls();
    std::vector<std::list<KalmanRobot>> kry = c.get_kalman_robots_yellow();
    std::vector<std::list<KalmanRobot>> krb = c.get_kalman_robots_blue();

    EXPECT_TRUE(c.get_is_valid());
    EXPECT_EQ(kb.size(), 1);
    EXPECT_GT(kry.size(), 0);
    EXPECT_GT(krb.size(), 0);
    EXPECT_EQ(kry.at(0).size(), 1);
    EXPECT_EQ(krb.at(0).size(), 1);

    EXPECT_NEAR(kb.front().get_pos().x(), 0.25, 0.01);
    EXPECT_NEAR(kb.front().get_pos().y(), 0.25, 0.01);
    EXPECT_NEAR(kry.at(0).front().get_pos().y(), 1.25, 0.01);
    EXPECT_NEAR(kry.at(0).front().get_pos().y(), 1.25, 0.01);
    EXPECT_NEAR(kry.at(0).front().get_theta(), 0.25, 0.01);
}
}  // namespace vision_filter
