#include <gtest/gtest.h>
#include <Constants.hpp>
#include "vision/camera/Camera.hpp"

TEST(Camera, invalid_camera) {
    Camera c = Camera();

    EXPECT_FALSE(c.getIsValid());
}


TEST(Camera, valid_camera) {
    Camera c = Camera(1);

    std::list<KalmanBall> kb = c.getKalmanBalls();
    std::vector<std::list<KalmanRobot>> kry = c.getKalmanRobotsYellow();
    std::vector<std::list<KalmanRobot>> krb = c.getKalmanRobotsBlue();

    EXPECT_TRUE(c.getIsValid());
    EXPECT_EQ(kb.size(), 0);
    EXPECT_GT(kry.size(), 0);
    EXPECT_GT(krb.size(), 0);
    EXPECT_EQ(kry.at(0).size(), 0);
    EXPECT_EQ(krb.at(0).size(), 0);
}

TEST(Camera, update_no_frame_empty) {
    Camera c = Camera(1);
    c.updateWithoutFrame(RJ::now());

    std::list<KalmanBall> kb = c.getKalmanBalls();
    std::vector<std::list<KalmanRobot>> kry = c.getKalmanRobotsYellow();
    std::vector<std::list<KalmanRobot>> krb = c.getKalmanRobotsBlue();

    EXPECT_TRUE(c.getIsValid());
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
    std::vector<std::list<CameraRobot>> yr(Num_Shells);
    std::vector<std::list<CameraRobot>> br(Num_Shells);
    WorldBall wb;
    std::vector<WorldRobot> wry(Num_Shells, WorldRobot());
    std::vector<WorldRobot> wrb(Num_Shells, WorldRobot());

    c.updateWithFrame(t, b, yr, br, wb, wry, wrb);

    std::list<KalmanBall> kb = c.getKalmanBalls();
    std::vector<std::list<KalmanRobot>> kry = c.getKalmanRobotsYellow();
    std::vector<std::list<KalmanRobot>> krb = c.getKalmanRobotsBlue();

    EXPECT_TRUE(c.getIsValid());
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
    std::vector<std::list<CameraRobot>> yr(Num_Shells);
    std::vector<std::list<CameraRobot>> br(Num_Shells);
    WorldBall wb;
    std::vector<WorldRobot> wry(Num_Shells, WorldRobot());
    std::vector<WorldRobot> wrb(Num_Shells, WorldRobot());

    b.emplace_back(RJ::now(), Geometry2d::Point(0, 0));
    b.emplace_back(RJ::now(), Geometry2d::Point(0.5, 0.5));

    yr.at(0).emplace_back(RJ::now(),
                          Geometry2d::Pose(Geometry2d::Point(1, 1), 0), 0);
    yr.at(0).emplace_back(
        RJ::now(), Geometry2d::Pose(Geometry2d::Point(1.5, 1.5), 0.5), 0);

    br.at(0).emplace_back(
        RJ::now(), Geometry2d::Pose(Geometry2d::Point(1.5, 1.5), 0.5), 0);

    c.updateWithFrame(t, b, yr, br, wb, wry, wrb);

    std::list<KalmanBall> kb = c.getKalmanBalls();
    std::vector<std::list<KalmanRobot>> kry = c.getKalmanRobotsYellow();
    std::vector<std::list<KalmanRobot>> krb = c.getKalmanRobotsBlue();

    EXPECT_TRUE(c.getIsValid());
    EXPECT_EQ(kb.size(), 1);
    EXPECT_GT(kry.size(), 0);
    EXPECT_GT(krb.size(), 0);
    EXPECT_EQ(kry.at(0).size(), 1);
    EXPECT_EQ(krb.at(0).size(), 1);

    EXPECT_NEAR(kb.front().getPos().x(), 0.25, 0.01);
    EXPECT_NEAR(kb.front().getPos().y(), 0.25, 0.01);
    EXPECT_NEAR(kry.at(0).front().getPos().y(), 1.25, 0.01);
    EXPECT_NEAR(kry.at(0).front().getPos().y(), 1.25, 0.01);
    EXPECT_NEAR(kry.at(0).front().getTheta(), 0.25, 0.01);
}