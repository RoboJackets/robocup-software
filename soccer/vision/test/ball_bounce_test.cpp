#include <gtest/gtest.h>

#include "vision/ball/ball_bounce.hpp"
#include "vision/ball/camera_ball.hpp"
#include "vision/ball/kalman_ball.hpp"
#include "vision/ball/world_ball.hpp"

namespace vision_filter {
TEST(BallBounce, no_input) {
    RJ::Time tc = RJ::now();
    rj_geometry::Point p = rj_geometry::Point(0, 0);
    CameraBall cb = CameraBall(tc, p);
    WorldBall wb;
    KalmanBall kb = KalmanBall(1, tc, cb, wb);

    std::vector<WorldRobot> yellow;
    std::vector<WorldRobot> blue;

    rj_geometry::Point out_vel;

    bool is_bounce = BallBounce::calc_ball_bounce(kb, yellow, blue, out_vel);

    EXPECT_FALSE(is_bounce);
}

TEST(BallBounce, invalid_robot) {
    RJ::Time tc = RJ::now();
    rj_geometry::Point p1 = rj_geometry::Point(0, 0);
    CameraBall cb = CameraBall(tc, p1);
    WorldBall wb;
    KalmanBall kb = KalmanBall(1, tc, cb, wb);
    kb.set_vel(rj_geometry::Point(-1, 0));

    std::vector<WorldRobot> yellow;
    yellow.push_back(WorldRobot());
    std::vector<WorldRobot> blue;

    rj_geometry::Point out_vel;

    bool is_bounce = BallBounce::calc_ball_bounce(kb, yellow, blue, out_vel);

    EXPECT_FALSE(is_bounce);
}

TEST(BallBounce, no_intersection) {
    RJ::Time tc = RJ::now();
    rj_geometry::Point p1 = rj_geometry::Point(0, 0);
    CameraBall cb = CameraBall(tc, p1);
    WorldBall wb;
    KalmanBall kb = KalmanBall(1, tc, cb, wb);
    kb.set_vel(rj_geometry::Point(-1, 0));

    rj_geometry::Point p2 = rj_geometry::Point(1, 1);
    double th = 1;
    CameraRobot cr = CameraRobot(tc, rj_geometry::Pose(p2, th), 1);
    WorldRobot wr1;
    KalmanRobot kr = KalmanRobot(1, tc, cr, wr1);

    std::list<KalmanRobot> krl;
    krl.push_back(kr);

    WorldRobot wr2 = WorldRobot(tc, WorldRobot::Team::BLUE, 1, krl);

    std::vector<WorldRobot> yellow;
    yellow.push_back(wr2);
    std::vector<WorldRobot> blue;

    rj_geometry::Point out_vel;

    bool is_bounce = BallBounce::calc_ball_bounce(kb, yellow, blue, out_vel);

    EXPECT_FALSE(is_bounce);
}

TEST(BallBounce, wrong_direction) {
    RJ::Time tc = RJ::now();
    rj_geometry::Point p1 = rj_geometry::Point(0, 0);
    CameraBall cb = CameraBall(tc, p1);
    WorldBall wb;
    KalmanBall kb = KalmanBall(1, tc, cb, wb);
    kb.set_vel(rj_geometry::Point(-1, 0));

    rj_geometry::Point p2 = rj_geometry::Point(1, 0);
    double th = 1;
    CameraRobot cr = CameraRobot(tc, rj_geometry::Pose(p2, th), 1);
    WorldRobot wr1;
    KalmanRobot kr = KalmanRobot(1, tc, cr, wr1);

    std::list<KalmanRobot> krl;
    krl.push_back(kr);

    WorldRobot wr2 = WorldRobot(tc, WorldRobot::Team::BLUE, 1, krl);

    std::vector<WorldRobot> yellow;
    yellow.push_back(wr2);
    std::vector<WorldRobot> blue;

    rj_geometry::Point out_vel;

    bool is_bounce = BallBounce::calc_ball_bounce(kb, yellow, blue, out_vel);

    EXPECT_FALSE(is_bounce);
}

TEST(BallBounce, too_far) {
    RJ::Time tc = RJ::now();
    rj_geometry::Point p1 = rj_geometry::Point(0, 0);
    CameraBall cb = CameraBall(tc, p1);
    WorldBall wb;
    KalmanBall kb = KalmanBall(1, tc, cb, wb);
    kb.set_vel(rj_geometry::Point(-1, 0));

    rj_geometry::Point p2 = rj_geometry::Point(-1, 0);
    double th = 1;
    CameraRobot cr = CameraRobot(tc, rj_geometry::Pose(p2, th), 1);
    WorldRobot wr1;
    KalmanRobot kr = KalmanRobot(1, tc, cr, wr1);

    std::list<KalmanRobot> krl;
    krl.push_back(kr);

    WorldRobot wr2 = WorldRobot(tc, WorldRobot::Team::BLUE, 1, krl);

    std::vector<WorldRobot> yellow;
    yellow.push_back(wr2);
    std::vector<WorldRobot> blue;

    rj_geometry::Point out_vel;

    bool is_bounce = BallBounce::calc_ball_bounce(kb, yellow, blue, out_vel);

    EXPECT_FALSE(is_bounce);
}

TEST(BallBounce, flat_intersect_side) {
    RJ::Time tc = RJ::now();
    rj_geometry::Point p1 = rj_geometry::Point(0, 0);
    CameraBall cb = CameraBall(tc, p1);
    WorldBall wb;
    KalmanBall kb = KalmanBall(1, tc, cb, wb);
    kb.set_vel(rj_geometry::Point(-1, 0));

    rj_geometry::Point p2 = rj_geometry::Point(-.1, 0);
    double th = 3.14 / 2;
    CameraRobot cr = CameraRobot(tc, rj_geometry::Pose(p2, th), 1);
    WorldRobot wr1;
    KalmanRobot kr = KalmanRobot(1, tc, cr, wr1);

    std::list<KalmanRobot> krl;
    krl.push_back(kr);

    WorldRobot wr2 = WorldRobot(tc, WorldRobot::Team::BLUE, 1, krl);

    std::vector<WorldRobot> yellow;
    yellow.push_back(wr2);
    std::vector<WorldRobot> blue;

    rj_geometry::Point out_vel;

    bool is_bounce = BallBounce::calc_ball_bounce(kb, yellow, blue, out_vel);

    EXPECT_TRUE(is_bounce);
    EXPECT_GT(out_vel.x(), 0);
    EXPECT_LT(out_vel.x(), 1.01);
    EXPECT_NEAR(out_vel.y(), 0, 0.01);
}

TEST(BallBounce, flat_intersect_mouth) {
    RJ::Time tc = RJ::now();
    rj_geometry::Point p1 = rj_geometry::Point(0, 0);
    CameraBall cb = CameraBall(tc, p1);
    WorldBall wb;
    KalmanBall kb = KalmanBall(1, tc, cb, wb);
    kb.set_vel(rj_geometry::Point(-1, 0));

    rj_geometry::Point p2 = rj_geometry::Point(-.1, 0);
    double th = 0;
    CameraRobot cr = CameraRobot(tc, rj_geometry::Pose(p2, th), 1);
    WorldRobot wr1;
    KalmanRobot kr = KalmanRobot(1, tc, cr, wr1);

    std::list<KalmanRobot> krl;
    krl.push_back(kr);

    WorldRobot wr2 = WorldRobot(tc, WorldRobot::Team::BLUE, 1, krl);

    std::vector<WorldRobot> yellow;
    yellow.push_back(wr2);
    std::vector<WorldRobot> blue;

    rj_geometry::Point out_vel;

    bool is_bounce = BallBounce::calc_ball_bounce(kb, yellow, blue, out_vel);

    EXPECT_TRUE(is_bounce);
    EXPECT_GT(out_vel.x(), 0);
    EXPECT_LT(out_vel.x(), 1.01);
    EXPECT_NEAR(out_vel.y(), 0, 0.01);
}

TEST(BallBounce, angle_intersect_side) {
    RJ::Time tc = RJ::now();
    rj_geometry::Point p1 = rj_geometry::Point(0, 0);
    CameraBall cb = CameraBall(tc, p1);
    WorldBall wb;
    KalmanBall kb = KalmanBall(1, tc, cb, wb);
    kb.set_vel(rj_geometry::Point(-1, 0));

    rj_geometry::Point p2 = rj_geometry::Point(-0.1, -0.06);
    double th = 3.14;
    CameraRobot cr = CameraRobot(tc, rj_geometry::Pose(p2, th), 1);
    WorldRobot wr1;
    KalmanRobot kr = KalmanRobot(1, tc, cr, wr1);

    std::list<KalmanRobot> krl;
    krl.push_back(kr);

    WorldRobot wr2 = WorldRobot(tc, WorldRobot::Team::BLUE, 1, krl);

    std::vector<WorldRobot> yellow;
    yellow.push_back(wr2);
    std::vector<WorldRobot> blue;

    rj_geometry::Point out_vel;

    bool is_bounce = BallBounce::calc_ball_bounce(kb, yellow, blue, out_vel);

    EXPECT_TRUE(is_bounce);
    EXPECT_GT(out_vel.x(), 0);
    EXPECT_LT(out_vel.x(), 1.0);
    EXPECT_GT(out_vel.y(), 0);
    EXPECT_LT(out_vel.y(), 1.0);
}

TEST(BallBounce, angle_intersect_mouth) {
    RJ::Time tc = RJ::now();
    rj_geometry::Point p1 = rj_geometry::Point(0, 0);
    CameraBall cb = CameraBall(tc, p1);
    WorldBall wb;
    KalmanBall kb = KalmanBall(1, tc, cb, wb);
    kb.set_vel(rj_geometry::Point(-1, 0));

    rj_geometry::Point p2 = rj_geometry::Point(-0.06, -0.04);
    double th = 1 * 3.14 / 4;
    CameraRobot cr = CameraRobot(tc, rj_geometry::Pose(p2, th), 1);
    WorldRobot wr1;
    KalmanRobot kr = KalmanRobot(1, tc, cr, wr1);

    std::list<KalmanRobot> krl;
    krl.push_back(kr);

    WorldRobot wr2 = WorldRobot(tc, WorldRobot::Team::BLUE, 1, krl);

    std::vector<WorldRobot> yellow;
    yellow.push_back(wr2);
    std::vector<WorldRobot> blue;

    rj_geometry::Point out_vel;

    bool is_bounce = BallBounce::calc_ball_bounce(kb, yellow, blue, out_vel);

    // Straight left with a 45 degree wall causes ball to go straight up
    EXPECT_TRUE(is_bounce);
    EXPECT_NEAR(out_vel.x(), 0, 0.1);
    EXPECT_NEAR(out_vel.y(), 1, 0.1);
}
}  // namespace vision_filter