#include <gtest/gtest.h>

#include <rj_vision_filter/ball/ball_bounce.hpp>
#include <rj_vision_filter/ball/camera_ball.hpp>
#include <rj_vision_filter/ball/kalman_ball.hpp>
#include <rj_vision_filter/ball/world_ball.hpp>

namespace vision_filter {

//NOLINTNEXTLINE
TEST(BallBounce, no_input) {
    RJ::Time tc = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Point p = rj_geometry::Point(0, 0); //NOLINT(readability-identifier-length)
    CameraBall cb = CameraBall(tc, p); //NOLINT(readability-identifier-length)
    WorldBall wb; //NOLINT(readability-identifier-length)
    KalmanBall kb = KalmanBall(1, tc, cb, wb); //NOLINT(readability-identifier-length)

    std::vector<WorldRobot> yellow;
    std::vector<WorldRobot> blue;

    rj_geometry::Point out_vel;

    bool is_bounce = BallBounce().calc_ball_bounce(kb, yellow, blue, out_vel);

    EXPECT_FALSE(is_bounce);
}

//NOLINTNEXTLINE
TEST(BallBounce, invalid_robot) {
    RJ::Time tc = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Point p1 = rj_geometry::Point(0, 0); //NOLINT(readability-identifier-length)
    CameraBall cb = CameraBall(tc, p1); //NOLINT(readability-identifier-length)
    WorldBall wb; //NOLINT(readability-identifier-length)
    KalmanBall kb = KalmanBall(1, tc, cb, wb); //NOLINT(readability-identifier-length)
    kb.set_vel(rj_geometry::Point(-1, 0));

    std::vector<WorldRobot> yellow;
    yellow.emplace_back(WorldRobot());
    std::vector<WorldRobot> blue;

    rj_geometry::Point out_vel;

    bool is_bounce = BallBounce().calc_ball_bounce(kb, yellow, blue, out_vel);

    EXPECT_FALSE(is_bounce);
}

//NOLINTNEXTLINE
TEST(BallBounce, no_intersection) {
    RJ::Time tc = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Point p1 = rj_geometry::Point(0, 0); //NOLINT(readability-identifier-length)
    CameraBall cb = CameraBall(tc, p1); //NOLINT(readability-identifier-length)
    WorldBall wb; //NOLINT(readability-identifier-length)
    KalmanBall kb = KalmanBall(1, tc, cb, wb); //NOLINT(readability-identifier-length)
    kb.set_vel(rj_geometry::Point(-1, 0));

    rj_geometry::Point p2 = rj_geometry::Point(1, 1); //NOLINT(readability-identifier-length)
    double th = 1; //NOLINT(readability-identifier-length)
    CameraRobot cr = CameraRobot(tc, rj_geometry::Pose(p2, th), 1); //NOLINT(readability-identifier-length)
    WorldRobot wr1;
    KalmanRobot kr = KalmanRobot(1, tc, cr, wr1); //NOLINT(readability-identifier-length)

    std::list<KalmanRobot> krl;
    krl.push_back(kr);

    WorldRobot wr2 = WorldRobot(tc, WorldRobot::Team::BLUE, 1, krl);

    std::vector<WorldRobot> yellow;
    yellow.push_back(wr2);
    std::vector<WorldRobot> blue;

    rj_geometry::Point out_vel;

    bool is_bounce = BallBounce().calc_ball_bounce(kb, yellow, blue, out_vel); 

    EXPECT_FALSE(is_bounce);
}

//NOLINTNEXTLINE
TEST(BallBounce, wrong_direction) {
    RJ::Time tc = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Point p1 = rj_geometry::Point(0, 0); //NOLINT(readability-identifier-length)
    CameraBall cb = CameraBall(tc, p1); //NOLINT(readability-identifier-length)
    WorldBall wb; //NOLINT(readability-identifier-length)
    KalmanBall kb = KalmanBall(1, tc, cb, wb); //NOLINT(readability-identifier-length)
    kb.set_vel(rj_geometry::Point(-1, 0));

    rj_geometry::Point p2 = rj_geometry::Point(1, 0); //NOLINT(readability-identifier-length)
    double th = 1; //NOLINT(readability-identifier-length)
    CameraRobot cr = CameraRobot(tc, rj_geometry::Pose(p2, th), 1); //NOLINT(readability-identifier-length)
    WorldRobot wr1;
    KalmanRobot kr = KalmanRobot(1, tc, cr, wr1); //NOLINT(readability-identifier-length)

    std::list<KalmanRobot> krl;
    krl.push_back(kr);

    WorldRobot wr2 = WorldRobot(tc, WorldRobot::Team::BLUE, 1, krl);

    std::vector<WorldRobot> yellow;
    yellow.push_back(wr2);
    std::vector<WorldRobot> blue;

    rj_geometry::Point out_vel;

    bool is_bounce = BallBounce().calc_ball_bounce(kb, yellow, blue, out_vel);

    EXPECT_FALSE(is_bounce);
}

//NOLINTNEXTLINE
TEST(BallBounce, too_far) {
    RJ::Time tc = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Point p1 = rj_geometry::Point(0, 0); //NOLINT(readability-identifier-length)
    CameraBall cb = CameraBall(tc, p1); //NOLINT(readability-identifier-length)
    WorldBall wb; //NOLINT(readability-identifier-length)
    KalmanBall kb = KalmanBall(1, tc, cb, wb); //NOLINT(readability-identifier-length)
    kb.set_vel(rj_geometry::Point(-1, 0));

    rj_geometry::Point p2 = rj_geometry::Point(-1, 0); //NOLINT(readability-identifier-length)
    double th = 1; //NOLINT(readability-identifier-length)
    CameraRobot cr = CameraRobot(tc, rj_geometry::Pose(p2, th), 1); //NOLINT(readability-identifier-length)
    WorldRobot wr1;
    KalmanRobot kr = KalmanRobot(1, tc, cr, wr1); //NOLINT(readability-identifier-length)

    std::list<KalmanRobot> krl;
    krl.push_back(kr);

    WorldRobot wr2 = WorldRobot(tc, WorldRobot::Team::BLUE, 1, krl);

    std::vector<WorldRobot> yellow;
    yellow.push_back(wr2);
    std::vector<WorldRobot> blue;

    rj_geometry::Point out_vel;

    bool is_bounce = BallBounce().calc_ball_bounce(kb, yellow, blue, out_vel);

    EXPECT_FALSE(is_bounce);
}

//NOLINTNEXTLINE
TEST(BallBounce, flat_intersect_side) {
    RJ::Time tc = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Point p1 = rj_geometry::Point(0, 0); //NOLINT(readability-identifier-length)
    CameraBall cb = CameraBall(tc, p1); //NOLINT(readability-identifier-length)
    WorldBall wb; //NOLINT(readability-identifier-length)
    KalmanBall kb = KalmanBall(1, tc, cb, wb); //NOLINT(readability-identifier-length)
    kb.set_vel(rj_geometry::Point(-1, 0));

    rj_geometry::Point p2 = rj_geometry::Point(-.1, 0); //NOLINT(readability-identifier-length)
    double th = 3.14 / 2; //NOLINT(readability-identifier-length)
    CameraRobot cr = CameraRobot(tc, rj_geometry::Pose(p2, th), 1); //NOLINT(readability-identifier-length)
    WorldRobot wr1;
    KalmanRobot kr = KalmanRobot(1, tc, cr, wr1); //NOLINT(readability-identifier-length)

    std::list<KalmanRobot> krl;
    krl.push_back(kr);

    WorldRobot wr2 = WorldRobot(tc, WorldRobot::Team::BLUE, 1, krl);

    std::vector<WorldRobot> yellow;
    yellow.push_back(wr2);
    std::vector<WorldRobot> blue;

    rj_geometry::Point out_vel;

    bool is_bounce = BallBounce().calc_ball_bounce(kb, yellow, blue, out_vel);

    EXPECT_TRUE(is_bounce);
    EXPECT_GT(out_vel.x(), 0);
    EXPECT_LT(out_vel.x(), 1.01);
    EXPECT_NEAR(out_vel.y(), 0, 0.01);
}

//NOLINTNEXTLINE
TEST(BallBounce, flat_intersect_mouth) {
    RJ::Time tc = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Point p1 = rj_geometry::Point(0, 0); //NOLINT(readability-identifier-length)
    CameraBall cb = CameraBall(tc, p1); //NOLINT(readability-identifier-length)
    WorldBall wb; //NOLINT(readability-identifier-length)
    KalmanBall kb = KalmanBall(1, tc, cb, wb); //NOLINT(readability-identifier-length)
    kb.set_vel(rj_geometry::Point(-1, 0));

    rj_geometry::Point p2 = rj_geometry::Point(-.1, 0); //NOLINT(readability-identifier-length)
    double th = 0; //NOLINT(readability-identifier-length)
    CameraRobot cr = CameraRobot(tc, rj_geometry::Pose(p2, th), 1); //NOLINT(readability-identifier-length)
    WorldRobot wr1;
    KalmanRobot kr = KalmanRobot(1, tc, cr, wr1); //NOLINT(readability-identifier-length)

    std::list<KalmanRobot> krl;
    krl.push_back(kr);

    WorldRobot wr2 = WorldRobot(tc, WorldRobot::Team::BLUE, 1, krl);

    std::vector<WorldRobot> yellow;
    yellow.push_back(wr2);
    std::vector<WorldRobot> blue;

    rj_geometry::Point out_vel;

    bool is_bounce = BallBounce().calc_ball_bounce(kb, yellow, blue, out_vel);

    EXPECT_TRUE(is_bounce);
    EXPECT_GT(out_vel.x(), 0);
    EXPECT_LT(out_vel.x(), 1.01);
    EXPECT_NEAR(out_vel.y(), 0, 0.01);
}

//NOLINTNEXTLINE
TEST(BallBounce, angle_intersect_side) {
    RJ::Time tc = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Point p1 = rj_geometry::Point(0, 0); //NOLINT(readability-identifier-length)
    CameraBall cb = CameraBall(tc, p1); //NOLINT(readability-identifier-length)
    WorldBall wb; //NOLINT(readability-identifier-length)
    KalmanBall kb = KalmanBall(1, tc, cb, wb); //NOLINT(readability-identifier-length)
    kb.set_vel(rj_geometry::Point(-1, 0));

    rj_geometry::Point p2 = rj_geometry::Point(-0.1, -0.06); //NOLINT(readability-identifier-length)
    double th = 3.14; //NOLINT(readability-identifier-length)
    CameraRobot cr = CameraRobot(tc, rj_geometry::Pose(p2, th), 1); //NOLINT(readability-identifier-length)
    WorldRobot wr1;
    KalmanRobot kr = KalmanRobot(1, tc, cr, wr1); //NOLINT(readability-identifier-length)

    std::list<KalmanRobot> krl;
    krl.push_back(kr);

    WorldRobot wr2 = WorldRobot(tc, WorldRobot::Team::BLUE, 1, krl);

    std::vector<WorldRobot> yellow;
    yellow.push_back(wr2);
    std::vector<WorldRobot> blue;

    rj_geometry::Point out_vel;

    bool is_bounce = BallBounce().calc_ball_bounce(kb, yellow, blue, out_vel);

    EXPECT_TRUE(is_bounce);
    EXPECT_GT(out_vel.x(), 0);
    EXPECT_LT(out_vel.x(), 1.0);
    EXPECT_GT(out_vel.y(), 0);
    EXPECT_LT(out_vel.y(), 1.0);
}

//NOLINTNEXTLINE
TEST(BallBounce, angle_intersect_mouth) {
    RJ::Time tc = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Point p1 = rj_geometry::Point(0, 0); //NOLINT(readability-identifier-length)
    CameraBall cb = CameraBall(tc, p1); //NOLINT(readability-identifier-length)
    WorldBall wb; //NOLINT(readability-identifier-length)
    KalmanBall kb = KalmanBall(1, tc, cb, wb); //NOLINT(readability-identifier-length)
    kb.set_vel(rj_geometry::Point(-1, 0));

    rj_geometry::Point p2 = rj_geometry::Point(-0.06, -0.04); //NOLINT(readability-identifier-length)
    double th = 1 * 3.14 / 4; //NOLINT(readability-identifier-length)
    CameraRobot cr = CameraRobot(tc, rj_geometry::Pose(p2, th), 1); //NOLINT(readability-identifier-length)
    WorldRobot wr1;
    KalmanRobot kr = KalmanRobot(1, tc, cr, wr1); //NOLINT(readability-identifier-length)

    std::list<KalmanRobot> krl;
    krl.push_back(kr);

    WorldRobot wr2 = WorldRobot(tc, WorldRobot::Team::BLUE, 1, krl);

    std::vector<WorldRobot> yellow;
    yellow.push_back(wr2);
    std::vector<WorldRobot> blue;

    rj_geometry::Point out_vel;

    bool is_bounce = BallBounce().calc_ball_bounce(kb, yellow, blue, out_vel);

    // Straight left with a 45 degree wall causes ball to go straight up
    EXPECT_TRUE(is_bounce);
    EXPECT_NEAR(out_vel.x(), 0, 0.1);
    EXPECT_NEAR(out_vel.y(), 1, 0.1);
}
}  // namespace vision_filter