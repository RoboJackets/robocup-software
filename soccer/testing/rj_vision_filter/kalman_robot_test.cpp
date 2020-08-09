#include <cmath>

#include <gtest/gtest.h>

#include <rj_vision_filter/robot/kalman_robot.hpp>
#include <rj_vision_filter/robot/world_robot.hpp>

namespace vision_filter {
TEST(KalmanRobot, invalid_world_robot) {
    RJ::Time t = RJ::now();
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 1);
    int robot_id = 1;

    CameraRobot b = CameraRobot(t, pose, robot_id);
    int c_id = 1;
    WorldRobot w;

    KalmanRobot kb = KalmanRobot(c_id, t, b, w);

    rj_geometry::Point rp = kb.get_pos();
    rj_geometry::Point rv = kb.get_vel();
    double th = kb.get_theta();
    double om = kb.get_omega();

    EXPECT_EQ(rp.x(), pose.position().x());
    EXPECT_EQ(rp.y(), pose.position().y());
    EXPECT_EQ(pose.heading(), pose.heading());
    EXPECT_EQ(rv.x(), 0);
    EXPECT_EQ(rv.y(), 0);
    EXPECT_EQ(om, 0);
    EXPECT_FALSE(kb.is_unhealthy());
    EXPECT_EQ(kb.get_camera_id(), c_id);
    EXPECT_GT(kb.get_health(), 0);
}

TEST(KalmanRobot, valid_world_robot) {
    RJ::Time t = RJ::now();
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 1);
    int robot_id = 1;

    CameraRobot b1 = CameraRobot(t, pose, robot_id);
    CameraRobot b2 = CameraRobot(
        t, rj_geometry::Pose(pose.position() + pose.position(), pose.heading() + pose.heading()),
        robot_id);
    int c_id = 1;
    WorldRobot w;

    KalmanRobot kb = KalmanRobot(c_id, t, b1, w);
    kb.predict_and_update(t, b2);

    std::list<KalmanRobot> kbl;
    kbl.push_back(kb);

    WorldRobot wb = WorldRobot(t, WorldRobot::Team::BLUE, robot_id, kbl);

    KalmanRobot kb2 = KalmanRobot(c_id, t, b1, wb);

    rj_geometry::Point rp = kb2.get_pos();
    rj_geometry::Point rv = kb2.get_vel();
    double th = kb2.get_theta();
    double om = kb2.get_omega();

    EXPECT_EQ(rp.x(), pose.position().x());
    EXPECT_EQ(rp.y(), pose.position().y());
    EXPECT_EQ(th, pose.heading());
    EXPECT_GT(rv.x(), 0);
    EXPECT_GT(rv.y(), 0);
    EXPECT_GT(om, 0);
    EXPECT_LT(rv.x(), pose.position().x());
    EXPECT_LT(rv.y(), pose.position().y());
    EXPECT_LT(om, pose.heading());
}

TEST(KalmanRobot, predict) {
    RJ::Time t = RJ::now();
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 1);
    int robot_id = 1;

    CameraRobot b1 = CameraRobot(t, pose, robot_id);
    CameraRobot b2 = CameraRobot(
        t, rj_geometry::Pose(pose.position() + pose.position(), pose.heading() + pose.heading()),
        robot_id);
    int c_id = 1;
    WorldRobot w;

    KalmanRobot kb = KalmanRobot(c_id, t, b1, w);
    kb.predict_and_update(t, b2);

    rj_geometry::Point rp = kb.get_pos();
    rj_geometry::Point rv = kb.get_vel();
    double th = kb.get_theta();
    double om = kb.get_omega();

    kb.predict(t);

    rj_geometry::Point rp2 = kb.get_pos();
    rj_geometry::Point rv2 = kb.get_vel();
    double th2 = kb.get_theta();
    double om2 = kb.get_omega();

    EXPECT_NEAR(rp2.x(), rp.x() + rv.y() * 0.01, 0.01);
    EXPECT_NEAR(rp2.y(), rp.y() + rv.y() * 0.01, 0.01);
    EXPECT_NEAR(th2, th + om * 0.01, 0.01);
    EXPECT_FALSE(kb.is_unhealthy());
    EXPECT_EQ(kb.get_camera_id(), c_id);
    EXPECT_GT(kb.get_health(), 0);
}

TEST(KalmanRobot, predict_and_update) {
    RJ::Time t = RJ::now();
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 1);
    int robot_id = 1;

    CameraRobot b1 = CameraRobot(t, pose, robot_id);
    CameraRobot b2 = CameraRobot(
        t, rj_geometry::Pose(pose.position() + pose.position(), pose.heading() + pose.heading()),
        robot_id);
    int c_id = 1;
    WorldRobot w;

    KalmanRobot kb = KalmanRobot(c_id, t, b1, w);
    kb.predict_and_update(t, b2);

    rj_geometry::Point rp = kb.get_pos();
    rj_geometry::Point rv = kb.get_vel();
    double th = kb.get_theta();
    double om = kb.get_omega();

    EXPECT_NEAR(rp.x(), pose.position().x() * 2, 0.1);
    EXPECT_NEAR(rp.y(), pose.position().y() * 2, 0.1);
    EXPECT_NEAR(th, pose.heading() * 2, 0.1);
    EXPECT_GT(rv.x(), 0);
    EXPECT_GT(rv.y(), 0);
    EXPECT_GT(om, 0);
    EXPECT_LT(rv.x(), pose.position().x() / .01);
    EXPECT_LT(rv.y(), pose.position().y() / .01);
    EXPECT_LT(om, th / 0.01);
    EXPECT_FALSE(kb.is_unhealthy());
    EXPECT_EQ(kb.get_camera_id(), c_id);
    EXPECT_GT(kb.get_health(), 0);
}

TEST(KalmanRobot, is_unhealthy) {
    RJ::Time t = RJ::now();
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 1);
    int robot_id = 1;

    CameraRobot b = CameraRobot(t, pose, robot_id);
    int c_id = 1;
    WorldRobot w;

    KalmanRobot kb = KalmanRobot(c_id, t, b, w);

    kb.predict(RJ::now() + RJ::Seconds(10));

    EXPECT_TRUE(kb.is_unhealthy());
}

TEST(KalmanRobot, max_measurement_size) {
    RJ::Time t = RJ::now();
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 1);
    int robot_id = 1;

    CameraRobot b = CameraRobot(t, pose, robot_id);
    int c_id = 1;
    WorldRobot w;

    KalmanRobot kb = KalmanRobot(c_id, t, b, w);

    for (int i = 0; i < 100; i++) {
        kb.predict_and_update(RJ::now() + RJ::Seconds(10), b);
    }

    boost::circular_buffer<CameraRobot> list = kb.get_prev_measurements();

    EXPECT_LT(list.size(), 10);
}

TEST(KalmanRobot, getters) {
    RJ::Time t = RJ::now();
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 1);
    int robot_id = 1;

    CameraRobot b = CameraRobot(t, pose, robot_id);
    int c_id = 1;
    WorldRobot w;

    KalmanRobot kb = KalmanRobot(c_id, t, b, w);

    rj_geometry::Point rpc = kb.get_pos_cov();
    double rtc = kb.get_theta_cov();
    rj_geometry::Point rvc = kb.get_vel_cov();
    double roc = kb.get_omega_cov();
    rj_geometry::Point rp = kb.get_pos();
    double rt = kb.get_theta();
    rj_geometry::Point rv = kb.get_vel();
    double ro = kb.get_omega();

    boost::circular_buffer<CameraRobot> list = kb.get_prev_measurements();

    EXPECT_EQ(kb.get_camera_id(), c_id);
    EXPECT_GT(kb.get_health(), 0);
    EXPECT_EQ(rp.x(), pose.position().x());
    EXPECT_EQ(rp.y(), pose.position().y());
    EXPECT_EQ(rt, pose.heading());
    EXPECT_EQ(rv.x(), 0);
    EXPECT_EQ(rv.y(), 0);
    EXPECT_EQ(ro, 0);
    EXPECT_GT(rpc.x(), 0);
    EXPECT_GT(rpc.y(), 0);
    EXPECT_GT(rtc, 0);
    EXPECT_GT(rvc.x(), 0);
    EXPECT_GT(rvc.y(), 0);
    EXPECT_GT(roc, 0);
    EXPECT_LT(rpc.x(), 10000);
    EXPECT_LT(rpc.y(), 10000);
    EXPECT_LT(rtc, 10000);
    EXPECT_LT(rvc.x(), 10000);
    EXPECT_LT(rvc.y(), 10000);
    EXPECT_LT(roc, 10000);
    EXPECT_EQ(list.size(), 1);
}

TEST(KalmanRobot, wrap_theta_up) {
    RJ::Time t = RJ::now();
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 0);
    int robot_id = 1;

    CameraRobot b = CameraRobot(t, pose, robot_id);
    int c_id = 1;
    WorldRobot w;

    KalmanRobot kb = KalmanRobot(c_id, t, b, w);

    double ut = 0;
    for (int i = 0; i < 800; i++) {
        pose.heading() += 1.0 / 100.0;
        ut += 1.0 / 100.0;

        if (pose.heading() > M_PI) {
            pose.heading() -= 2 * M_PI;
        }

        pose.position() += rj_geometry::Point(1, 1) * 1.0 / 100.0;

        b = CameraRobot(t, pose, robot_id);
        kb.predict_and_update(RJ::now() + RJ::Seconds(10), b);
    }

    double rt = kb.get_theta();
    double ro = kb.get_omega();
    EXPECT_NEAR(rt, ut, 0.01);
    EXPECT_NEAR(ro, 1, 0.01);

    rj_geometry::Point rp = kb.get_pos();
    rj_geometry::Point rv = kb.get_vel();
    EXPECT_NEAR(rp.x(), pose.position().x(), 0.01);
    EXPECT_NEAR(rp.y(), pose.position().y(), 0.01);
    EXPECT_NEAR(rv.x(), 1, 0.01);
    EXPECT_NEAR(rv.y(), 1, 0.01);
}

TEST(KalmanRobot, wrap_theta_down) {
    RJ::Time t = RJ::now();
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 0);

    int robot_id = 1;

    CameraRobot b = CameraRobot(t, pose, robot_id);
    int c_id = 1;
    WorldRobot w;

    KalmanRobot kb = KalmanRobot(c_id, t, b, w);

    double ut = 0;
    for (int i = 0; i < 800; i++) {
        pose.heading() -= 1.0 / 100.0;
        ut -= 1.0 / 100.0;

        if (pose.heading() < -M_PI) {
            pose.heading() += 2 * M_PI;
        }

        pose.position() -= rj_geometry::Point(1, 1) * 1.0 / 100.0;

        b = CameraRobot(t, pose, robot_id);
        kb.predict_and_update(RJ::now() + RJ::Seconds(10), b);
    }

    double rt = kb.get_theta();
    double ro = kb.get_omega();
    EXPECT_NEAR(rt, ut, 0.01);
    EXPECT_NEAR(ro, -1, 0.01);

    rj_geometry::Point rp = kb.get_pos();
    rj_geometry::Point rv = kb.get_vel();
    EXPECT_NEAR(rp.x(), pose.position().x(), 0.01);
    EXPECT_NEAR(rp.y(), pose.position().y(), 0.01);
    EXPECT_NEAR(rv.x(), -1, 0.01);
    EXPECT_NEAR(rv.y(), -1, 0.01);
}
}  // namespace vision_filter