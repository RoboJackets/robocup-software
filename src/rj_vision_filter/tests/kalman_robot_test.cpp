#include <cmath>

#include <gtest/gtest.h>

#include <rj_param_utils/vision/vision_params.hpp>
#include <rj_vision_filter/robot/kalman_robot.hpp>
#include <rj_vision_filter/robot/world_robot.hpp>

namespace vision_filter {

//NOLINTNEXTLINE
TEST(KalmanRobot, invalid_world_robot) {
    RJ::Time t = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 1);
    int robot_id = 1;

    CameraRobot b = CameraRobot(t, pose, robot_id); //NOLINT(readability-identifier-length)
    int c_id = 1;
    WorldRobot w; //NOLINT(readability-identifier-length)

    KalmanRobot kb = KalmanRobot(c_id, t, b, w); //NOLINT(readability-identifier-length)

    rj_geometry::Point rp = kb.get_pos(); //NOLINT(readability-identifier-length)
    rj_geometry::Point rv = kb.get_vel(); //NOLINT(readability-identifier-length)
    double om = kb.get_omega(); //NOLINT(readability-identifier-length)

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

//NOLINTNEXTLINE
TEST(KalmanRobot, valid_world_robot) {
    RJ::Time t = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 1);
    int robot_id = 1;

    CameraRobot b1 = CameraRobot(t, pose, robot_id); //NOLINT(readability-identifier-length)
    CameraRobot b2 = CameraRobot( //NOLINT(readability-identifier-length)
        t, rj_geometry::Pose(pose.position() + pose.position(), pose.heading() + pose.heading()),
        robot_id);
    int c_id = 1;
    WorldRobot w; //NOLINT(readability-identifier-length)

    KalmanRobot kb = KalmanRobot(c_id, t, b1, w); //NOLINT(readability-identifier-length)
    kb.predict_and_update(t, b2);

    std::list<KalmanRobot> kbl;
    kbl.push_back(kb);

    WorldRobot wb = WorldRobot(t, WorldRobot::Team::BLUE, robot_id, kbl); //NOLINT(readability-identifier-length)

    KalmanRobot kb2 = KalmanRobot(c_id, t, b1, wb);

    rj_geometry::Point rp = kb2.get_pos(); //NOLINT(readability-identifier-length)
    rj_geometry::Point rv = kb2.get_vel(); //NOLINT(readability-identifier-length)
    double th = kb2.get_theta(); //NOLINT(readability-identifier-length)
    double om = kb2.get_omega(); //NOLINT(readability-identifier-length)

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

//NOLINTNEXTLINE
TEST(KalmanRobot, predict) {
    RJ::Time t = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 1);
    int robot_id = 1;

    CameraRobot b1 = CameraRobot(t, pose, robot_id); //NOLINT(readability-identifier-length)
    CameraRobot b2 = CameraRobot( //NOLINT(readability-identifier-length)
        t, rj_geometry::Pose(pose.position() + pose.position(), pose.heading() + pose.heading()),
        robot_id);
    int c_id = 1;
    WorldRobot w; //NOLINT(readability-identifier-length)

    KalmanRobot kb = KalmanRobot(c_id, t, b1, w); //NOLINT(readability-identifier-length)
    kb.predict_and_update(t, b2);

    rj_geometry::Point rp = kb.get_pos(); //NOLINT(readability-identifier-length)
    rj_geometry::Point rv = kb.get_vel(); //NOLINT(readability-identifier-length)
    double th = kb.get_theta(); //NOLINT(readability-identifier-length)
    double om = kb.get_omega(); //NOLINT(readability-identifier-length)

    kb.predict(t);

    rj_geometry::Point rp2 = kb.get_pos();
    double th2 = kb.get_theta();

    EXPECT_NEAR(rp2.x(), rp.x() + rv.y() * 0.01, 0.01);
    EXPECT_NEAR(rp2.y(), rp.y() + rv.y() * 0.01, 0.01);
    EXPECT_NEAR(th2, th + om * 0.01, 0.01);
    EXPECT_FALSE(kb.is_unhealthy());
    EXPECT_EQ(kb.get_camera_id(), c_id);
    EXPECT_GT(kb.get_health(), 0);
}

//NOLINTNEXTLINE
TEST(KalmanRobot, predict_and_update) {
    RJ::Time t = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 1);
    int robot_id = 1;

    CameraRobot b1 = CameraRobot(t, pose, robot_id); //NOLINT(readability-identifier-length)
    CameraRobot b2 = CameraRobot( //NOLINT(readability-identifier-length)
        t, rj_geometry::Pose(pose.position() + pose.position(), pose.heading() + pose.heading()),
        robot_id);
    int c_id = 1;
    WorldRobot w; //NOLINT(readability-identifier-length)

    KalmanRobot kb = KalmanRobot(c_id, t, b1, w); //NOLINT(readability-identifier-length)
    kb.predict_and_update(t, b2);

    rj_geometry::Point rp = kb.get_pos(); //NOLINT(readability-identifier-length)
    rj_geometry::Point rv = kb.get_vel(); //NOLINT(readability-identifier-length)
    double th = kb.get_theta(); //NOLINT(readability-identifier-length)
    double om = kb.get_omega(); //NOLINT(readability-identifier-length)

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

//NOLINTNEXTLINE
TEST(KalmanRobot, is_unhealthy) {
    RJ::Time t = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 1);
    int robot_id = 1;

    CameraRobot b = CameraRobot(t, pose, robot_id); //NOLINT(readability-identifier-length)
    int c_id = 1;
    WorldRobot w; //NOLINT(readability-identifier-length)

    KalmanRobot kb = KalmanRobot(c_id, t, b, w); //NOLINT(readability-identifier-length)

    kb.predict(RJ::now() + RJ::Seconds(10));

    EXPECT_TRUE(kb.is_unhealthy());
}

//NOLINTNEXTLINE
TEST(KalmanRobot, max_measurement_size) {
    RJ::Time t = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 1);
    int robot_id = 1;

    CameraRobot b = CameraRobot(t, pose, robot_id); //NOLINT(readability-identifier-length)
    int c_id = 1;
    WorldRobot w; //NOLINT(readability-identifier-length)

    KalmanRobot kb = KalmanRobot(c_id, t, b, w); //NOLINT(readability-identifier-length)

    for (int i = 0; i < 100; i++) {
        kb.predict_and_update(RJ::now() + RJ::Seconds(10), b);
    }

    boost::circular_buffer<CameraRobot> list = kb.get_prev_measurements();

    EXPECT_LT(list.size(), 10);
}

//NOLINTNEXTLINE
TEST(KalmanRobot, getters) {
    RJ::Time t = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 1);
    int robot_id = 1;

    CameraRobot b = CameraRobot(t, pose, robot_id); //NOLINT(readability-identifier-length)
    int c_id = 1;
    WorldRobot w; //NOLINT(readability-identifier-length)

    KalmanRobot kb = KalmanRobot(c_id, t, b, w); //NOLINT(readability-identifier-length)

    rj_geometry::Point rpc = kb.get_pos_cov();
    double rtc = kb.get_theta_cov();
    rj_geometry::Point rvc = kb.get_vel_cov();
    double roc = kb.get_omega_cov();
    rj_geometry::Point rp = kb.get_pos(); //NOLINT(readability-identifier-length)
    double rt = kb.get_theta(); //NOLINT(readability-identifier-length)
    rj_geometry::Point rv = kb.get_vel(); //NOLINT(readability-identifier-length)
    double ro = kb.get_omega(); //NOLINT(readability-identifier-length)

    const boost::circular_buffer<CameraRobot>& list = kb.get_prev_measurements();

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

//NOLINTNEXTLINE
TEST(KalmanRobot, wrap_theta_up) {
    RJ::Time t = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 0);
    int robot_id = 1;

    CameraRobot b = CameraRobot(t, pose, robot_id); //NOLINT(readability-identifier-length)
    int c_id = 1;
    WorldRobot w; //NOLINT(readability-identifier-length)

    KalmanRobot kb = KalmanRobot(c_id, t, b, w); //NOLINT(readability-identifier-length)

    double ut = 0; //NOLINT(readability-identifier-length)
    for (int i = 0; i < 800; i++) {
        pose.heading() += 1 * PARAM_vision_loop_dt;
        ut += 1 * PARAM_vision_loop_dt;

        if (pose.heading() > M_PI) {
            pose.heading() -= 2 * M_PI;
        }

        pose.position() += rj_geometry::Point(1, 1) * PARAM_vision_loop_dt;

        b = CameraRobot(t, pose, robot_id);
        kb.predict_and_update(RJ::now() + RJ::Seconds(10), b);
    }

    double rt = kb.get_theta(); //NOLINT(readability-identifier-length)
    double ro = kb.get_omega(); //NOLINT(readability-identifier-length)
    EXPECT_NEAR(rt, ut, 0.01);
    EXPECT_NEAR(ro, 1, 0.01);

    rj_geometry::Point rp = kb.get_pos(); //NOLINT(readability-identifier-length)
    rj_geometry::Point rv = kb.get_vel(); //NOLINT(readability-identifier-length)
    EXPECT_NEAR(rp.x(), pose.position().x(), 0.01);
    EXPECT_NEAR(rp.y(), pose.position().y(), 0.01);
    EXPECT_NEAR(rv.x(), 1, 0.01);
    EXPECT_NEAR(rv.y(), 1, 0.01);
}

//NOLINTNEXTLINE
TEST(KalmanRobot, wrap_theta_down) {
    RJ::Time t = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Pose pose(rj_geometry::Point(1, 1), 0);

    int robot_id = 1;

    CameraRobot b = CameraRobot(t, pose, robot_id); //NOLINT(readability-identifier-length)
    int c_id = 1;
    WorldRobot w; //NOLINT(readability-identifier-length)

    KalmanRobot kb = KalmanRobot(c_id, t, b, w); //NOLINT(readability-identifier-length)

    double ut = 0; //NOLINT(readability-identifier-length)
    for (int i = 0; i < 800; i++) {
        pose.heading() -= 1.0 * PARAM_vision_loop_dt;
        ut -= 1.0 * PARAM_vision_loop_dt;

        if (pose.heading() < -M_PI) {
            pose.heading() += 2 * M_PI;
        }

        pose.position() -= rj_geometry::Point(1, 1) * PARAM_vision_loop_dt;

        b = CameraRobot(t, pose, robot_id);
        kb.predict_and_update(RJ::now() + RJ::Seconds(10), b);
    }

    double rt = kb.get_theta(); //NOLINT(readability-identifier-length)
    double ro = kb.get_omega(); //NOLINT(readability-identifier-length)
    EXPECT_NEAR(rt, ut, 0.01);
    EXPECT_NEAR(ro, -1, 0.01);

    rj_geometry::Point rp = kb.get_pos(); //NOLINT(readability-identifier-length)
    rj_geometry::Point rv = kb.get_vel(); //NOLINT(readability-identifier-length)
    EXPECT_NEAR(rp.x(), pose.position().x(), 0.01);
    EXPECT_NEAR(rp.y(), pose.position().y(), 0.01);
    EXPECT_NEAR(rv.x(), -1, 0.01);
    EXPECT_NEAR(rv.y(), -1, 0.01);
}
}  // namespace vision_filter