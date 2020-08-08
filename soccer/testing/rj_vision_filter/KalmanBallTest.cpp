#include <gtest/gtest.h>

#include <rj_vision_filter/ball/KalmanBall.hpp>
#include <rj_vision_filter/ball/WorldBall.hpp>

namespace vision_filter {
TEST(KalmanBall, invalid_world_ball) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1, 1);
    CameraBall b = CameraBall(t, p);
    int c_id = 1;
    WorldBall w;

    KalmanBall kb = KalmanBall(c_id, t, b, w);

    Geometry2d::Point rv = kb.get_vel();
    Geometry2d::Point rp = kb.get_pos();

    EXPECT_EQ(rp.x(), p.x());
    EXPECT_EQ(rp.y(), p.y());
    EXPECT_EQ(rv.x(), 0);
    EXPECT_EQ(rv.y(), 0);
    EXPECT_FALSE(kb.is_unhealthy());
    EXPECT_EQ(kb.get_camera_id(), c_id);
    EXPECT_GT(kb.get_health(), 0);
}

TEST(KalmanBall, valid_world_ball) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1, 1);
    CameraBall b = CameraBall(t, p);
    int c_id = 1;
    WorldBall w;

    KalmanBall kb = KalmanBall(c_id, t, b, w);
    kb.set_vel(p);
    std::list<KalmanBall> kbl;
    kbl.push_back(kb);

    WorldBall wb = WorldBall(t, kbl);

    KalmanBall kb2 = KalmanBall(c_id, t, b, wb);

    Geometry2d::Point rv = kb2.get_vel();
    Geometry2d::Point rp = kb2.get_pos();

    EXPECT_EQ(rp.x(), p.x());
    EXPECT_EQ(rp.y(), p.y());
    EXPECT_EQ(rv.x(), p.x());
    EXPECT_EQ(rv.y(), p.y());
}

TEST(KalmanBall, predict) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1, 1);
    CameraBall b = CameraBall(t, p);
    int c_id = 1;
    WorldBall w;

    KalmanBall kb = KalmanBall(c_id, t, b, w);
    kb.set_vel(p);

    kb.predict(RJ::now());

    Geometry2d::Point rp = kb.get_pos();
    Geometry2d::Point rv = kb.get_vel();

    EXPECT_GT(rp.x(), p.x());
    EXPECT_GT(rp.y(), p.y());
    EXPECT_LT(rp.x(), p.x() + p.x() * 0.02);
    EXPECT_LT(rp.y(), p.y() + p.x() * 0.02);
    EXPECT_EQ(rv.x(), p.x());
    EXPECT_EQ(rv.y(), p.y());
}

TEST(KalmanBall, predict_and_update) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1, 1);
    CameraBall b = CameraBall(t, p);
    int c_id = 1;
    WorldBall w;

    KalmanBall kb = KalmanBall(c_id, t, b, w);
    kb.set_vel(p);

    kb.predict_and_update(RJ::now(), b);

    Geometry2d::Point rp = kb.get_pos();
    Geometry2d::Point rv = kb.get_vel();

    EXPECT_NEAR(rp.x(), p.x(), 0.1);
    EXPECT_NEAR(rp.y(), p.y(), 0.01);
    EXPECT_LT(rp.x(), p.x() + p.x() * 0.02);
    EXPECT_LT(rp.y(), p.y() + p.x() * 0.02);
    EXPECT_LT(rv.x(), p.x());
    EXPECT_LT(rv.y(), p.y());
}

TEST(KalmanBall, is_unhealthy) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1, 1);
    CameraBall b = CameraBall(t, p);
    int c_id = 1;
    WorldBall w;

    KalmanBall kb = KalmanBall(c_id, t, b, w);

    kb.predict(RJ::now() + RJ::Seconds(10));

    EXPECT_TRUE(kb.is_unhealthy());
}

TEST(KalmanBall, max_measurement_size) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1, 1);
    CameraBall b = CameraBall(t, p);
    int c_id = 1;
    WorldBall w;

    KalmanBall kb = KalmanBall(c_id, t, b, w);
    kb.set_vel(p);

    for (int i = 0; i < 100; i++) {
        kb.predict_and_update(RJ::now(), b);
    }

    boost::circular_buffer<CameraBall> list = kb.get_prev_measurements();

    EXPECT_LT(list.size(), 10);
}

TEST(KalmanBall, getters) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1, 1);
    CameraBall b = CameraBall(t, p);
    int c_id = 1;
    WorldBall w;

    KalmanBall kb = KalmanBall(c_id, t, b, w);

    Geometry2d::Point rpc = kb.get_pos_cov();
    Geometry2d::Point rvc = kb.get_vel_cov();
    Geometry2d::Point rp = kb.get_pos();
    Geometry2d::Point rv = kb.get_vel();

    kb.set_vel(p);
    Geometry2d::Point rv2 = kb.get_vel();

    boost::circular_buffer<CameraBall> list = kb.get_prev_measurements();

    EXPECT_EQ(kb.get_camera_id(), c_id);
    EXPECT_GT(kb.get_health(), 0);
    EXPECT_EQ(rp.x(), p.x());
    EXPECT_EQ(rp.y(), p.y());
    EXPECT_EQ(rv.x(), 0);
    EXPECT_EQ(rv.y(), 0);
    EXPECT_GT(rpc.x(), 0);
    EXPECT_GT(rpc.y(), 0);
    EXPECT_GT(rvc.x(), 0);
    EXPECT_GT(rvc.y(), 0);
    EXPECT_LT(rpc.x(), 10000);
    EXPECT_LT(rpc.y(), 10000);
    EXPECT_LT(rvc.x(), 10000);
    EXPECT_LT(rvc.y(), 10000);
    EXPECT_EQ(list.size(), 1);
    EXPECT_EQ(rv2.x(), p.x());
    EXPECT_EQ(rv2.y(), p.y());
}
}  // namespace vision_filter