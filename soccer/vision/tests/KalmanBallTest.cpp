#include <gtest/gtest.h>
#include "vision/ball/KalmanBall.hpp"
#include "vision/ball/WorldBall.hpp"

TEST(KalmanBall, invalid_world_ball) {
    RJ::Time t = RJ::now();
    geometry2d::Point p = geometry2d::Point(1, 1);
    CameraBall b = CameraBall(t, p);
    int cID = 1;
    WorldBall w;

    KalmanBall kb = KalmanBall(cID, t, b, w);

    geometry2d::Point rv = kb.getVel();
    geometry2d::Point rp = kb.getPos();

    EXPECT_EQ(rp.x(), p.x());
    EXPECT_EQ(rp.y(), p.y());
    EXPECT_EQ(rv.x(), 0);
    EXPECT_EQ(rv.y(), 0);
    EXPECT_FALSE(kb.isUnhealthy());
    EXPECT_EQ(kb.getCameraID(), cID);
    EXPECT_GT(kb.getHealth(), 0);
}

TEST(KalmanBall, valid_world_ball) {
    RJ::Time t = RJ::now();
    geometry2d::Point p = geometry2d::Point(1, 1);
    CameraBall b = CameraBall(t, p);
    int cID = 1;
    WorldBall w;

    KalmanBall kb = KalmanBall(cID, t, b, w);
    kb.setVel(p);
    std::list<KalmanBall> kbl;
    kbl.push_back(kb);

    WorldBall wb = WorldBall(t, kbl);

    KalmanBall kb2 = KalmanBall(cID, t, b, wb);

    geometry2d::Point rv = kb2.getVel();
    geometry2d::Point rp = kb2.getPos();

    EXPECT_EQ(rp.x(), p.x());
    EXPECT_EQ(rp.y(), p.y());
    EXPECT_EQ(rv.x(), p.x());
    EXPECT_EQ(rv.y(), p.y());
}

TEST(KalmanBall, predict) {
    RJ::Time t = RJ::now();
    geometry2d::Point p = geometry2d::Point(1, 1);
    CameraBall b = CameraBall(t, p);
    int cID = 1;
    WorldBall w;

    KalmanBall kb = KalmanBall(cID, t, b, w);
    kb.setVel(p);

    kb.predict(RJ::now());

    geometry2d::Point rp = kb.getPos();
    geometry2d::Point rv = kb.getVel();

    EXPECT_GT(rp.x(), p.x());
    EXPECT_GT(rp.y(), p.y());
    EXPECT_LT(rp.x(), p.x() + p.x()*0.02);
    EXPECT_LT(rp.y(), p.y() + p.x()*0.02);
    EXPECT_EQ(rv.x(), p.x());
    EXPECT_EQ(rv.y(), p.y());
}

TEST(KalmanBall, predict_and_update) {
    RJ::Time t = RJ::now();
    geometry2d::Point p = geometry2d::Point(1, 1);
    CameraBall b = CameraBall(t, p);
    int cID = 1;
    WorldBall w;

    KalmanBall kb = KalmanBall(cID, t, b, w);
    kb.setVel(p);

    kb.predictAndUpdate(RJ::now(), b);

    geometry2d::Point rp = kb.getPos();
    geometry2d::Point rv = kb.getVel();

    EXPECT_NEAR(rp.x(), p.x(), 0.1);
    EXPECT_NEAR(rp.y(), p.y(), 0.01);
    EXPECT_LT(rp.x(), p.x() + p.x()*0.02);
    EXPECT_LT(rp.y(), p.y() + p.x()*0.02);
    EXPECT_LT(rv.x(), p.x());
    EXPECT_LT(rv.y(), p.y());
}

TEST(KalmanBall, is_unhealthy) {
    RJ::Time t = RJ::now();
    geometry2d::Point p = geometry2d::Point(1, 1);
    CameraBall b = CameraBall(t, p);
    int cID = 1;
    WorldBall w;

    KalmanBall kb = KalmanBall(cID, t, b, w);

    kb.predict(RJ::now() + RJ::Seconds(10));

    EXPECT_TRUE(kb.isUnhealthy());
}

TEST(KalmanBall, max_measurement_size) {
    RJ::Time t = RJ::now();
    geometry2d::Point p = geometry2d::Point(1, 1);
    CameraBall b = CameraBall(t, p);
    int cID = 1;
    WorldBall w;

    KalmanBall kb = KalmanBall(cID, t, b, w);
    kb.setVel(p);

    for (int i = 0; i < 100; i++) {
        kb.predictAndUpdate(RJ::now(), b);
    }

    boost::circular_buffer<CameraBall> list = kb.getPrevMeasurements();

    EXPECT_LT(list.size(), 10);
}

TEST(KalmanBall, getters) {
    RJ::Time t = RJ::now();
    geometry2d::Point p = geometry2d::Point(1, 1);
    CameraBall b = CameraBall(t, p);
    int cID = 1;
    WorldBall w;

    KalmanBall kb = KalmanBall(cID, t, b, w);

    geometry2d::Point rpc = kb.getPosCov();
    geometry2d::Point rvc = kb.getVelCov();
    geometry2d::Point rp = kb.getPos();
    geometry2d::Point rv = kb.getVel();

    kb.setVel(p);
    geometry2d::Point rv2 = kb.getVel();

    boost::circular_buffer<CameraBall> list = kb.getPrevMeasurements();

    EXPECT_EQ(kb.getCameraID(), cID);
    EXPECT_GT(kb.getHealth(), 0);
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
