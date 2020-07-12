#include <gtest/gtest.h>

#include <rj_vision_filter/ball/WorldBall.hpp>

namespace vision_filter {
TEST(WorldBall, invalid) {
    WorldBall wb;

    EXPECT_FALSE(wb.getIsValid());
}

TEST(WorldBall, no_ball) {
    std::list<KalmanBall> kbl;

    WorldBall wb = WorldBall(RJ::now(), kbl);

    EXPECT_FALSE(wb.getIsValid());
}

TEST(WorldBall, one_ball) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1, 1);
    CameraBall b = CameraBall(t, p);
    int cID = 1;
    WorldBall w;

    KalmanBall kb = KalmanBall(cID, t, b, w);

    std::list<KalmanBall> kbl;
    kbl.push_back(kb);

    WorldBall wb = WorldBall(t, kbl);

    Geometry2d::Point rp = wb.getPos();
    Geometry2d::Point rv = wb.getVel();
    double rpc = wb.getPosCov();
    double rvc = wb.getVelCov();

    std::list<KalmanBall> list = wb.getBallComponents();

    EXPECT_TRUE(wb.getIsValid());
    EXPECT_EQ(rp.x(), p.x());
    EXPECT_EQ(rp.y(), p.y());
    EXPECT_EQ(rv.x(), 0);
    EXPECT_EQ(rv.y(), 0);
    EXPECT_GT(rpc, 0);
    EXPECT_GT(rvc, 0);
    EXPECT_LT(rpc, 10000);
    EXPECT_LT(rvc, 10000);
    EXPECT_EQ(list.size(), 1);
}

TEST(WorldBall, two_ball) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p1 = Geometry2d::Point(1, 1);
    Geometry2d::Point p2 = Geometry2d::Point(2, 2);
    CameraBall b1 = CameraBall(t, p1);
    CameraBall b2 = CameraBall(t, p2);
    int cID = 1;
    WorldBall w;

    KalmanBall kb1 = KalmanBall(cID, t, b1, w);
    KalmanBall kb2 = KalmanBall(cID, t, b2, w);

    std::list<KalmanBall> kbl;
    kbl.push_back(kb1);
    kbl.push_back(kb2);

    WorldBall wb = WorldBall(t, kbl);

    Geometry2d::Point rp = wb.getPos();
    Geometry2d::Point rv = wb.getVel();
    double rpc = wb.getPosCov();
    double rvc = wb.getVelCov();

    std::list<KalmanBall> list = wb.getBallComponents();

    EXPECT_TRUE(wb.getIsValid());
    EXPECT_EQ(rp.x(), (p1.x() + p2.x()) / 2);
    EXPECT_EQ(rp.y(), (p1.y() + p2.y()) / 2);
    EXPECT_EQ(rv.x(), 0);
    EXPECT_EQ(rv.y(), 0);
    EXPECT_GT(rpc, 0);
    EXPECT_GT(rvc, 0);
    EXPECT_LT(rpc, 10000);
    EXPECT_LT(rvc, 10000);
    EXPECT_EQ(list.size(), 2);
}
}  // namespace vision_filter
