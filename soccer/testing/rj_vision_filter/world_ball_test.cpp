#include <gtest/gtest.h>

#include <rj_vision_filter/ball/world_ball.hpp>

namespace vision_filter {
TEST(WorldBall, invalid) {
    WorldBall wb;

    EXPECT_FALSE(wb.get_is_valid());
}

TEST(WorldBall, no_ball) {
    std::list<KalmanBall> kbl;

    EXPECT_ANY_THROW(WorldBall(RJ::now(), kbl));
}

TEST(WorldBall, one_ball) {
    RJ::Time t = RJ::now();
    rj_geometry::Point p = rj_geometry::Point(1, 1);
    CameraBall b = CameraBall(t, p);
    int c_id = 1;
    WorldBall w;

    KalmanBall kb = KalmanBall(c_id, t, b, w);

    std::list<KalmanBall> kbl;
    kbl.push_back(kb);

    WorldBall wb = WorldBall(t, kbl);

    rj_geometry::Point rp = wb.get_pos();
    rj_geometry::Point rv = wb.get_vel();
    double rpc = wb.get_pos_cov();
    double rvc = wb.get_vel_cov();

    std::list<KalmanBall> list = wb.get_ball_components();

    EXPECT_TRUE(wb.get_is_valid());
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
    rj_geometry::Point p1 = rj_geometry::Point(1, 1);
    rj_geometry::Point p2 = rj_geometry::Point(2, 2);
    CameraBall b1 = CameraBall(t, p1);
    CameraBall b2 = CameraBall(t, p2);
    int c_id = 1;
    WorldBall w;

    KalmanBall kb1 = KalmanBall(c_id, t, b1, w);
    KalmanBall kb2 = KalmanBall(c_id, t, b2, w);

    std::list<KalmanBall> kbl;
    kbl.push_back(kb1);
    kbl.push_back(kb2);

    WorldBall wb = WorldBall(t, kbl);

    rj_geometry::Point rp = wb.get_pos();
    rj_geometry::Point rv = wb.get_vel();
    double rpc = wb.get_pos_cov();
    double rvc = wb.get_vel_cov();

    std::list<KalmanBall> list = wb.get_ball_components();

    EXPECT_TRUE(wb.get_is_valid());
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
