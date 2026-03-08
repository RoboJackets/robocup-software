#include "rj_vision_filter/ball/world_ball.hpp"

#include <gtest/gtest.h>

namespace vision_filter {

//NOLINTNEXTLINE
TEST(WorldBall, invalid) {
    WorldBall wb; //NOLINT(readability-identifier-length)

    EXPECT_FALSE(wb.get_is_valid());
}

//NOLINTNEXTLINE
TEST(WorldBall, no_ball) {
    std::list<KalmanBall> kbl;

    EXPECT_ANY_THROW(WorldBall(RJ::now(), kbl)); //NOLINT
}

//NOLINTNEXTLINE
TEST(WorldBall, one_ball) {
    RJ::Time t = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Point p = rj_geometry::Point(1, 1); //NOLINT(readability-identifier-length)
    CameraBall b = CameraBall(t, p); //NOLINT(readability-identifier-length)
    int c_id = 1;
    WorldBall w; //NOLINT(readability-identifier-length)

    KalmanBall kb = KalmanBall(c_id, t, b, w); //NOLINT(readability-identifier-length)

    std::list<KalmanBall> kbl;
    kbl.push_back(kb);

    WorldBall wb = WorldBall(t, kbl); //NOLINT(readability-identifier-length)

    rj_geometry::Point rp = wb.get_pos(); //NOLINT(readability-identifier-length)
    rj_geometry::Point rv = wb.get_vel(); //NOLINT(readability-identifier-length)
    double rpc = wb.get_pos_cov();
    double rvc = wb.get_vel_cov();

    const std::list<KalmanBall>& list = wb.get_ball_components();

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

//NOLINTNEXTLINE
TEST(WorldBall, two_ball) {
    RJ::Time t = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Point p1 = rj_geometry::Point(1, 1); //NOLINT(readability-identifier-length)
    rj_geometry::Point p2 = rj_geometry::Point(2, 2); //NOLINT(readability-identifier-length)
    CameraBall b1 = CameraBall(t, p1); //NOLINT(readability-identifier-length)
    CameraBall b2 = CameraBall(t, p2); //NOLINT(readability-identifier-length)
    int c_id = 1;
    WorldBall w; //NOLINT(readability-identifier-length)

    KalmanBall kb1 = KalmanBall(c_id, t, b1, w);
    KalmanBall kb2 = KalmanBall(c_id, t, b2, w);

    std::list<KalmanBall> kbl;
    kbl.push_back(kb1);
    kbl.push_back(kb2);

    WorldBall wb = WorldBall(t, kbl); //NOLINT(readability-identifier-length)

    rj_geometry::Point rp = wb.get_pos(); //NOLINT(readability-identifier-length)
    rj_geometry::Point rv = wb.get_vel(); //NOLINT(readability-identifier-length)
    double rpc = wb.get_pos_cov();
    double rvc = wb.get_vel_cov();

    const std::list<KalmanBall>& list = wb.get_ball_components();

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
