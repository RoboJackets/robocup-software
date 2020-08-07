#include <gtest/gtest.h>

#include <rj_vision_filter/ball/CameraBall.hpp>

namespace vision_filter {
TEST(CameraBall, get_time_captured) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1, 1);

    CameraBall b = CameraBall(t, p);

    RJ::Time r = b.get_time_captured();

    EXPECT_EQ(t, r);
}

TEST(CameraBall, get_pos) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1, 1);

    CameraBall b = CameraBall(t, p);

    Geometry2d::Point r = b.get_pos();

    EXPECT_EQ(p.x(), r.x());
    EXPECT_EQ(p.x(), r.x());
}

TEST(CameraBall, combine_zero) {
    std::vector<CameraBall> balls;

    CameraBall r = CameraBall::combine_balls(balls);

    EXPECT_EQ(r.get_pos().x(), 0);
    EXPECT_EQ(r.get_pos().y(), 0);
}

TEST(CameraBall, combine_one) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1, 1);

    std::vector<CameraBall> balls;
    balls.emplace_back(t, p);

    CameraBall r = CameraBall::combine_balls(balls);

    EXPECT_EQ(r.get_pos().x(), p.x());
    EXPECT_EQ(r.get_pos().y(), p.y());
    EXPECT_EQ(r.get_time_captured(), t);
}

TEST(CameraBall, combine_two) {
    RJ::Time t1 = RJ::now();
    Geometry2d::Point p1 = Geometry2d::Point(1, 1);

    RJ::Time t2 = t1;
    Geometry2d::Point p2 = Geometry2d::Point(2, 2);

    std::vector<CameraBall> balls;
    balls.emplace_back(t1, p1);
    balls.emplace_back(t2, p2);

    CameraBall r = CameraBall::combine_balls(balls);

    EXPECT_EQ(r.get_pos().x(), (p1.x() + p2.x()) / 2);
    EXPECT_EQ(r.get_pos().y(), (p1.y() + p2.y()) / 2);
    EXPECT_EQ(r.get_time_captured(), t1);
}
}  // namespace vision_filter