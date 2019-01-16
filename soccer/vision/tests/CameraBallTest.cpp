#include <gtest/gtest.h>
#include "vision/ball/CameraBall.hpp"

TEST(CameraBall, get_time_captured) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1,1);

    CameraBall b = CameraBall(t, p);

    RJ::Time r = b.getTimeCaptured();

    EXPECT_EQ(t, r);
}

TEST(CameraBall, get_pos) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1,1);

    CameraBall b = CameraBall(t, p);

    Geometry2d::Point r = b.getPos();

    EXPECT_EQ(p.x(), r.x());
    EXPECT_EQ(p.x(), r.x());
}

TEST(CameraBall, combine_zero) {
    std::vector<CameraBall> balls;

    CameraBall r = CameraBall::CombineBalls(balls);

    EXPECT_EQ(r.getPos().x(), 0);
    EXPECT_EQ(r.getPos().y(), 0);
}


TEST(CameraBall, combine_one) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1,1);

    std::vector<CameraBall> balls;
    balls.emplace_back(t, p);

    CameraBall r = CameraBall::CombineBalls(balls);

    EXPECT_EQ(r.getPos().x(), p.x());
    EXPECT_EQ(r.getPos().y(), p.y());
    EXPECT_EQ(r.getTimeCaptured(), t);
}

TEST(CameraBall, combine_two) {
    RJ::Time t1 = RJ::now();
    Geometry2d::Point p1 = Geometry2d::Point(1,1);

    RJ::Time t2 = t1;
    Geometry2d::Point p2 = Geometry2d::Point(2,2);

    std::vector<CameraBall> balls;
    balls.emplace_back(t1, p1);
    balls.emplace_back(t2, p2);

    CameraBall r = CameraBall::CombineBalls(balls);

    EXPECT_EQ(r.getPos().x(), (p1.x() + p2.x()) / 2);
    EXPECT_EQ(r.getPos().y(), (p1.y() + p2.y()) / 2);
    EXPECT_EQ(r.getTimeCaptured(), t1);
}