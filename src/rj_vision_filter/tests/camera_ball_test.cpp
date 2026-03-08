#include <gtest/gtest.h>

#include <rj_vision_filter/ball/camera_ball.hpp>

namespace vision_filter {

//NOLINTNEXTLINE
TEST(CameraBall, get_time_captured) {
    RJ::Time t = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Point p = rj_geometry::Point(1, 1); //NOLINT(readability-identifier-length)

    CameraBall b = CameraBall(t, p); //NOLINT(readability-identifier-length)

    RJ::Time r = b.get_time_captured(); //NOLINT(readability-identifier-length)

    EXPECT_EQ(t, r);
}

//NOLINTNEXTLINE
TEST(CameraBall, get_pos) {
    RJ::Time t = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Point p = rj_geometry::Point(1, 1); //NOLINT(readability-identifier-length)

    CameraBall b = CameraBall(t, p); //NOLINT(readability-identifier-length)

    rj_geometry::Point r = b.get_pos(); //NOLINT(readability-identifier-length)

    EXPECT_EQ(p.x(), r.x());
    EXPECT_EQ(p.x(), r.x());
}

//NOLINTNEXTLINE
TEST(CameraBall, combine_zero) {
    std::vector<CameraBall> balls;

    CameraBall r = CameraBall::combine_balls(balls); //NOLINT(readability-identifier-length)

    EXPECT_EQ(r.get_pos().x(), 0);
    EXPECT_EQ(r.get_pos().y(), 0);
}

//NOLINTNEXTLINE
TEST(CameraBall, combine_one) {
    RJ::Time t = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Point p = rj_geometry::Point(1, 1); //NOLINT(readability-identifier-length)

    std::vector<CameraBall> balls;
    balls.emplace_back(t, p);

    CameraBall r = CameraBall::combine_balls(balls); //NOLINT(readability-identifier-length)

    EXPECT_EQ(r.get_pos().x(), p.x());
    EXPECT_EQ(r.get_pos().y(), p.y());
    EXPECT_EQ(r.get_time_captured(), t);
}

//NOLINTNEXTLINE
TEST(CameraBall, combine_two) {
    RJ::Time t1 = RJ::now(); //NOLINT(readability-identifier-length)
    rj_geometry::Point p1 = rj_geometry::Point(1, 1); //NOLINT(readability-identifier-length)

    RJ::Time t2 = t1; //NOLINT(readability-identifier-length)
    rj_geometry::Point p2 = rj_geometry::Point(2, 2); //NOLINT(readability-identifier-length)

    std::vector<CameraBall> balls;
    balls.emplace_back(t1, p1);
    balls.emplace_back(t2, p2);

    CameraBall r = CameraBall::combine_balls(balls); //NOLINT(readability-identifier-length)

    EXPECT_EQ(r.get_pos().x(), (p1.x() + p2.x()) / 2);
    EXPECT_EQ(r.get_pos().y(), (p1.y() + p2.y()) / 2);
    EXPECT_EQ(r.get_time_captured(), t1);
}
}  // namespace vision_filter