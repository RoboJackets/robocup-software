#include <gtest/gtest.h>
#include "NelderMead2D.hpp"
#include "NelderMead2DConfig.hpp"
#include <Geometry2d/Util.hpp>
#include <iostream>

static float evalFunction1(Geometry2d::Point p) {
    return -1 * sqrt(p.x() * p.x() + p.y() * p.y());
}

static float evalFunction2(Geometry2d::Point p) { return 1; }

TEST(NelderMead2D, execute) {
    std::function<float(Geometry2d::Point)> f = &evalFunction1;
    NelderMead2DConfig config(
        f, Geometry2d::Point(1, 1), Geometry2d::Point(1, 1),
        Geometry2d::Point(0.001, 0.001), 1, 2, .5, .5, 100, 0, 0);

    NelderMead2D nm(config);

    nm.execute();

    EXPECT_NEAR(nm.getValue(), 0, 0.001);
    EXPECT_NEAR(nm.getPoint().x(), 0, 0.001);
    EXPECT_NEAR(nm.getPoint().y(), 0, 0.001);
}

TEST(NelderMead2D, iteration_limit) {
    std::function<float(Geometry2d::Point)> f = &evalFunction2;
    NelderMead2DConfig config(
        f, Geometry2d::Point(1, 1), Geometry2d::Point(.0001, .0001),
        Geometry2d::Point(0.001, 0.001), 1, 2, .5, .5, 100, 0, 0);

    NelderMead2D nm(config);

    nm.execute();

    EXPECT_NEAR(nm.getValue(), 1, 0.001);
    EXPECT_NEAR(nm.getPoint().x(), 1, 0.001);
    EXPECT_NEAR(nm.getPoint().y(), 1, 0.001);
}

TEST(NelderMead2D, max_limit) {
    std::function<float(Geometry2d::Point)> f = &evalFunction1;
    NelderMead2DConfig config(
        f, Geometry2d::Point(1, 1), Geometry2d::Point(1, 1),
        Geometry2d::Point(0.001, 0.001), 1, 2, .5, .5, 100, 0, 0.1);

    NelderMead2D nm(config);

    nm.execute();

    EXPECT_NEAR(nm.getValue(), 0, 0.1);
    EXPECT_NEAR(nm.getPoint().x(), 0, 0.1);
    EXPECT_NEAR(nm.getPoint().y(), 0, 0.1);
}