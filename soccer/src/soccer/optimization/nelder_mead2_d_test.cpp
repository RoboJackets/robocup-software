#include <iostream>

#include <gtest/gtest.h>

#include <rj_geometry/util.hpp>

#include "nelder_mead2_d.hpp"
#include "nelder_mead2_d_config.hpp"

static float eval_function1(rj_geometry::Point p) {
    return -1 * sqrt(p.x() * p.x() + p.y() * p.y());
}

static float eval_function2(rj_geometry::Point p) { return 1; }

TEST(NelderMead2D, execute) {
    std::function<float(rj_geometry::Point)> f = &eval_function1;
    NelderMead2DConfig config(f, rj_geometry::Point(1, 1), rj_geometry::Point(1, 1),
                              rj_geometry::Point(0.001, 0.001), 1, 2, .5, .5, 100, 0, 0);

    NelderMead2D nm(config);

    nm.execute();

    EXPECT_NEAR(nm.get_value(), 0, 0.001);
    EXPECT_NEAR(nm.get_point().x(), 0, 0.001);
    EXPECT_NEAR(nm.get_point().y(), 0, 0.001);
}

TEST(NelderMead2D, iteration_limit) {
    std::function<float(rj_geometry::Point)> f = &eval_function2;
    NelderMead2DConfig config(f, rj_geometry::Point(1, 1), rj_geometry::Point(.0001, .0001),
                              rj_geometry::Point(0.001, 0.001), 1, 2, .5, .5, 100, 0, 0);

    NelderMead2D nm(config);

    nm.execute();

    EXPECT_NEAR(nm.get_value(), 1, 0.001);
    EXPECT_NEAR(nm.get_point().x(), 1, 0.001);
    EXPECT_NEAR(nm.get_point().y(), 1, 0.001);
}

TEST(NelderMead2D, max_limit) {
    std::function<float(rj_geometry::Point)> f = &eval_function1;
    NelderMead2DConfig config(f, rj_geometry::Point(1, 1), rj_geometry::Point(1, 1),
                              rj_geometry::Point(0.001, 0.001), 1, 2, .5, .5, 100, 0, 0.1);

    NelderMead2D nm(config);

    nm.execute();

    EXPECT_NEAR(nm.get_value(), 0, 0.1);
    EXPECT_NEAR(nm.get_point().x(), 0, 0.1);
    EXPECT_NEAR(nm.get_point().y(), 0, 0.1);
}