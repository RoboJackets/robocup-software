#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "geometry2d/circle.h"
#include "geometry2d/line.h"
#include "geometry2d/point.h"
#include "geometry2d/segment.h"

namespace geometry2d::Testing {

TEST(Circle, GetterSetter) {
    Point center(1, 1);
    float radius = 1.1;

    Circle c(center, radius);

    EXPECT_NEAR(c.center.x(), center.x(), 0.001);
    EXPECT_NEAR(c.center.y(), center.y(), 0.001);
    EXPECT_NEAR(c.radius(), radius, 0.001);
    EXPECT_NEAR(c.radius_sq(), radius * radius, 0.001);

    radius = 2.2;
    c.radius_sq(radius * radius);

    EXPECT_NEAR(c.radius(), radius, 0.001);
    EXPECT_NEAR(c.radius_sq(), radius * radius, 0.001);

    radius = 3.3;
    c.radius(radius);

    EXPECT_NEAR(c.radius(), radius, 0.001);
    EXPECT_NEAR(c.radius_sq(), radius * radius, 0.001);
}

TEST(Circle, ContainsPt) {
    Circle c({0, 0}, 1);

    EXPECT_TRUE(c.containsPoint({0.4, 0.4}));
    EXPECT_FALSE(c.containsPoint({1.4, 0}));
}

TEST(Circle, Hit) {
    Circle c({0, 0}, 1);

    EXPECT_TRUE(c.hit(Point(0.4, 0.4)));
    EXPECT_FALSE(c.hit(Point(2, 0)));

    EXPECT_TRUE(c.hit(Segment({1, -1}, {1, 1})));
    EXPECT_FALSE(c.hit(Segment({4, -1}, {4, 1})));
}

TEST(Circle, NearPoint) {
    Circle c({0, 0}, 1);

    EXPECT_TRUE(c.nearPoint({0.5, 0}, 1));
    EXPECT_TRUE(c.nearPoint({1.5, 0}, 1));
    EXPECT_FALSE(c.nearPoint({2.5, 0}, 1));
}

TEST(Circle, Intersects) {
    Circle c({0, 0}, 1);

    Point out[2];

    Circle c1({1, 0}, 1);
    Circle c2({2, 0}, 1);
    Circle c3({3, 0}, 1);
    Circle c4({0, 0}, 1);

    // With nullptr
    EXPECT_EQ(c.intersects(c1), 2);
    EXPECT_EQ(c.intersects(c2), 1);
    EXPECT_EQ(c.intersects(c3), 0);
    EXPECT_EQ(c.intersects(c4), 0);

    // Will return points
    EXPECT_EQ(c.intersects(c1, out), 2);
    EXPECT_NEAR(out[0].x(), 0.5, 0.01);
    EXPECT_NEAR(out[1].x(), 0.5, 0.01);
    EXPECT_NEAR(out[0].y(), -0.866, 0.1);
    EXPECT_NEAR(out[1].y(), 0.866, 0.1);
    EXPECT_EQ(c.intersects(c2, out), 1);
    EXPECT_NEAR(out[0].x(), 1, 0.01);
    EXPECT_NEAR(out[0].y(), 0, 0.01);
    EXPECT_EQ(c.intersects(c3, out), 0);
    EXPECT_EQ(c.intersects(c4, out), 0);

    Line l1({0, -1}, {0, 1});
    Line l2({1, -1}, {1, 1});
    Line l3({2, -1}, {2, 1});

    // With nullptr
    EXPECT_EQ(c.intersects(l1), 2);
    EXPECT_EQ(c.intersects(l2), 1);
    EXPECT_EQ(c.intersects(l3), 0);

    // Will return points
    EXPECT_EQ(c.intersects(l1, out), 2);
    EXPECT_NEAR(out[0].x(), 0, 0.01);
    EXPECT_NEAR(out[1].x(), 0, 0.01);
    EXPECT_NEAR(out[0].y(), 1, 0.01);
    EXPECT_NEAR(out[1].y(), -1, 0.01);
    EXPECT_EQ(c.intersects(l2, out), 1);
    EXPECT_NEAR(out[0].x(), 1, 0.01);
    EXPECT_NEAR(out[0].y(), 0, 0.01);
    EXPECT_EQ(c.intersects(l3, out), 0);
}

TEST(Circle, TangentPoints) {
    Circle c({0, 0}, 1);

    Point out1;
    Point out2;

    Point s1(0, 0);
    Point s2(1000000, 0);

    // Nullptrs
    EXPECT_FALSE(c.tangentPoints(s1));

    // Inside circle
    EXPECT_FALSE(c.tangentPoints(s1, &out1, &out2));

    // Outside circle very far away
    // Tangets are basically top and bottom
    EXPECT_TRUE(c.tangentPoints(s2, &out1, &out2));
    EXPECT_NEAR(out1.x(), 0.01, 0.01);
    EXPECT_NEAR(out1.y(), -0.99, 0.01);
    EXPECT_NEAR(out2.x(), 0.01, 0.01);
    EXPECT_NEAR(out2.y(), 0.99, 0.01);
}

TEST(Circle, NearestPoint) {
    Circle c({0, 0}, 1);

    // No solution so it returns center
    Point out1 = c.nearestPoint({0, 0});
    EXPECT_NEAR(out1.x(), 0, 0.01);
    EXPECT_NEAR(out1.y(), 0, 0.01);

    // Inside circle
    Point out2 = c.nearestPoint({0.1, 0});
    EXPECT_NEAR(out2.x(), 1, 0.01);
    EXPECT_NEAR(out2.y(), 0, 0.01);

    // On circle
    Point out3 = c.nearestPoint({1, 0});
    EXPECT_NEAR(out3.x(), 1, 0.01);
    EXPECT_NEAR(out3.y(), 0, 0.01);

    // Outside circle
    Point out4 = c.nearestPoint({2, 0});
    EXPECT_NEAR(out4.x(), 1, 0.01);
    EXPECT_NEAR(out4.y(), 0, 0.01);
}

}  // namespace geometry2d::Testing
