#include <gtest/gtest.h>
#include "Arc.hpp"
#include "Circle.hpp"
#include "CompositeShape.hpp"
#include "Polygon.hpp"
#include "Rect.hpp"
#include "Util.hpp"

#include <vector>

using namespace Geometry2d;
using namespace std;

TEST(CompositeShape, nearPoint) {
    vector<Point> verts{Point(5,-5), Point(6,-5), Point(6,-6), Point(5,-6)};

    shared_ptr<Circle>  circle = make_shared<Circle>(Point(0, 0), 1);
    shared_ptr<Polygon> polygon = make_shared<Polygon>(verts);
    shared_ptr<Rect>    rect = make_shared<Rect>(Point(-5,-5), Point(-4,-4));

    CompositeShape cs;
    cs.add(circle);
    cs.add(polygon);
    cs.add(rect);

    EXPECT_TRUE(cs.nearPoint(Point(1, 1), 1)); // Near circle
    EXPECT_TRUE(cs.nearPoint(Point(5, -7), 2)); // Near polygon
    EXPECT_TRUE(cs.nearPoint(Point(7, -6), 2)); // Near polygon
    EXPECT_TRUE(cs.nearPoint(Point(-6, -6), 2)); // Near rect

    EXPECT_FALSE(cs.nearPoint(Point(0, 3), 1));
    EXPECT_FALSE(cs.nearPoint(Point(-10, -10), 1));
    EXPECT_FALSE(cs.nearPoint(Point(10, -1), 1));
}