#include <gtest/gtest.h>

#include <vector>

#include "Geometry2d/Arc.hpp"
#include "Geometry2d/Circle.hpp"
#include "Geometry2d/CompositeShape.hpp"
#include "Geometry2d/Polygon.hpp"
#include "Geometry2d/Rect.hpp"
#include "Geometry2d/Util.hpp"

using namespace Geometry2d;
using namespace std;

TEST(CompositeShape, ContainsPoint) {
    vector<Point> verts{Point(5, -5), Point(6, -5), Point(6, -6), Point(5, -6)};

    shared_ptr<Circle> circle = make_shared<Circle>(Point(0, 0), 1);
    shared_ptr<Polygon> polygon = make_shared<Polygon>(verts);
    shared_ptr<Rect> rect = make_shared<Rect>(Point(-5, -5), Point(-4, -4));

    CompositeShape cs;
    cs.add(circle);
    cs.add(polygon);
    cs.add(rect);

    EXPECT_TRUE(cs.containsPoint({0, 0.5}));      // In circle
    EXPECT_TRUE(cs.containsPoint({5.5, -5.5}));   // In polygon
    EXPECT_TRUE(cs.containsPoint({-4.5, -4.5}));  // In rect

    EXPECT_FALSE(cs.containsPoint(Point(0, 3)));
}

TEST(CompositeShape, NearPoint) {
    vector<Point> verts{Point(5, -5), Point(6, -5), Point(6, -6), Point(5, -6)};

    shared_ptr<Circle> circle = make_shared<Circle>(Point(0, 0), 1);
    shared_ptr<Polygon> polygon = make_shared<Polygon>(verts);
    shared_ptr<Rect> rect = make_shared<Rect>(Point(-5, -5), Point(-4, -4));

    CompositeShape cs;
    cs.add(circle);
    cs.add(polygon);
    cs.add(rect);

    EXPECT_TRUE(cs.nearPoint(Point(1, 1), 1));    // Near circle
    EXPECT_TRUE(cs.nearPoint(Point(5, -7), 2));   // Near polygon
    EXPECT_TRUE(cs.nearPoint(Point(7, -6), 2));   // Near polygon
    EXPECT_TRUE(cs.nearPoint(Point(-6, -6), 2));  // Near rect

    EXPECT_FALSE(cs.nearPoint(Point(0, 3), 1));
    EXPECT_FALSE(cs.nearPoint(Point(-10, -10), 1));
    EXPECT_FALSE(cs.nearPoint(Point(10, -1), 1));
}

TEST(CompositeShape, Hit) {
    vector<Point> verts{Point(5, -5), Point(6, -5), Point(6, -6), Point(5, -6)};

    shared_ptr<Circle> circle = make_shared<Circle>(Point(0, 0), 1);
    shared_ptr<Polygon> polygon = make_shared<Polygon>(verts);
    shared_ptr<Rect> rect = make_shared<Rect>(Point(-5, -5), Point(-4, -4));

    CompositeShape cs;
    cs.add(circle);
    cs.add(polygon);
    cs.add(rect);

    Point p1(0, 0);
    Point p2(5.5, -5.5);
    Point p3(-4.5, -4.5);
    Point p4(10, 10);

    EXPECT_TRUE(cs.hit(p1));  // Hit circle
    EXPECT_TRUE(cs.hit(p2));  // Hit polygon
    EXPECT_TRUE(cs.hit(p3));  // Hit rect

    EXPECT_FALSE(cs.hit(p4));

    Segment s1({-1, 0}, {1, 0});
    Segment s2({5, -5}, {6, -6});
    Segment s3({-4, -4}, {-5, -5});
    Segment s4({-10, 1}, {-20, 0});

    EXPECT_TRUE(cs.hit(s1));  // Hit circle
    EXPECT_TRUE(cs.hit(s2));  // Hit polygon
    EXPECT_TRUE(cs.hit(s3));  // Hit rect

    EXPECT_FALSE(cs.hit(s4));
}

TEST(CompositeShape, Container) {
    vector<Point> verts{Point(5, -5), Point(6, -5), Point(6, -6), Point(5, -6)};

    shared_ptr<Circle> circle = make_shared<Circle>(Point(0, 0), 1);
    shared_ptr<Polygon> polygon = make_shared<Polygon>(verts);
    shared_ptr<Rect> rect = make_shared<Rect>(Point(-5, -5), Point(-4, -4));

    CompositeShape cs;
    cs.add(circle);

    EXPECT_EQ(cs.subshapes().size(), 1);

    cs.add(polygon);

    EXPECT_EQ(cs.subshapes().size(), 2);

    cs.add(rect);

    EXPECT_EQ(cs.subshapes().size(), 3);
    EXPECT_FALSE(cs.empty());

    // Check [] accessor
    EXPECT_NE(cs[0], nullptr);
    EXPECT_NE(cs[1], nullptr);
    EXPECT_NE(cs[2], nullptr);

    // Check iterators
    EXPECT_NE(*cs.begin(), nullptr);
    EXPECT_NE(*(--cs.end()), nullptr);  // end is 1 past end, so -- to get end
}
