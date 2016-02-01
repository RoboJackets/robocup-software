#include <gtest/gtest.h>
#include "Geometry2d/Point.hpp"
#include "Geometry2d/Util.hpp"
using namespace std;
using namespace Geometry2d;

bool pointEqual(Point p1, Point p2) {
    return nearlyEqual(p1.x, p2.x) && nearlyEqual(p1.y, p2.y);
}
/*
 * Tests the constructor and basic operators of the Point class
 */
TEST(Point, constructors) {
    // Test Constructors
    Point defaultConstructed;
    EXPECT_FLOAT_EQ(defaultConstructed.x, 0);
    EXPECT_FLOAT_EQ(defaultConstructed.y, 0);

    Point test(0, 0);
    EXPECT_FLOAT_EQ(defaultConstructed.x, 0);
    EXPECT_FLOAT_EQ(defaultConstructed.y, 0);

    EXPECT_EQ(test, defaultConstructed);
}

TEST(Point, operators) {
    // Test +, - operator
    Point expected;
    expected.x += 5.5;
    expected.y -= 1.3;
    EXPECT_NE(expected, Point());

    EXPECT_EQ(expected, Point() + Point(5.5, -1.3));
    EXPECT_EQ(expected, Point() - Point(-5.5, 1.3));

    // Test +=, -= operator
    Point temp;
    temp += Point(5.5, -1.3);
    EXPECT_EQ(expected, temp);

    temp = Point();
    temp -= Point(-5.5, 1.3);
    EXPECT_EQ(expected, temp);

    // Test negative Operator
    EXPECT_FLOAT_EQ(-5.5, (-Point(5.5, 3.5)).x);
    EXPECT_FLOAT_EQ(-3.5, (-Point(5.5, 3.5)).y);

    // Test * Point operator
    expected = Point(2.5, 4.5);
    expected.x *= 2.5;
    expected.y *= -4.5;

    EXPECT_EQ(expected, Point(2.5, 4.5) * Point(2.5, -4.5));

    // Test / Point Operator
    expected = Point(2.5, 4.5);
    expected.x /= 2.5;
    expected.y /= -2.5;

    EXPECT_EQ(expected, Point(2.5, 4.5) / Point(2.5, -2.5));

    // Test *= float operator
    temp = Point(2.5, -4.5);
    temp *= 2.5;
    EXPECT_EQ(Point(2.5 * 2.5, -4.5 * 2.5), temp);

    // Test /=  float Operator
    temp = Point(2.5, -4.5);
    temp /= 2.5;
    EXPECT_EQ(Point(2.5 / 2.5, -4.5 / 2.5), temp);

    // Test operator==
    EXPECT_TRUE(Point(1, 4) == Point(1, 4));
    EXPECT_FALSE(Point(1, 4) == Point(1.01, 4));
    EXPECT_FALSE(Point(1, 4) == Point(1, 4.01));

    // TEst operator!=
    EXPECT_FALSE(Point(1, 4) != Point(1, 4));
    EXPECT_TRUE(Point(1, 4) != Point(1.01, 4));
    EXPECT_TRUE(Point(1, 4) != Point(1, 4.01));

    // Test operator * Point x float
    temp = Point(1, -5);
    temp *= 3.5;
    EXPECT_PRED2(pointEqual, temp, Point(1 * 3.5, -5 * 3.5));
    EXPECT_PRED2(pointEqual, Point(1, -5) * 3.5, Point(1 * 3.5, -5 * 3.5));
    EXPECT_PRED2(pointEqual, 3.5 * Point(1, -5), Point(1 * 3.5, -5 * 3.5));

    temp = Point(5.6, -3.6);
    std::stringstream out;
    out << temp;
    EXPECT_EQ("Point(5.6, -3.6)", out.str());
    EXPECT_EQ("Point(5.6, -3.6)", temp.toString());
}

TEST(Point, mag) {
    EXPECT_FLOAT_EQ(5, Point(-3, 4).mag());
    EXPECT_FLOAT_EQ(25, Point(3, -4).magsq());
    EXPECT_FLOAT_EQ(8, Point(2, 2).magsq());
    EXPECT_FLOAT_EQ(sqrtf(8), Point(2, 2).mag());
}

TEST(Point, dot) {
    EXPECT_FLOAT_EQ(3.5 * 5.5 + 4.5 * 6.5,
                    Point(3.5, 4.5).dot(Point(5.5, 6.5)));
    EXPECT_FLOAT_EQ(3.5 * -5.5 + -4.5 * 6.5,
                    Point(3.5, -4.5).dot(Point(-5.5, 6.5)));
}

TEST(Point, rotation) {
    // test rotate
    const Point unit(1, 0);
    Point test = unit;
    Point expected = Point(sqrt(2) / 2, sqrt(2) / 2);
    EXPECT_PRED2(pointEqual, expected, test.rotate(M_PI / 4));
    EXPECT_PRED2(pointEqual, expected, unit.rotated(M_PI / 4));
    test = unit;
    EXPECT_PRED2(pointEqual, expected, test.rotate(Point(0, 0), M_PI / 4));
    EXPECT_PRED2(pointEqual, expected,
                 Point::rotated(unit, Point(0, 0), M_PI / 4));
    EXPECT_PRED2(pointEqual, expected, Point::direction(M_PI / 4));
    EXPECT_FLOAT_EQ(M_PI / 4, expected.angle());

    test = unit;
    EXPECT_PRED2(pointEqual, expected, test.rotate(M_PI / 4 - 12 * M_PI));
    EXPECT_PRED2(pointEqual, expected, unit.rotated(M_PI / 4 - 12 * M_PI));
    test = unit;
    EXPECT_PRED2(pointEqual, expected,
                 test.rotate(Point(0, 0), M_PI / 4 - 12 * M_PI));
    EXPECT_PRED2(pointEqual, expected,
                 Point::rotated(unit, Point(0, 0), M_PI / 4 - 12 * M_PI));
    EXPECT_PRED2(pointEqual, expected, Point::direction(M_PI / 4 - 12 * M_PI));

    test = unit;
    expected = Point(3, 2);
    EXPECT_PRED2(pointEqual, expected,
                 test.rotate(Point(3, 0), -M_PI / 2 + 12 * M_PI));
    EXPECT_PRED2(pointEqual, expected,
                 unit.rotated(Point(3, 0), -M_PI / 2 + 12 * M_PI));
    EXPECT_PRED2(pointEqual, expected,
                 Point::rotated(unit, Point(3, 0), -M_PI / 2 + 12 * M_PI));

    EXPECT_PRED2(pointEqual, expected.rotated(M_PI / 2), expected.perpCCW());
    EXPECT_PRED2(pointEqual, expected.rotated(-M_PI / 2), expected.perpCW());
}

TEST(Point, clampBig) {
    Point p0(10.0, 10.0);
    p0.clamp(10);
    EXPECT_FLOAT_EQ(10, p0.mag());
    EXPECT_PRED2(pointEqual, p0, Point(sqrt(50), sqrt(50)));

    p0 = Point(-20.5, 1.5);
    float mag = p0.mag();
    p0.clamp(1);
    EXPECT_FLOAT_EQ(1, p0.mag());
    EXPECT_PRED2(pointEqual, p0, Point(-20.5 / mag, 1.5 / mag));
}

//  if you clamp a vector to a value that's bigger
//  than it's magnitude, it shouldn't change
TEST(Point, clampSmall) {
    Point p0(10, -6);
    p0.clamp(15);
    EXPECT_TRUE(p0 == Point(10, -6));
    p0.clamp(sqrt(10 * 10 + 6 * 6));
    EXPECT_PRED2(pointEqual, p0, Point(10, -6));
}

TEST(Point, distTo) {
    EXPECT_FLOAT_EQ(0, Point(5, 3).distTo(Point(5, 3)));
    EXPECT_FLOAT_EQ(5, Point(4, 5).distTo(Point(1, 1)));
    EXPECT_FLOAT_EQ(5, Point(1.5, 1.5).distTo(Point(4.5, 5.5)));
    EXPECT_FLOAT_EQ((Point(93, 44) - Point(4.5, 5.5)).mag(),
                    Point(93, 44).distTo(Point(4.5, 5.5)));
}

TEST(Point, nearPoint) {
    for (float y = -5; y <= 5; y += 0.01) {
        for (float x = -5; x <= 5; x += 0.01) {
            Point point(x, y);
            float close = (pow(x - 1.1, 2) + pow(y - 1.3, 2)) - 2.5 * 2.5;
            if (!nearlyEqual(close, 0)) {
                EXPECT_EQ(close <= 0, point.nearPoint(Point(1.1, 1.3), 2.5))
                    << "x:" << x << " y:" << y;
            }
        }
    }
}

// TODO(ashaw596) Add tests for the changed normalized
TEST(Point, normalized) {
    Point expected(5 / sqrt(5 * 5 + 2 * 2), 2 / sqrt(5 * 5 + 2 * 2));
    EXPECT_PRED2(pointEqual, expected, Point(5 * 1.4, 2 * 1.4).normalized());
    EXPECT_PRED2(pointEqual, expected, Point(5 / 1.4, 2 / 1.4).normalized());
    EXPECT_PRED2(pointEqual, Point(0, 0), Point(0, 0).normalized());
}

TEST(Point, nearlyEquals) {
    EXPECT_TRUE(Point(0, 5).nearlyEquals(Point(0, 5)));
    EXPECT_TRUE(Point(0, 5).nearlyEquals(
        Point(0 - FLOAT_EPSILON / 2, 5 + FLOAT_EPSILON / 2)));
    EXPECT_TRUE(Point(0, 5).nearlyEquals(
        Point(0 + FLOAT_EPSILON / 2, 5 - FLOAT_EPSILON / 2)));

    EXPECT_FALSE(Point(0, 5).nearlyEquals(
        Point(0 - FLOAT_EPSILON * 1.1, 5 + FLOAT_EPSILON * 1.1)));
    EXPECT_FALSE(Point(0, 5).nearlyEquals(
        Point(0 + FLOAT_EPSILON * 1.1, 5 - FLOAT_EPSILON * 1.1)));
}
// TODO(ashaw596) Add tests for angleTo and angleBetween once those changes are
// merged