#include <gtest/gtest.h>

#include "rj_geometry/point.hpp"
#include "rj_geometry/util.hpp"
using namespace std;
using namespace rj_geometry;

bool point_equal(Point p1, Point p2) {
    return nearly_equal(p1.x(), p2.x()) && nearly_equal(p1.y(), p2.y());
}
/*
 * Tests the constructor and basic operators of the Point class
 */
TEST(Point, constructors) {
    // Test Constructors
    Point default_constructed;
    EXPECT_FLOAT_EQ(default_constructed.x(), 0);
    EXPECT_FLOAT_EQ(default_constructed.y(), 0);

    Point test(0, 0);
    EXPECT_FLOAT_EQ(default_constructed.x(), 0);
    EXPECT_FLOAT_EQ(default_constructed.y(), 0);

    EXPECT_EQ(test, default_constructed);
}

TEST(Point, to_eigen) {
    Eigen::Vector2d p = Point(1.0, 2.0);
    ASSERT_EQ(p(0), 1.0);
    ASSERT_EQ(p(1), 2.0);
}

TEST(Point, from_eigen) {
    Point p = Eigen::Vector2d(1.0, 2.0);
    ASSERT_EQ(p.x(), 1.0);
    ASSERT_EQ(p.y(), 2.0);
}

TEST(Point, operators) {
    // Test +, - operator
    Point expected;
    expected.x() += 5.5;
    expected.y() -= 1.3;
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
    EXPECT_FLOAT_EQ(-5.5, (-Point(5.5, 3.5)).x());
    EXPECT_FLOAT_EQ(-3.5, (-Point(5.5, 3.5)).y());

    // Test * Point operator
    expected = Point(2.5, 4.5);
    expected.x() *= 2.5;
    expected.y() *= -4.5;

    EXPECT_EQ(expected, Point(2.5, 4.5) * Point(2.5, -4.5));

    // Test / Point Operator
    expected = Point(2.5, 4.5);
    expected.x() /= 2.5;
    expected.y() /= -2.5;

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
    EXPECT_PRED2(point_equal, temp, Point(1 * 3.5, -5 * 3.5));
    EXPECT_PRED2(point_equal, Point(1, -5) * 3.5, Point(1 * 3.5, -5 * 3.5));
    EXPECT_PRED2(point_equal, 3.5 * Point(1, -5), Point(1 * 3.5, -5 * 3.5));

    temp = Point(5.6, -3.6);
    std::stringstream out;
    out << temp;
    EXPECT_EQ("Point(5.6, -3.6)", out.str());
    EXPECT_EQ("Point(5.6, -3.6)", temp.to_string());
}

TEST(Point, mag) {
    EXPECT_FLOAT_EQ(5, Point(-3, 4).mag());
    EXPECT_FLOAT_EQ(25, Point(3, -4).magsq());
    EXPECT_FLOAT_EQ(8, Point(2, 2).magsq());
    EXPECT_FLOAT_EQ(sqrtf(8), Point(2, 2).mag());
}

TEST(Point, dot) {
    EXPECT_FLOAT_EQ(3.5 * 5.5 + 4.5 * 6.5, Point(3.5, 4.5).dot(Point(5.5, 6.5)));
    EXPECT_FLOAT_EQ(3.5 * -5.5 + -4.5 * 6.5, Point(3.5, -4.5).dot(Point(-5.5, 6.5)));
}

TEST(Point, rotation) {
    // test rotate
    const Point unit(1, 0);
    Point test = unit;
    Point expected = Point(sqrt(2) / 2, sqrt(2) / 2);
    EXPECT_PRED2(point_equal, expected, test.rotate(M_PI / 4));
    EXPECT_PRED2(point_equal, expected, unit.rotated(M_PI / 4));
    test = unit;
    EXPECT_PRED2(point_equal, expected, test.rotate(Point(0, 0), M_PI / 4));
    EXPECT_PRED2(point_equal, expected, Point::rotated(unit, Point(0, 0), M_PI / 4));
    EXPECT_PRED2(point_equal, expected, Point::direction(M_PI / 4));
    EXPECT_FLOAT_EQ(M_PI / 4, expected.angle());

    test = unit;
    EXPECT_PRED2(point_equal, expected, test.rotate(M_PI / 4 - 12 * M_PI));
    EXPECT_PRED2(point_equal, expected, unit.rotated(M_PI / 4 - 12 * M_PI));
    test = unit;
    EXPECT_PRED2(point_equal, expected, test.rotate(Point(0, 0), M_PI / 4 - 12 * M_PI));
    EXPECT_PRED2(point_equal, expected, Point::rotated(unit, Point(0, 0), M_PI / 4 - 12 * M_PI));
    EXPECT_PRED2(point_equal, expected, Point::direction(M_PI / 4 - 12 * M_PI));

    test = unit;
    expected = Point(3, 2);
    EXPECT_PRED2(point_equal, expected, test.rotate(Point(3, 0), -M_PI / 2 + 12 * M_PI));
    EXPECT_PRED2(point_equal, expected, unit.rotated(Point(3, 0), -M_PI / 2 + 12 * M_PI));
    EXPECT_PRED2(point_equal, expected, Point::rotated(unit, Point(3, 0), -M_PI / 2 + 12 * M_PI));

    EXPECT_PRED2(point_equal, expected.rotated(M_PI / 2), expected.perp_ccw());
    EXPECT_PRED2(point_equal, expected.rotated(-M_PI / 2), expected.perp_cw());
}

TEST(Point, clamp_big) {
    Point p0(10.0, 10.0);
    p0.clamp(10);
    EXPECT_FLOAT_EQ(10, p0.mag());
    EXPECT_PRED2(point_equal, p0, Point(sqrt(50), sqrt(50)));

    p0 = Point(-20.5, 1.5);
    float mag = p0.mag();
    p0.clamp(1);
    EXPECT_FLOAT_EQ(1, p0.mag());
    EXPECT_PRED2(point_equal, p0, Point(-20.5 / mag, 1.5 / mag));
}

//  if you clamp a vector to a value that's bigger
//  than it's magnitude, it shouldn't change
TEST(Point, clamp_small) {
    Point p0(10, -6);
    p0.clamp(15);
    EXPECT_TRUE(p0 == Point(10, -6));
    p0.clamp(sqrt(10 * 10 + 6 * 6));
    EXPECT_PRED2(point_equal, p0, Point(10, -6));
}

TEST(Point, dist_to) {
    EXPECT_FLOAT_EQ(0, Point(5, 3).dist_to(Point(5, 3)));
    EXPECT_FLOAT_EQ(5, Point(4, 5).dist_to(Point(1, 1)));
    EXPECT_FLOAT_EQ(5, Point(1.5, 1.5).dist_to(Point(4.5, 5.5)));
    EXPECT_FLOAT_EQ((Point(93, 44) - Point(4.5, 5.5)).mag(),
                    Point(93, 44).dist_to(Point(4.5, 5.5)));
}

TEST(Point, near_point) {
    for (float y = -5; y <= 5; y += 0.01) {
        for (float x = -5; x <= 5; x += 0.01) {
            Point point(x, y);
            float close = (pow(x - 1.1, 2) + pow(y - 1.3, 2)) - 2.5 * 2.5;
            if (!nearly_equal(close, 0)) {
                EXPECT_EQ(close <= 0, point.near_point(Point(1.1, 1.3), 2.5))
                    << "x:" << x << " y:" << y;
            }
        }
    }
}

// TODO(ashaw596) Add tests for the changed normalized
TEST(Point, normalized) {
    Point expected(5 / sqrt(5 * 5 + 2 * 2), 2 / sqrt(5 * 5 + 2 * 2));
    EXPECT_PRED2(point_equal, expected, Point(5 * 1.4, 2 * 1.4).normalized());
    EXPECT_PRED2(point_equal, expected, Point(5 / 1.4, 2 / 1.4).normalized());
    EXPECT_PRED2(point_equal, Point(0, 0), Point(0, 0).normalized());
}

TEST(Point, nearly_equals) {
    double test_tolerance = 1e-4;
    EXPECT_TRUE(Point(0, 5).nearly_equals(Point(0, 5), test_tolerance));
    EXPECT_TRUE(Point(0, 5).nearly_equals(Point(0 - test_tolerance / 2, 5 + test_tolerance / 2),
                                          test_tolerance));
    EXPECT_TRUE(Point(0, 5).nearly_equals(Point(0 + test_tolerance / 2, 5 - test_tolerance / 2),
                                          test_tolerance));

    EXPECT_FALSE(Point(0, 5).nearly_equals(
        Point(0 - test_tolerance * 1.1, 5 + test_tolerance * 1.1), test_tolerance));
    EXPECT_FALSE(Point(0, 5).nearly_equals(
        Point(0 + test_tolerance * 1.1, 5 - test_tolerance * 1.1), test_tolerance));
}
// TODO(ashaw596) Add tests for angle_to and angle_between once those changes are
// merged
