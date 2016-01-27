#include <gtest/gtest.h>
// #include <../../common/Point.hpp>
#include <Geometry2d/Point.hpp>

using namespace std;
using namespace Geometry2d;

bool floatEqual(float a, float b) {
    const float EPSILON = 0.00001;
    return fabs(a-b) < EPSILON;
}

bool pointEqual(Point p1, Point p2) {
    return floatEqual(p1.x, p2.x) && floatEqual(p1.y, p2.y);
}
/*
 * Tests the constructor and basic operators of the Point class
 */
TEST(Point, basic) {
    // Test Constructors
    Point defaultConstructed;
    EXPECT_FLOAT_EQ(defaultConstructed.x, 0);
    EXPECT_FLOAT_EQ(defaultConstructed.y, 0);

    Point test(0, 0);
    EXPECT_FLOAT_EQ(defaultConstructed.x, 0);
    EXPECT_FLOAT_EQ(defaultConstructed.y, 0);

    EXPECT_EQ(test, defaultConstructed);

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

    EXPECT_FLOAT_EQ(5, Point(3,4).mag());
    EXPECT_FLOAT_EQ(25, Point(3,4).magsq());
    EXPECT_FLOAT_EQ(8, Point(2,2).magsq());
    EXPECT_FLOAT_EQ(sqrtf(8), Point(2,2).mag());
    EXPECT_FLOAT_EQ(3.5*5.5 + 4.5*6.5, Point(3.5,4.5).dot(Point(5.5,6.5)));


    //EXPECT_PRED_FORMAT2(pointEqual, direction())

}

TEST(Point, rotation) {

}

TEST(Point, clampBig) {
    Point p0(10.0, 10.0);
    p0.clamp(10);
    EXPECT_FLOAT_EQ(10, p0.mag());
}

//  if you clamp a vector to a value that's bigger
//  than it's magnitued, it shouldn't change
TEST(Point, clampSmall) {
    Point p0(10, 0);
    p0.clamp(15);
    EXPECT_FLOAT_EQ(10, p0.mag());
}
