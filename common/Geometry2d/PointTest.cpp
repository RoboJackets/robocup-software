#include <gtest/gtest.h>
// #include <../../common/Point.hpp>
#include <Geometry2d/Point.hpp>

using namespace std;
using namespace Geometry2d;

/*
 * Tests the constructor and basic operators of the Point class
 */
TEST(testPoint, basic) {
    //Test Constructors
    Point defaultConstructed;
    EXPECT_FLOAT_EQ(defaultConstructed.x, 0);
    EXPECT_FLOAT_EQ(defaultConstructed.y, 0);

    Point test(0,0);
    EXPECT_FLOAT_EQ(defaultConstructed.x, 0);
    EXPECT_FLOAT_EQ(defaultConstructed.y, 0);

    EXPECT_EQ(test, defaultConstructed);

    //Test +, - operator
    Point expected;
    expected.x += 5.5;
    expected.y -= 1.3;
    EXPECT_NE(expected, Point());

    EXPECT_EQ(expected, Point() + Point(5.5, -1.3));
    EXPECT_EQ(expected, Point() - Point(-5.5, 1.3));

    //Test +=, -= operator
    Point temp;
    temp += Point(5.5, -1.3);
    EXPECT_EQ(expected, temp);

    temp = Point();
    temp -= Point(-5.5, 1.3);
    EXPECT_EQ(expected, temp);


    //Test negative Operator
    EXPECT_FLOAT_EQ(-5.5, (-Point(5.5, 3.5)).x);
    EXPECT_FLOAT_EQ(-3.5, (-Point(5.5, 3.5)).y);

    //Test * Point operator
    expected = Point(2.5, 4.5);
    expected.x *= 2.5;
    expected.y *= -4.5;

    EXPECT_EQ(expected, Point(2.5, 4.5) * Point(2.5, -4.5));

    //Test / Point Operator
    expected = Point(2.5, 4.5);
    expected.x /= 2.5;
    expected.y /= -2.5;

    EXPECT_EQ(expected, Point(2.5, 4.5) / Point(2.5, -2.5));

    //Test *= float operator
    temp = Point(2.5, -4.5);
    temp *= 2.5;
    EXPECT_EQ(Point(2.5*2.5, -4.5*2.5), temp);

    //Test /=  float Operator
    temp = Point(2.5, -4.5);
    temp /= 2.5;

    EXPECT_EQ(Point(2.5/2.5, -4.5/2.5), temp);

}

TEST(testPoint, clampBig) {
    Point p0(10.0, 10.0);
    p0.clamp(10);
    EXPECT_FLOAT_EQ(10, p0.mag());
}

//  if you clamp a vector to a value that's bigger
//  than it's magnitued, it shouldn't change
TEST(testPoint, clampSmall) {
    Point p0(10, 0);
    p0.clamp(15);
    EXPECT_FLOAT_EQ(10, p0.mag());
}
