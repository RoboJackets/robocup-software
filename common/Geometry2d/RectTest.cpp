#include <gtest/gtest.h>
#include <Geometry2d/Rect.hpp>
#include <Constants.hpp>

namespace Geometry2d {

static Rect example(Point(0, 0), Point(2, 1));
static Rect intersectExample(Point(-1,1), Point(1,2));
static Rect nullExample(Point(0,0),Point(0,0));

TEST(Rect, hit) {
    EXPECT_FALSE(example.hit(Point(10, 10)));
    EXPECT_TRUE(example.hit(Point(0, 0.5)));
    EXPECT_TRUE(example.hit(Point(0.5, example.maxy() + Robot_Radius - 0.01)));
    EXPECT_TRUE(example.hit(Point(0, 0)));
    EXPECT_TRUE(example.hit(Point(2, 1)));
    EXPECT_TRUE(example.hit(Point(0, 1)));
    EXPECT_TRUE(example.hit(Point(2, 0)));
    EXPECT_TRUE(example.hit(Point(1, -0.09)));
    EXPECT_FALSE(example.hit(Point(1, -0.091)));

    // degenerate case
    EXPECT_TRUE(nullExample.hit(Point(0, 0)));
    EXPECT_TRUE(nullExample.hit(Point(0, 0.09)));
    EXPECT_FALSE(nullExample.hit(Point(0, 0.091)));
    EXPECT_FALSE(nullExample.hit(Point(1, 1)));
}

TEST(Rect, containsPoint) {
    EXPECT_TRUE(example.containsPoint(Point(0, 0.5)));
    EXPECT_FALSE(example.containsPoint(Point(-1, 01)));
}

/* Tests below can be uncommented to test if this function breaks
 * Needs to be made public for tests to work.
TEST(Rect, cohenCodes){
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(0,1.2)), 0x00);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(-1,1)), 0x00);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(-1.5,1.5)), 0x01);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(1.5,1.2)), 0x02);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(1.5,2)), 0x02);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(-.5,0)), 0x04);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(-1,0)), 0x04);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(-1,3)), 0x08);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(-.5,3)), 0x08);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(-1.1,0)), 0x05);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(1.1,0)), 0x06);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(-1.5,2.5)), 0x09);
    EXPECT_EQ(intersectExample.CohenSutherlandOutCode(Point(1.5,2.5)), 0x0A);
}*/

/*TEST(Rect, degenerageCohenCodes){
    EXPECT_EQ(nullExample.CohenSutherlandOutCode(Point(0,0)), 0x00);
    EXPECT_EQ(nullExample.CohenSutherlandOutCode(Point(1,0)), 0x02);
    EXPECT_EQ(nullExample.CohenSutherlandOutCode(Point(-1,0)), 0x01);
    EXPECT_EQ(nullExample.CohenSutherlandOutCode(Point(0,1)), 0x08);
    EXPECT_EQ(nullExample.CohenSutherlandOutCode(Point(0,-.1)), 0x04);
}*/

TEST(Rect, degenerageSegmentIntersection){
    std::tuple<bool, std::vector<Point> > res = nullExample.intersects(Segment(Point(1,0),Point(-1,0)));
    EXPECT_TRUE(std::get<0>(res));
    res = nullExample.intersects(Segment(Point(1,.1),Point(-1,0)));
    EXPECT_FALSE(std::get<0>(res));
}

TEST(Rect, SegmentIntersection){
    Point v1 = Point(-1,1);
    Point v2 = Point(1,2);
    std::tuple<bool, std::vector<Point> > res = intersectExample.intersects(Segment(Point(1,.1),Point(-1,0)));
    EXPECT_FALSE(std::get<0>(res));
    res = intersectExample.intersects(Segment(Point(-2,.5),Point(2,2.5)));
    std::vector<Point> intersectionPoints = std::get<1>(res);
    std::vector<Geometry2d::Point>::iterator it;
    EXPECT_TRUE(std::get<0>(res));
    EXPECT_TRUE(intersectionPoints.size() == 2);
    for (it=intersectionPoints.begin(); it!=intersectionPoints.end(); it++){
        EXPECT_TRUE(*it==v1 || *it==v2 );
    }
}

}  // namespace Geometry2d  
