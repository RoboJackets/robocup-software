#include "WorldState.hpp"
#include <gtest/gtest.h>

TEST(BallState, Predict) {
    RJ::Time start = RJ::now();
    BallState state(
        Geometry2d::Point(0, 0),
        Geometry2d::Point(1, 0),
        start);

    BallState in_3_seconds = state.predict_in(RJ::Seconds(3));

    EXPECT_NEAR(in_3_seconds.velocity.x(), 1 - 3 * kBallDecayConstant, 1e-6);

    BallState in_6_seconds = state.predict_in(RJ::Seconds(6));

    EXPECT_EQ(in_6_seconds.velocity.x(), 0);
    EXPECT_EQ(in_6_seconds.position.x(), 1 / (2 * kBallDecayConstant));
}

TEST(BallState, QuerySecondsTo) {
    RJ::Time start = RJ::now();
    BallState state(
        Geometry2d::Point(0, 0),
        Geometry2d::Point(1, 0),
        start);

    BallState in_3_seconds = state.predict_in(RJ::Seconds(3));

    Geometry2d::Point actual;
    EXPECT_NEAR(state.query_seconds_to(in_3_seconds.position, &actual).count(),
              3, 1e-6);
    EXPECT_TRUE(actual.nearPoint(in_3_seconds.position, 1e-6));
}

TEST(BallState, QueryFar) {
    RJ::Time start = RJ::now();
    BallState state(
        Geometry2d::Point(0, 0),
        Geometry2d::Point(1, 0),
        start);

    Geometry2d::Point actual;
    RJ::Seconds t = state.query_seconds_to(Geometry2d::Point(5, 0), &actual);

    EXPECT_EQ(t, RJ::Seconds::max());
    EXPECT_TRUE(actual.nearPoint(state.predict_in(t).position, 1e-6));
}
