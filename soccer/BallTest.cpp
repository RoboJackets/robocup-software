#include <gtest/gtest.h>

#include "WorldState.hpp"

TEST(BallState, Predict) {
    RJ::Time start = RJ::now();
    BallState state(Geometry2d::Point(0, 0), Geometry2d::Point(1, 0), start);

    BallState in_3_seconds = state.predict_in(RJ::Seconds(3));

    EXPECT_NEAR(in_3_seconds.velocity.x(), 1 - 3 * kBallDecayConstant, 1e-6);

    BallState in_6_seconds = state.predict_in(RJ::Seconds(6));

    EXPECT_EQ(in_6_seconds.velocity.x(), 0);
    EXPECT_EQ(in_6_seconds.position.x(), 1 / (2 * kBallDecayConstant));
}

TEST(BallState, QuerySecondsTo) {
    RJ::Time start = RJ::now();
    BallState state(Geometry2d::Point(0, 0), Geometry2d::Point(1, 0), start);

    BallState in_3_seconds = state.predict_in(RJ::Seconds(3));

    Geometry2d::Point actual;
    EXPECT_NEAR(
        state.query_seconds_near(in_3_seconds.position, &actual).count(), 3,
        1e-6);
    EXPECT_TRUE(actual.nearPoint(in_3_seconds.position, 1e-6));
}

TEST(BallState, QueryFar) {
    RJ::Time start = RJ::now();
    BallState state(Geometry2d::Point(0, 0), Geometry2d::Point(1, 0), start);

    Geometry2d::Point actual;
    RJ::Seconds t = state.query_seconds_near(Geometry2d::Point(5, 0), &actual);

    // TODO: For some reason query_stop_time and query_seconds_to_dist give
    // slightly
    //  different results, which results in this error being really high (2e-3).
    //  Diagnose this and make sure it isn't a bug.
    EXPECT_NEAR(state.query_stop_time().count(), t.count(), 2e-3);
    EXPECT_TRUE(actual.nearPoint(state.predict_in(t).position, 1e-6));
}
