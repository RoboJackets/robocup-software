#include <gtest/gtest.h>

#include "rj_common/world_state.hpp"

// Same value as ball_decay_constant in sim_params.yaml / real_params.yaml.
constexpr double kBallDecayConstant = 0.18;

TEST(BallState, Predict) {
    RJ::Time start = RJ::now();
    BallState state(rj_geometry::Point(0, 0), rj_geometry::Point(1, 0), start);

    BallState in_3_seconds = state.predict_in(RJ::Seconds(3), kBallDecayConstant);

    EXPECT_NEAR(in_3_seconds.velocity.x(), 1 - 3 * kBallDecayConstant, 1e-6);

    BallState in_6_seconds = state.predict_in(RJ::Seconds(6), kBallDecayConstant);

    EXPECT_EQ(in_6_seconds.velocity.x(), 0);
    EXPECT_EQ(in_6_seconds.position.x(), 1 / (2 * kBallDecayConstant));
}

TEST(BallState, QuerySecondsTo) {
    RJ::Time start = RJ::now();
    BallState state(rj_geometry::Point(0, 0), rj_geometry::Point(1, 0), start);

    BallState in_3_seconds = state.predict_in(RJ::Seconds(3), kBallDecayConstant);

    rj_geometry::Point actual;
    EXPECT_NEAR(
        state.query_seconds_near(in_3_seconds.position, kBallDecayConstant, &actual).count(), 3,
        1e-6);
    EXPECT_TRUE(actual.near_point(in_3_seconds.position, 1e-6));
}

TEST(BallState, QueryFar) {
    RJ::Time start = RJ::now();
    BallState state(rj_geometry::Point(0, 0), rj_geometry::Point(1, 0), start);

    rj_geometry::Point actual;
    RJ::Seconds t = state.query_seconds_near(rj_geometry::Point(5, 0), kBallDecayConstant, &actual);

    // TODO(#1499): For some reason query_stop_time and query_seconds_to_dist
    // give slightly different results, which results in this error being really
    // high (2e-3). Diagnose this and make sure it isn't a bug.
    EXPECT_NEAR(state.query_stop_time(kBallDecayConstant).count(), t.count(), 2e-3);
    EXPECT_TRUE(actual.near_point(state.predict_in(t, kBallDecayConstant).position, 1e-6));
}
