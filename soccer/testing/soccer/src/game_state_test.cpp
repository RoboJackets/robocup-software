#include <gtest/gtest.h>

#include <GameState.hpp>
#include <rj_convert/testing/ros_convert_testing.hpp>

bool operator==(const GameState& a, const GameState& b) {
    return a.period == b.period && a.state == b.state &&
           a.restart == b.restart && a.our_restart == b.our_restart &&
           a.stage_time_left == b.stage_time_left &&
           a.ball_placement_point == b.ball_placement_point;
}

TEST(ROSConvertGameState, game_state_lossless_convert) {
    GameState mock_state{GameState::Period::Halftime,
                         GameState::State::Playing,
                         GameState::Restart::Kickoff,
                         true,
                         RJ::Seconds(1.0),
                         std::nullopt};
    TEST_LOSSLESS_CONVERT_CPP_VALUE(GameState, GameState::Msg, mock_state);
}

TEST(ROSConvertGameState, game_state_lossless_convert_with_placement) {
    GameState mock_state{GameState::Period::Halftime,
                         GameState::State::Playing,
                         GameState::Restart::Placement,
                         true,
                         RJ::Seconds(1.0),
                         Geometry2d::Point(1.0, 2.0)};
    TEST_LOSSLESS_CONVERT_CPP_VALUE(GameState, GameState::Msg, mock_state);
}
