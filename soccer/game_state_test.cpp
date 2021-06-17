#include <gtest/gtest.h>

#include <game_state.hpp>
#include <rj_convert/testing/ros_convert_testing.hpp>

TEST(ROSConvertPlayState, play_state_lossless_convert) {
    PlayState mock_state = PlayState::ready_kickoff(true);
    test_lossless_convert_cpp_value<PlayState>(mock_state);
}

TEST(ROSConvertPlayState, play_state_lossless_convert_with_placement) {
    PlayState mock_state = PlayState::ball_placement(true, rj_geometry::Point(2.0, 3.0));
    test_lossless_convert_cpp_value<PlayState>(mock_state);
}
