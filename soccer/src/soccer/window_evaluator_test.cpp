#include <gtest/gtest.h>

#include "configuration.hpp"
#include "system_state.hpp"
#include "window_evaluator.hpp"

using namespace Geometry2d;

Configuration config;

// Places a robot on the field that counts as an obstacle to the window
// evaluator, but is not actuallly in the way.  Should return one segment as
// the result.
TEST(WindowEvaluator, eval_pt_to_seg) {
    Context context;
    OurRobot* obstacle_bot = context.state.self[0];
    obstacle_bot->mutable_state().visible = true;
    obstacle_bot->mutable_state().pose = Pose(1, 1, 0);

    Segment our_goal_segment(Point(FieldDimensions::current_dimensions.goal_width() / 2.0, 0),
                             Point(-FieldDimensions::current_dimensions.goal_width() / 2.0, 0));

    WindowEvaluator win_eval(&context);
    WindowingResult result = win_eval.eval_pt_to_seg(Point(1, 2), our_goal_segment);

    auto& windows = result.first;
    auto& best = result.second;

    // there should be a best window, there's nothing in the way
    EXPECT_NE(std::nullopt, best);

    // there should only be one window
    EXPECT_EQ(1, windows.size());

    // the window should be our goal segment
    EXPECT_EQ(our_goal_segment, windows[0].segment);
}