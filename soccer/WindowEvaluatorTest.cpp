#include <gtest/gtest.h>
#include "WindowEvaluator.hpp"
#include "SystemState.hpp"
#include "Configuration.hpp"

using namespace Geometry2d;

Configuration config;

// Places a robot on the field that counts as an obstacle to the window
// evaluator, but is not actuallly in the way.  Should return one segment as
// the result.
TEST(WindowEvaluator, eval_pt_to_seg) {
    SystemState state;
    OurRobot* obstacleBot = state.self[0];
    obstacleBot->visible = true;
    obstacleBot->pos = Point(1, 1);

    Segment ourGoalSegment(
        Point(Field_Dimensions::Current_Dimensions.GoalWidth() / 2.0, 0),
        Point(-Field_Dimensions::Current_Dimensions.GoalWidth() / 2.0, 0));

    WindowEvaluator winEval(&state);
    WindowingResult result =
        winEval.eval_pt_to_seg(Point(1, 2), ourGoalSegment);

    auto& windows = result.first;
    auto& best = result.second;

    // there should be a best window, there's nothing in the way
    EXPECT_NE(boost::none, best);

    // there should only be one window
    EXPECT_EQ(1, windows.size());

    // the window should be our goal segment
    EXPECT_EQ(ourGoalSegment, windows[0].segment);
}
