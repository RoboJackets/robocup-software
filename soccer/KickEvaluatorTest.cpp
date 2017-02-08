#include <gtest/gtest.h>
#include "KickEvaluator.hpp"
#include "SystemState.hpp"

#include <stdlib.h>

using namespace Geometry2d;

// Places a robot on the field that counts as an obstacle to the window
// evaluator, but is not actuallly in the way.  Should return one double as
// the result.
TEST(KickEvaluator, eval_pt_to_pt) {
    SystemState state;
    OurRobot* obstacleBot = state.self[0];
    obstacleBot->visible = true;
    obstacleBot->pos = Point(1, 1);

    Segment ourGoalSegment(
        Point(Field_Dimensions::Current_Dimensions.GoalWidth() / 2.0, 0),
        Point(-Field_Dimensions::Current_Dimensions.GoalWidth() / 2.0, 0));
    
    KickEvaluator kickEval(&state);
    double pt_to_seg = kickEval.eval_pt_to_seg(Point(0, 2), ourGoalSegment);
    double pt_to_our_goal = kickEval.eval_pt_to_our_goal(Point(0, 2));

    EXPECT_EQ(pt_to_seg, pt_to_our_goal);

    double pt_to_pt = kickEval.eval_pt_to_pt(Point(0, 0.1), Point(0, 0), 5);

    EXPECT_EQ(pt_to_pt, 1);
}
