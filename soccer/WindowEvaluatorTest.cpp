#include <gtest/gtest.h>
#include "WindowEvaluator.hpp"
#include "SystemState.hpp"
#include <stdlib.h>

using namespace Geometry2d;

// Places a robot on the field that counts as an obstacle to the window
// evaluator, but is not actuallly in the way.  Should return one segment as
// the result.
TEST(WindowEvaluator, eval_pt_to_pt) {
    SystemState state;

    float w = Field_Dimensions::Current_Dimensions.Width();
    float l = Field_Dimensions::Current_Dimensions.Length();

    srand(10);
    for (int i = 0; i < 6; i++) {
        state.self[i]->pos = Point(rand() * w - w/2, rand() * l);
        state.self[i]->visible = true;
    }
    
    Point ourGoalCenter(0, 0);
    int num_w = 500;
    int num_l = 1000;

    for (float x = -w/2; x < w/2; x += w/num_w) {
        for (float y = 0; y < l; y += l/num_l) {
            WindowEvaluator winEval(&state);
            WindowingResult result = winEval.eval_pt_to_pt(Point(x, y), ourGoalCenter, .2);
        }
    }

    EXPECT_EQ(0, 0);
}
