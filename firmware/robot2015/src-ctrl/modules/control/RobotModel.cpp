#include "RobotModel.hpp"

const RobotModel RobotModel2015 = []() {
    RobotModel model;
    model.WheelRadius = 0.02856;
    // note: wheels are numbered clockwise, starting with the top-right
    model.WheelAngles = {
        DegreesToRadians(45), DegreesToRadians(315), DegreesToRadians(225),
        DegreesToRadians(135),
    };
    model.WheelDist = 0.0798576;

    model.DutyCycleMultiplier = 9;  // TODO: tune this value

    model.recalculateBotToWheel();

    return model;
}();
