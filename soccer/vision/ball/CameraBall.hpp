#pragma once

#include <geometry2d/point.h>
#include <utils.h>

#include <vector>

/**
 * Wrapper for the protobuf observation
 */
class CameraBall {
public:
    /**
     * @param timeCaptured Time that the picture was taken
     * @param pos Position of the ball at that time
     */
 CameraBall(RJ::Time timeCaptured, geometry2d::Point pos)
     : timeCaptured(timeCaptured), pos(pos) {}

 /**
  * @return Time this measurement was captured
  */
 RJ::Time getTimeCaptured() const;

 /**
  * @return Position of the measurement
  */
 geometry2d::Point getPos() const;

 /**
  * Combines all the balls in the list and returns a ball
  * with the average pos and time
  *
  * @param balls The list of balls to combine
  */
 static CameraBall CombineBalls(const std::vector<CameraBall>& balls);

private:
    RJ::Time timeCaptured;
    geometry2d::Point pos;
};
