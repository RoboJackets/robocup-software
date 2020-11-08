#pragma once

#include <rj_geometry/point.hpp>
#include <rj_geometry/transform_matrix.hpp>
#include <rj_common/time.hpp>
#include <rj_msgs/msg/detection_ball.hpp>
#include <vector>

namespace vision_filter {
using DetectionBallMsg = rj_msgs::msg::DetectionBall;

/**
 * Wrapper for the protobuf observation
 */
class CameraBall {
public:
    /**
     * @param time_captured Time that the picture was taken
     * @param pos Position of the ball at that time
     */
    CameraBall(RJ::Time time_captured, rj_geometry::Point pos)
        : time_captured_(time_captured), pos_(pos) {}

    /**
     * @brief Constructor from DetectionBallMsg.
     * @param msg
     */
    CameraBall(RJ::Time time_captured, const DetectionBallMsg& msg,
               const rj_geometry::TransformMatrix& world_to_team);

    /**
     * @return Time this measurement was captured
     */
    RJ::Time get_time_captured() const;

    /**
     * @return Position of the measurement
     */
    rj_geometry::Point get_pos() const;

    /**
     * Combines all the balls in the list and returns a ball
     * with the average pos and time
     *
     * @param balls The list of balls to combine
     */
    static CameraBall combine_balls(const std::vector<CameraBall>& balls);

private:
    RJ::Time time_captured_;
    rj_geometry::Point pos_;
};
}  // namespace vision_filter
