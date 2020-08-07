#pragma once

#include <Geometry2d/Point.hpp>
#include <Geometry2d/Pose.hpp>
#include <rj_common/time.hpp>
#include <rj_msgs/msg/detection_robot.hpp>
#include <vector>

namespace vision_filter {
using DetectionRobotMsg = rj_msgs::msg::DetectionRobot;

/**
 * Wrapper for the protobuf observation
 */
class CameraRobot {
public:
    /**
     * @param time_captured Time the frame was captured
     * @param pos Position of the robot observation
     * @param theta Heading of the robot observation
     * @param robot_id ID of the robot
     */
    CameraRobot(RJ::Time time_captured, Geometry2d::Pose pose, int robot_id)
        : time_captured_(time_captured), pose_(pose), robot_id_(robot_id) {}

    /**
     * Constructor that takes in a DetectionRobotMsg.
     * @param time_captured
     * @param msg
     * @param world_to_team
     * @param team_angle
     */
    CameraRobot(const RJ::Time& time_captured, const DetectionRobotMsg& msg,
                const Geometry2d::TransformMatrix& world_to_team,
                double team_angle);

    /**
     * @return the time the detection was captured
     */
    RJ::Time get_time_captured() const;

    /**
     * @return the position of the measurement
     */
    Geometry2d::Point get_pos() const;

    /**
     * @return the heading of the measurement
     */
    double get_theta() const;

    /**
     * @return the robot ID of the measurement
     */
    int get_robot_id() const;

    /**
     * @return the pose of the measurement
     */
    Geometry2d::Pose get_pose() const;

    /**
     * Combines all the robots in the list and returns a robot
     * with the average pos, time, and theta
     *
     * @param robots The list of robots to combine
     *
     * Note: All robots must have the same robot_id
     */
    static CameraRobot combine_robots(const std::list<CameraRobot>& robots);

private:
    RJ::Time time_captured_;
    Geometry2d::Pose pose_;

    int robot_id_;
};
}  // namespace vision_filter
