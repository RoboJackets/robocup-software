#pragma once

#include <Geometry2d/Point.hpp>
#include <Geometry2d/Pose.hpp>
#include <rj_common/Utils.hpp>
#include <rj_msgs/msg/detection_robot.hpp>
#include <vector>

using DetectionRobotMsg = rj_msgs::msg::DetectionRobot;

/**
 * Wrapper for the protobuf observation
 */
class CameraRobot {
public:
    /**
     * @param timeCaptured Time the frame was captured
     * @param pos Position of the robot observation
     * @param theta Heading of the robot observation
     * @param robotID ID of the robot
     */
    CameraRobot(RJ::Time timeCaptured, Geometry2d::Pose pose, int robotID)
        : timeCaptured(timeCaptured), pose(pose), robotID(robotID) {}

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
    RJ::Time getTimeCaptured() const;

    /**
     * @return the position of the measurement
     */
    Geometry2d::Point getPos() const;

    /**
     * @return the heading of the measurement
     */
    double getTheta() const;

    /**
     * @return the robot ID of the measurement
     */
    int getRobotID() const;

    /**
     * @return the pose of the measurement
     */
    Geometry2d::Pose getPose() const;

    /**
     * Combines all the robots in the list and returns a robot
     * with the average pos, time, and theta
     *
     * @param robots The list of robots to combine
     *
     * Note: All robots must have the same robotID
     */
    static CameraRobot CombineRobots(const std::list<CameraRobot>& robots);

private:
    RJ::Time timeCaptured;
    Geometry2d::Pose pose;

    int robotID;
};
