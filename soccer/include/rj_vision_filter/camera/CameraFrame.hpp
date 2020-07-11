#pragma once
#include <rj_common/Utils.hpp>
#include <rj_msgs/msg/detection_frame.hpp>
#include <utility>
#include <vector>

#include <rj_vision_filter/ball/CameraBall.hpp>
#include <rj_vision_filter/robot/CameraRobot.hpp>

using DetectionFrameMsg = rj_msgs::msg::DetectionFrame;

/**
 * Simple non-protobuf object representing a camera frame
 */
class CameraFrame {
public:
    /**
     * Creates a camera frame directly from the protobuf packets
     *
     * @param tCapture Time the frame was captured
     * @param cameraID ID of the camera this frame corresponds to
     * @param cameraBalls Unsorted list of ball detections
     * @param cameraRobotsYellow Unsorted list of yellow team robot detections
     * @param cameraRobotsBlue Unsorted list of blue team robot detections
     */
    CameraFrame(RJ::Time tCapture, int cameraID,
                std::vector<CameraBall> cameraBalls,
                std::vector<CameraRobot> cameraRobotsYellow,
                std::vector<CameraRobot> cameraRobotsBlue)
        : tCapture(tCapture),
          cameraID(cameraID),
          cameraBalls(std::move(cameraBalls)),
          cameraRobotsYellow(std::move(cameraRobotsYellow)),
          cameraRobotsBlue(std::move(cameraRobotsBlue)) {}

    /**
     * @brief Constructor from DetectionFrameMsg.
     * @param msg
     */
    CameraFrame(const DetectionFrameMsg& msg,
                const Geometry2d::TransformMatrix& world_to_team,
                double team_angle)
        : tCapture{RJ::FromROSTime(msg.t_capture)},
          cameraID{static_cast<int>(msg.camera_id)},
          cameraBalls{},
          cameraRobotsYellow{},
          cameraRobotsBlue{} {
        for (const DetectionBallMsg& ball_msg : msg.balls) {
            cameraBalls.emplace_back(tCapture, ball_msg, world_to_team);
        }

        for (const DetectionRobotMsg& robot_msg : msg.robots_blue) {
            cameraRobotsBlue.emplace_back(tCapture, robot_msg, world_to_team,
                                          team_angle);
        }
        for (const DetectionRobotMsg& robot_msg : msg.robots_yellow) {
            cameraRobotsYellow.emplace_back(tCapture, robot_msg, world_to_team,
                                            team_angle);
        }
    }

    RJ::Time tCapture;
    int cameraID;
    std::vector<CameraBall> cameraBalls;
    std::vector<CameraRobot> cameraRobotsYellow;
    std::vector<CameraRobot> cameraRobotsBlue;
};
