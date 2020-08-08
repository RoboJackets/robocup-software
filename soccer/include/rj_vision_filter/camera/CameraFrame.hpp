#pragma once
#include <rj_msgs/msg/detection_frame.hpp>
#include <rj_vision_filter/ball/CameraBall.hpp>
#include <rj_vision_filter/robot/CameraRobot.hpp>
#include <utility>
#include <vector>

namespace vision_filter {
using DetectionFrameMsg = rj_msgs::msg::DetectionFrame;

/**
 * Simple non-protobuf object representing a camera frame
 */
class CameraFrame {
public:
    /**
     * Creates a camera frame directly from the protobuf packets
     *
     * @param t_capture Time the frame was captured
     * @param camera_id ID of the camera this frame corresponds to
     * @param camera_balls Unsorted list of ball detections
     * @param camera_robots_yellow Unsorted list of yellow team robot detections
     * @param camera_robots_blue Unsorted list of blue team robot detections
     */
    CameraFrame(RJ::Time t_capture, int camera_id,
                std::vector<CameraBall> camera_balls,
                std::vector<CameraRobot> camera_robots_yellow,
                std::vector<CameraRobot> camera_robots_blue)
        : t_capture(t_capture),
          camera_id(camera_id),
          camera_balls(std::move(camera_balls)),
          camera_robots_yellow(std::move(camera_robots_yellow)),
          camera_robots_blue(std::move(camera_robots_blue)) {}

    /**
     * @brief Constructor from DetectionFrameMsg.
     * @param msg
     */
    CameraFrame(const DetectionFrameMsg& msg,
                const Geometry2d::TransformMatrix& world_to_team,
                double team_angle)
        : t_capture{rj_convert::convert_from_ros(msg.t_capture)},
          camera_id{static_cast<int>(msg.camera_id)},
          camera_balls{},
          camera_robots_yellow{},
          camera_robots_blue{} {
        for (const DetectionBallMsg& ball_msg : msg.balls) {
            camera_balls.emplace_back(t_capture, ball_msg, world_to_team);
        }

        for (const DetectionRobotMsg& robot_msg : msg.robots_blue) {
            camera_robots_blue.emplace_back(t_capture, robot_msg, world_to_team,
                                          team_angle);
        }
        for (const DetectionRobotMsg& robot_msg : msg.robots_yellow) {
            camera_robots_yellow.emplace_back(t_capture, robot_msg, world_to_team,
                                            team_angle);
        }
    }

    RJ::Time t_capture;
    int camera_id;
    std::vector<CameraBall> camera_balls;
    std::vector<CameraRobot> camera_robots_yellow;
    std::vector<CameraRobot> camera_robots_blue;
};
}  // namespace vision_filter
