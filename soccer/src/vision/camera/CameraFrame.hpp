#pragma once
#include <vector>

#include <rj_common/Utils.hpp>

#include "vision/ball/CameraBall.hpp"
#include "vision/robot/CameraRobot.hpp"

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
    CameraFrame(RJ::Time tCapture,
                int cameraID,
                std::vector<CameraBall> cameraBalls,
                std::vector<CameraRobot> cameraRobotsYellow,
                std::vector<CameraRobot> cameraRobotsBlue)
                : tCapture(tCapture), cameraID(cameraID),
                  cameraBalls(cameraBalls),
                  cameraRobotsYellow(cameraRobotsYellow),
                  cameraRobotsBlue(cameraRobotsBlue) {}

    RJ::Time tCapture;
    int cameraID;
    std::vector<CameraBall> cameraBalls;
    std::vector<CameraRobot> cameraRobotsYellow;
    std::vector<CameraRobot> cameraRobotsBlue;
};
