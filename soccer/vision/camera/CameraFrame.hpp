#pragma once
#include <vector>

#include <Utils.hpp>

#include "vision/ball/CameraBall.hpp"
#include "vision/robot/CameraRobot.hpp"

class CameraFrame {
public:
    /**
     * Creates a camera frame directly from the protobuf packets
     *
     * @param frameNumber Frame number
     * @param tCapture Time the frame was captured
     * @param cameraID ID of the camera this frame corresponds to
     * @param cameraBalls Unsorted list of ball detections
     * @param cameraRobotsYellow Unsorted list of yellow team robot detections
     * @param cameraRobotsBlue Unsorted list of blue team robot detections
     */
    CameraFrame(int frameNumber,
                RJ::Time tCapture,
                int cameraID,
                std::vector<CameraBall> cameraBalls,
                std::vector<CameraRobot> cameraRobotsYellow,
                std::vector<CameraRobot> cameraRobotsBlue);

    int getFrameNumber();
    RJ::Time getTCapture();
    int getCameraID();
    std::vector<CameraBall> getCameraBalls();
    std::vector<CameraRobot> getCameraRobotsYellow();
    std::vector<CameraRobot> getCameraRobotsBlue();

private:
    int frameNumber;
    RJ::Time tCapture;
    int cameraID;
    std::vector<CameraBall> cameraBalls;
    std::vector<CameraRobot> cameraRobotsYellow;
    std::vector<CameraRobot> cameraRobotsBlue;
};