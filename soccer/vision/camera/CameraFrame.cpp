#include "CameraFrame.hpp"

CameraFrame::CameraFrame(int frameNumber,
                         RJ::Time tCapture,
                         int cameraID,
                         std::vector<CameraBall> cameraBalls,
                         std::vector<CameraRobot> cameraRobotsYellow,
                         std::vector<CameraRobot> cameraRobotsBlue)
    : frameNumber(frameNumber), tCapture(tCapture), cameraID(cameraID),
      cameraBalls(cameraBalls), cameraRobotsYellow(cameraRobotsYellow),
      cameraRobotsBlue(cameraRobotsBlue) {}

int CameraFrame::getFrameNumber() {
    return frameNumber;
}

RJ::Time CameraFrame::getTCapture() {
    return tCapture;
}
int CameraFrame::getCameraID() {
    return cameraID;
}

std::vector<CameraBall> CameraFrame::getCameraBalls() {
    return cameraBalls;
}

std::vector<CameraRobot> CameraFrame::getCameraRobotsYellow() {
    return cameraRobotsYellow;
}

std::vector<CameraRobot> CameraFrame::getCameraRobotsBlue() {
    return cameraRobotsBlue;
}