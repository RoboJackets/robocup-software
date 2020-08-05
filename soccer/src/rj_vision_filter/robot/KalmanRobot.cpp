#include <rj_vision_filter/robot/KalmanRobot.hpp>

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <iostream>

#include <rj_vision_filter/params.hpp>
#include <rj_vision_filter/robot/WorldRobot.hpp>

namespace vision_filter {
DEFINE_NS_FLOAT64(
    kVisionFilterParamModule, kalman_robot, max_time_outside_vision, 0.5,
    "Max number of seconds without a measurement before the object is deleted")
using kalman_robot::PARAM_max_time_outside_vision;

KalmanRobot::KalmanRobot(unsigned int camera_id, RJ::Time creation_time,
                         CameraRobot init_measurement,
                         const WorldRobot& previous_world_robot)
    : cameraID(camera_id),
      health(filter::health::PARAM_init),
      lastUpdateTime(creation_time),
      lastPredictTime(creation_time),
      unwrapThetaCtr(0),
      robotID(init_measurement.getRobotID()),
      previousMeasurements(kick::detector::PARAM_slow_kick_hist_length) {
    Geometry2d::Pose init_pose = init_measurement.getPose();
    Geometry2d::Twist init_twist(0, 0, 0);

    if (previous_world_robot.getIsValid()) {
        init_twist.linear() = previous_world_robot.getVel();
        init_twist.angular() = previous_world_robot.getOmega();
    }

    filter = KalmanFilter3D(init_pose, init_twist);

    previousMeasurements.push_back(init_measurement);
    previousTheta = init_twist.angular();
}

void KalmanRobot::predict(RJ::Time current_time) {
    lastPredictTime = current_time;

    // Decrement but make sure you don't go too low
    health =
        std::max(health - filter::health::PARAM_dec, filter::health::PARAM_min);

    filter.predict();
}

void KalmanRobot::predictAndUpdate(RJ::Time current_time,
                                   CameraRobot update_robot) {
    lastPredictTime = current_time;
    lastUpdateTime = current_time;

    // Increment but make sure you don't go too high
    health =
        std::min(health + filter::health::PARAM_inc, filter::health::PARAM_max);

    // Keep last X camera observations in list for kick detection and filtering
    previousMeasurements.push_back(update_robot);

    // Unwrap theta so we have a continuous heading
    double cur_theta = update_robot.getTheta();

    // See if it went below -pi
    // Note: PI/2 is used to give a good buffer on either side
    if (previousTheta < -M_PI_2 && cur_theta > M_PI_2) {
        unwrapThetaCtr--;
        // Went above pi
    } else if (previousTheta > M_PI_2 && cur_theta < -M_PI_2) {
        unwrapThetaCtr++;
    }

    previousTheta = cur_theta;

    filter.predictWithUpdate(
        {update_robot.getPos(), cur_theta + unwrapThetaCtr * 2 * M_PI});
}

bool KalmanRobot::isUnhealthy() const {
    bool updated_recently = RJ::Seconds(lastPredictTime - lastUpdateTime) <
                            RJ::Seconds(PARAM_max_time_outside_vision);

    return !updated_recently;
}

unsigned int KalmanRobot::getCameraID() const { return cameraID; }

int KalmanRobot::getRobotID() const { return robotID; }

int KalmanRobot::getHealth() const { return health; }

Geometry2d::Point KalmanRobot::getPos() const { return filter.getPos(); }

double KalmanRobot::getTheta() const { return filter.getTheta(); }

Geometry2d::Point KalmanRobot::getVel() const { return filter.getVel(); }

double KalmanRobot::getOmega() const { return filter.getOmega(); }

Geometry2d::Point KalmanRobot::getPosCov() const { return filter.getPosCov(); }

double KalmanRobot::getThetaCov() const { return filter.getThetaCov(); }

Geometry2d::Point KalmanRobot::getVelCov() const { return filter.getVelCov(); }

double KalmanRobot::getOmegaCov() const { return filter.getOmegaCov(); }

const boost::circular_buffer<CameraRobot>& KalmanRobot::getPrevMeasurements()
    const {
    return previousMeasurements;
}
}  // namespace vision_filter