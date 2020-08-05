#include <Geometry2d/Point.hpp>
#include <rj_constants/constants.hpp>
#include <rj_vision_filter/camera/Camera.hpp>
#include <rj_vision_filter/params.hpp>

namespace vision_filter {

DEFINE_NS_FLOAT64(
    kVisionFilterParamModule, camera, mhkf_radius_cutoff, 0.5,
    "The cutoff radius for when to associate measurements to Kalman objects.")
DEFINE_NS_BOOL(kVisionFilterParamModule, camera, use_mhkf, true,
               "Whether to use MHKF or AKF.")
DEFINE_NS_INT64(kVisionFilterParamModule, camera, max_num_kalman_balls, 10,
                "Max number of Kalman balls for this specific camera.")
DEFINE_NS_INT64(
    kVisionFilterParamModule, camera, max_num_kalman_robots, 10,
    "Max number of Kalman robots for each robot id for this specific camera.")
using namespace camera;

Camera::Camera() : isValid(false) {}

Camera::Camera(int camera_id)
    : isValid(true),
      cameraID(camera_id),
      kalmanRobotYellowList(Num_Shells),
      kalmanRobotBlueList(Num_Shells) {}

bool Camera::getIsValid() const { return isValid; }

void Camera::processBallBounce(const std::vector<WorldRobot>& yellow_robots,
                               const std::vector<WorldRobot>& blue_robots) {
    for (KalmanBall& b : kalmanBallList) {
        Geometry2d::Point new_vel;
        bool is_collision =
            BallBounce::CalcBallBounce(b, yellow_robots, blue_robots, new_vel);

        if (is_collision) {
            b.setVel(new_vel);
        }
    }
}

void Camera::updateWithFrame(
    RJ::Time calc_time, const std::vector<CameraBall>& ball_list,
    const std::vector<std::list<CameraRobot>>& yellow_robot_list,
    const std::vector<std::list<CameraRobot>>& blue_robot_list,
    const WorldBall& previous_world_ball,
    const std::vector<WorldRobot>& previous_yellow_world_robots,
    const std::vector<WorldRobot>& previous_blue_world_robots) {
    // Prune list of balls and robots before doing anything
    removeInvalidBalls();
    removeInvalidRobots();

    updateBalls(calc_time, ball_list, previous_world_ball);
    updateRobots(calc_time, yellow_robot_list, blue_robot_list,
                 previous_yellow_world_robots, previous_blue_world_robots);
}

void Camera::updateWithoutFrame(RJ::Time calc_time) {
    removeInvalidBalls();
    removeInvalidRobots();

    for (KalmanBall& b : kalmanBallList) {
        b.predict(calc_time);
    }

    for (std::list<KalmanRobot>& robot_list : kalmanRobotYellowList) {
        for (KalmanRobot& robot : robot_list) {
            robot.predict(calc_time);
        }
    }

    for (std::list<KalmanRobot>& robot_list : kalmanRobotBlueList) {
        for (KalmanRobot& robot : robot_list) {
            robot.predict(calc_time);
        }
    }
}

void Camera::updateBalls(RJ::Time calc_time,
                         const std::vector<CameraBall>& ball_list,
                         const WorldBall& previous_world_ball) {
    // Make sure there are actually balls in the measurement
    // and only predict if that's the case
    if (ball_list.empty()) {
        for (KalmanBall& b : kalmanBallList) {
            b.predict(calc_time);
        }

        return;
    }

    // We have some balls, so choose which updater to use
    if (PARAM_use_mhkf) {
        updateBallsMHKF(calc_time, ball_list, previous_world_ball);
    } else {
        updateBallsAKF(calc_time, ball_list, previous_world_ball);
    }
}

void Camera::updateBallsMHKF(RJ::Time calc_time,
                             const std::vector<CameraBall>& ball_list,
                             const WorldBall& previous_world_ball) {
    // If we have no existing filters, create a new one from average of
    // everything Easier than trying to figure out which ones are more than X
    // meters away from each other Only delays the filter collection by a camera
    // frame or two
    if (kalmanBallList.empty()) {
        CameraBall avg_ball = CameraBall::CombineBalls(ball_list);
        kalmanBallList.emplace_back(cameraID, calc_time, avg_ball,
                                    previous_world_ball);

        return;
    }

    // TODO: Try merging some of the kalman filters together

    // Create list of bools corresponding to whether we have used this ball
    // as measurement yet
    std::vector<bool> used_camera_ball(ball_list.size(), false);

    // Which camera balls to apply to which kalman ball
    std::vector<std::vector<CameraBall>> applied_balls_list(
        kalmanBallList.size());

    // Figure out which measurements go with which kalman balls
    int kalman_ball_idx = 0;
    for (KalmanBall& kalman_ball : kalmanBallList) {
        std::vector<CameraBall>& measurement_balls =
            applied_balls_list.at(kalman_ball_idx);

        int camera_ball_idx = 0;
        for (const CameraBall& camera_ball : ball_list) {
            double dist = (kalman_ball.getPos() - camera_ball.getPos()).mag();

            // Increase the distance of our cutoff by the velocity
            // This is so the ball doesn't move outside the kalman filter
            // position radius when the ball instantly stops (like in sim)
            if (dist < PARAM_mhkf_radius_cutoff + kalman_ball.getVel().mag()) {
                measurement_balls.push_back(camera_ball);
                used_camera_ball.at(camera_ball_idx) = true;
            }
            camera_ball_idx++;
        }

        kalman_ball_idx++;
    }

    // Apply the ball measurements to the kalman filters
    kalman_ball_idx = 0;
    for (KalmanBall& kalman_ball : kalmanBallList) {
        std::vector<CameraBall>& measurement_balls =
            applied_balls_list.at(kalman_ball_idx);

        // We had at least one measurement near this ball
        if (!measurement_balls.empty()) {
            CameraBall avg_ball = CameraBall::CombineBalls(measurement_balls);
            kalman_ball.predictAndUpdate(calc_time, avg_ball);

            // There aren't any measurements so just predict
        } else {
            kalman_ball.predict(calc_time);
        }
    }

    // Any balls not used, create a kalman ball at that position
    //
    // If there are two measurements which too far away from any
    // current kalman filter, the two measurements will be form
    // two individual kalman filters intead of a single one
    // with an update.
    // A slight delay in will most likely be seen in these cases
    for (int i = 0; i < ball_list.size(); i++) {
        const CameraBall& camera_ball = ball_list.at(i);
        bool was_used = used_camera_ball.at(i);

        if (!was_used && kalmanBallList.size() < PARAM_max_num_kalman_balls) {
            kalmanBallList.emplace_back(cameraID, calc_time, camera_ball,
                                        previous_world_ball);
        }
    }
}

void Camera::updateBallsAKF(RJ::Time calc_time,
                            const std::vector<CameraBall>& ball_list,
                            const WorldBall& previous_world_ball) {
    // Average everything and add as measuremnet
    CameraBall avg_ball = CameraBall::CombineBalls(ball_list);

    // If we have no existing filters, create a new one from average of
    // everything
    if (kalmanBallList.empty()) {
        kalmanBallList.emplace_back(cameraID, calc_time, avg_ball,
                                    previous_world_ball);

        return;
    }

    // Kinda cheating, but we are only keeping a single element in the list
    kalmanBallList.front().predictAndUpdate(calc_time, avg_ball);
}

void Camera::updateRobots(
    RJ::Time calc_time,
    const std::vector<std::list<CameraRobot>>& yellow_robot_list,
    const std::vector<std::list<CameraRobot>>& blue_robot_list,
    const std::vector<WorldRobot>& previous_yellow_world_robots,
    const std::vector<WorldRobot>& previous_blue_world_robots) {
    for (int i = 0; i < Num_Shells; i++) {
        const std::list<CameraRobot>& single_yellow_robot_list =
            yellow_robot_list.at(i);
        const std::list<CameraRobot>& single_blue_robot_list =
            blue_robot_list.at(i);

        // Make sure we actually have robots for the yellow team
        if (single_yellow_robot_list.empty()) {
            for (KalmanRobot& robot : kalmanRobotYellowList.at(i)) {
                robot.predict(calc_time);
            }

            // If we do, do the fancy updates
        } else {
            if (PARAM_use_mhkf) {
                updateRobotsMHKF(calc_time, single_yellow_robot_list,
                                 previous_yellow_world_robots.at(i),
                                 kalmanRobotYellowList.at(i));
            } else {
                updateRobotsAKF(calc_time, single_yellow_robot_list,
                                previous_yellow_world_robots.at(i),
                                kalmanRobotYellowList.at(i));
            }
        }

        // Make sure we actually have robots for the blue team
        if (single_blue_robot_list.empty()) {
            for (KalmanRobot& robot : kalmanRobotBlueList.at(i)) {
                robot.predict(calc_time);
            }

            // If we do, do the fancy updates
        } else {
            if (PARAM_use_mhkf) {
                updateRobotsMHKF(calc_time, single_blue_robot_list,
                                 previous_blue_world_robots.at(i),
                                 kalmanRobotBlueList.at(i));
            } else {
                updateRobotsAKF(calc_time, single_blue_robot_list,
                                previous_blue_world_robots.at(i),
                                kalmanRobotBlueList.at(i));
            }
        }
    }
}

void Camera::updateRobotsMHKF(
    RJ::Time calc_time, const std::list<CameraRobot>& single_robot_list,
    const WorldRobot& previous_world_robot,
    std::list<KalmanRobot>& single_kalman_robot_list) {
    // If we have no existing filters, create a new one from average of
    // everything Easier than trying to figure out which ones are more than X
    // meters away from each other Only delays the filter collection by a camera
    // frame or two
    if (single_kalman_robot_list.empty()) {
        CameraRobot avg_robot = CameraRobot::CombineRobots(single_robot_list);
        single_kalman_robot_list.emplace_back(cameraID, calc_time, avg_robot,
                                              previous_world_robot);

        return;
    }

    // TODO: Merge some of the kalman filters together

    // Create list of bools corresponding to whether we have used this Robot
    // as measurement yet
    std::vector<bool> used_camera_robot(single_robot_list.size(), false);

    // Which camera robots to apply to which kalman Robot
    std::vector<std::list<CameraRobot>> applied_robots_list(
        single_kalman_robot_list.size());

    // Apply camera robots to different kalman filters based off a fixed
    // distance A single camera robots can go to multiple different kalman
    // robots
    int kalman_robot_idx = 0;
    for (KalmanRobot& kalman_robot : single_kalman_robot_list) {
        std::list<CameraRobot>& measurement_robot =
            applied_robots_list.at(kalman_robot_idx);

        int camera_robot_idx = 0;
        for (const CameraRobot& camera_robot : single_robot_list) {
            double dist = (kalman_robot.getPos() - camera_robot.getPos()).mag();

            // Increase the distance of our cutoff by the velocity
            // This is so the robot doesn't move outside the kalman filter
            // position radius when the robot instantly stops (like in sim)
            if (dist < PARAM_mhkf_radius_cutoff + kalman_robot.getVel().mag()) {
                measurement_robot.push_back(camera_robot);
                used_camera_robot.at(camera_robot_idx) = true;
            }
            camera_robot_idx++;
        }

        kalman_robot_idx++;
    }

    // Predict and update the filters based on measurements
    kalman_robot_idx = 0;
    for (KalmanRobot& kalman_robot : single_kalman_robot_list) {
        std::list<CameraRobot>& measurement_robots =
            applied_robots_list.at(kalman_robot_idx);

        // We had at least one measurement near this Robot
        if (!measurement_robots.empty()) {
            CameraRobot avg_robot =
                CameraRobot::CombineRobots(measurement_robots);
            kalman_robot.predictAndUpdate(calc_time, avg_robot);

            // There aren't any measurements so just predict
        } else {
            kalman_robot.predict(calc_time);
        }
    }

    // Create kalman robots if one isn't near camera measurement
    int camera_robot_idx = 0;
    for (const CameraRobot& camera_robot : single_robot_list) {
        bool was_used = used_camera_robot.at(camera_robot_idx);

        if (!was_used &&
            single_kalman_robot_list.size() < PARAM_max_num_kalman_robots) {
            single_kalman_robot_list.emplace_back(
                cameraID, calc_time, camera_robot, previous_world_robot);
        }

        camera_robot_idx++;
    }
}

void Camera::updateRobotsAKF(RJ::Time calc_time,
                             const std::list<CameraRobot>& single_robot_list,
                             const WorldRobot& previous_world_robot,
                             std::list<KalmanRobot>& single_kalman_robot_list) {
    // Average everything and add as measuremnet
    CameraRobot avg_robot = CameraRobot::CombineRobots(single_robot_list);

    // If we have no existing filters, create a new one from average of
    // everything
    if (single_kalman_robot_list.empty()) {
        single_kalman_robot_list.emplace_back(cameraID, calc_time, avg_robot,
                                              previous_world_robot);

        return;
    }

    // Kinda cheating, but we are only keeping a single element in the list
    single_kalman_robot_list.front().predictAndUpdate(calc_time, avg_robot);
}

void Camera::removeInvalidBalls() {
    // Remove all balls that are unhealthy
    kalmanBallList.remove_if([](KalmanBall& b) { return b.isUnhealthy(); });
}

void Camera::removeInvalidRobots() {
    // Remove all the robots that are unhealthy
    for (std::list<KalmanRobot>& robot_list : kalmanRobotBlueList) {
        robot_list.remove_if([](KalmanRobot& r) { return r.isUnhealthy(); });
    }

    for (std::list<KalmanRobot>& robot_list : kalmanRobotYellowList) {
        robot_list.remove_if([](KalmanRobot& r) { return r.isUnhealthy(); });
    }
}

void Camera::predictAllRobots(
    RJ::Time calc_time, std::vector<std::list<KalmanRobot>>& robot_list_list) {
    for (std::list<KalmanRobot>& robot_list : robot_list_list) {
        for (KalmanRobot& robot : robot_list) {
            robot.predict(calc_time);
        }
    }
}

const std::list<KalmanBall>& Camera::getKalmanBalls() const {
    return kalmanBallList;
}

const std::vector<std::list<KalmanRobot>>& Camera::getKalmanRobotsYellow()
    const {
    return kalmanRobotYellowList;
}

const std::vector<std::list<KalmanRobot>>& Camera::getKalmanRobotsBlue() const {
    return kalmanRobotBlueList;
}
}  // namespace vision_filter
