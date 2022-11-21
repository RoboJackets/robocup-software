#include <rj_geometry/point.hpp>
#include <rj_constants/constants.hpp>
#include <rj_vision_filter/camera/camera.hpp>
#include <rj_vision_filter/params.hpp>

namespace vision_filter {

DEFINE_NS_FLOAT64(kVisionFilterParamModule, camera, mhkf_radius_cutoff, 0.5,
                  "The cutoff radius for when to associate measurements to Kalman objects.")
DEFINE_NS_BOOL(kVisionFilterParamModule, camera, use_mhkf, true, "Whether to use MHKF or AKF.")
DEFINE_NS_INT64(kVisionFilterParamModule, camera, max_num_kalman_balls, 10,
                "Max number of Kalman balls for this specific camera.")
DEFINE_NS_INT64(kVisionFilterParamModule, camera, max_num_kalman_robots, 10,
                "Max number of Kalman robots for each robot id for this specific camera.")
using namespace camera;

Camera::Camera() : is_valid_(false) {}

Camera::Camera(int camera_id)
    : is_valid_(true),
      camera_id_(camera_id),
      kalman_robot_yellow_list_(kNumShells),
      kalman_robot_blue_list_(kNumShells) {}

bool Camera::get_is_valid() const { return is_valid_; }

void Camera::process_ball_bounce(const std::vector<WorldRobot>& yellow_robots,
                                 const std::vector<WorldRobot>& blue_robots) {
    for (KalmanBall& b : kalman_ball_list_) {
        rj_geometry::Point new_vel;
        bool is_collision = BallBounce::calc_ball_bounce(b, yellow_robots, blue_robots, new_vel);

        if (is_collision) {
            b.set_vel(new_vel);
        }
    }
}

void Camera::update_with_frame(RJ::Time calc_time, const std::vector<CameraBall>& ball_list,
                               const std::vector<std::list<CameraRobot>>& yellow_robot_list,
                               const std::vector<std::list<CameraRobot>>& blue_robot_list,
                               const WorldBall& previous_world_ball,
                               const std::vector<WorldRobot>& previous_yellow_world_robots,
                               const std::vector<WorldRobot>& previous_blue_world_robots) {
    // Prune list of balls and robots before doing anything
    remove_invalid_balls();
    remove_invalid_robots();

    update_balls(calc_time, ball_list, previous_world_ball);
    update_robots(calc_time, yellow_robot_list, blue_robot_list, previous_yellow_world_robots,
                  previous_blue_world_robots);
}

void Camera::update_without_frame(RJ::Time calc_time) {
    remove_invalid_balls();
    remove_invalid_robots();

    for (KalmanBall& b : kalman_ball_list_) {
        b.predict(calc_time);
    }

    for (std::list<KalmanRobot>& robot_list : kalman_robot_yellow_list_) {
        for (KalmanRobot& robot : robot_list) {
            robot.predict(calc_time);
        }
    }

    for (std::list<KalmanRobot>& robot_list : kalman_robot_blue_list_) {
        for (KalmanRobot& robot : robot_list) {
            robot.predict(calc_time);
        }
    }
}

void Camera::update_balls(RJ::Time calc_time, const std::vector<CameraBall>& ball_list,
                          const WorldBall& previous_world_ball) {
    // Make sure there are actually balls in the measurement
    // and only predict if that's the case
    if (ball_list.empty()) {
        for (KalmanBall& b : kalman_ball_list_) {
            b.predict(calc_time);
        }

        return;
    }

    // We have some balls, so choose which updater to use
    if (PARAM_use_mhkf) {
        update_balls_mhkf(calc_time, ball_list, previous_world_ball);
    } else {
        update_balls_akf(calc_time, ball_list, previous_world_ball);
    }
}

void Camera::update_balls_mhkf(RJ::Time calc_time, const std::vector<CameraBall>& ball_list,
                               const WorldBall& previous_world_ball) {
    // If we have no existing filters, create a new one from average of
    // everything Easier than trying to figure out which ones are more than X
    // meters away from each other Only delays the filter collection by a camera
    // frame or two
    if (kalman_ball_list_.empty()) {
        CameraBall avg_ball = CameraBall::combine_balls(ball_list);
        kalman_ball_list_.emplace_back(camera_id_, calc_time, avg_ball, previous_world_ball);

        return;
    }

    // TODO: Try merging some of the kalman filters together

    // Create list of bools corresponding to whether we have used this ball
    // as measurement yet
    std::vector<bool> used_camera_ball(ball_list.size(), false);

    // Which camera balls to apply to which kalman ball
    std::vector<std::vector<CameraBall>> applied_balls_list(kalman_ball_list_.size());

    // Figure out which measurements go with which kalman balls
    int kalman_ball_idx = 0;
    for (KalmanBall& kalman_ball : kalman_ball_list_) {
        std::vector<CameraBall>& measurement_balls = applied_balls_list.at(kalman_ball_idx);

        int camera_ball_idx = 0;
        for (const CameraBall& camera_ball : ball_list) {
            double dist = (kalman_ball.get_pos() - camera_ball.get_pos()).mag();

            // Increase the distance of our cutoff by the velocity
            // This is so the ball doesn't move outside the kalman filter
            // position radius when the ball instantly stops (like in sim)
            if (dist < PARAM_mhkf_radius_cutoff + kalman_ball.get_vel().mag()) {
                measurement_balls.push_back(camera_ball);
                used_camera_ball.at(camera_ball_idx) = true;
            }
            camera_ball_idx++;
        }

        kalman_ball_idx++;
    }

    // Apply the ball measurements to the kalman filters
    kalman_ball_idx = 0;
    for (KalmanBall& kalman_ball : kalman_ball_list_) {
        std::vector<CameraBall>& measurement_balls = applied_balls_list.at(kalman_ball_idx);

        // We had at least one measurement near this ball
        if (!measurement_balls.empty()) {
            CameraBall avg_ball = CameraBall::combine_balls(measurement_balls);
            kalman_ball.predict_and_update(calc_time, avg_ball);

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
    for (unsigned long i = 0; i < ball_list.size(); i++) {
        const CameraBall& camera_ball = ball_list.at(i);
        bool was_used = used_camera_ball.at(i);

        if (!was_used && kalman_ball_list_.size() < PARAM_max_num_kalman_balls) {
            kalman_ball_list_.emplace_back(camera_id_, calc_time, camera_ball, previous_world_ball);
        }
    }
}

void Camera::update_balls_akf(RJ::Time calc_time, const std::vector<CameraBall>& ball_list,
                              const WorldBall& previous_world_ball) {
    // Average everything and add as measuremnet
    CameraBall avg_ball = CameraBall::combine_balls(ball_list);

    // If we have no existing filters, create a new one from average of
    // everything
    if (kalman_ball_list_.empty()) {
        kalman_ball_list_.emplace_back(camera_id_, calc_time, avg_ball, previous_world_ball);

        return;
    }

    // Kinda cheating, but we are only keeping a single element in the list
    kalman_ball_list_.front().predict_and_update(calc_time, avg_ball);
}

void Camera::update_robots(RJ::Time calc_time,
                           const std::vector<std::list<CameraRobot>>& yellow_robot_list,
                           const std::vector<std::list<CameraRobot>>& blue_robot_list,
                           const std::vector<WorldRobot>& previous_yellow_world_robots,
                           const std::vector<WorldRobot>& previous_blue_world_robots) {
    for (unsigned long i = 0; i < kNumShells; i++) {
        const std::list<CameraRobot>& single_yellow_robot_list = yellow_robot_list.at(i);
        const std::list<CameraRobot>& single_blue_robot_list = blue_robot_list.at(i);

        // Make sure we actually have robots for the yellow team
        if (single_yellow_robot_list.empty()) {
            for (KalmanRobot& robot : kalman_robot_yellow_list_.at(i)) {
                robot.predict(calc_time);
            }

            // If we do, do the fancy updates
        } else {
            if (PARAM_use_mhkf) {
                update_robots_mhkf(calc_time, single_yellow_robot_list,
                                   previous_yellow_world_robots.at(i),
                                   kalman_robot_yellow_list_.at(i));
            } else {
                update_robots_akf(calc_time, single_yellow_robot_list,
                                  previous_yellow_world_robots.at(i),
                                  kalman_robot_yellow_list_.at(i));
            }
        }

        // Make sure we actually have robots for the blue team
        if (single_blue_robot_list.empty()) {
            for (KalmanRobot& robot : kalman_robot_blue_list_.at(i)) {
                robot.predict(calc_time);
            }

            // If we do, do the fancy updates
        } else {
            if (PARAM_use_mhkf) {
                update_robots_mhkf(calc_time, single_blue_robot_list,
                                   previous_blue_world_robots.at(i), kalman_robot_blue_list_.at(i));
            } else {
                update_robots_akf(calc_time, single_blue_robot_list,
                                  previous_blue_world_robots.at(i), kalman_robot_blue_list_.at(i));
            }
        }
    }
}

void Camera::update_robots_mhkf(RJ::Time calc_time, const std::list<CameraRobot>& single_robot_list,
                                const WorldRobot& previous_world_robot,
                                std::list<KalmanRobot>& single_kalman_robot_list) {
    // If we have no existing filters, create a new one from average of
    // everything Easier than trying to figure out which ones are more than X
    // meters away from each other Only delays the filter collection by a camera
    // frame or two
    if (single_kalman_robot_list.empty()) {
        CameraRobot avg_robot = CameraRobot::combine_robots(single_robot_list);
        single_kalman_robot_list.emplace_back(camera_id_, calc_time, avg_robot,
                                              previous_world_robot);

        return;
    }

    // TODO: Merge some of the kalman filters together

    // Create list of bools corresponding to whether we have used this Robot
    // as measurement yet
    std::vector<bool> used_camera_robot(single_robot_list.size(), false);

    // Which camera robots to apply to which kalman Robot
    std::vector<std::list<CameraRobot>> applied_robots_list(single_kalman_robot_list.size());

    // Apply camera robots to different kalman filters based off a fixed
    // distance A single camera robots can go to multiple different kalman
    // robots
    int kalman_robot_idx = 0;
    for (KalmanRobot& kalman_robot : single_kalman_robot_list) {
        std::list<CameraRobot>& measurement_robot = applied_robots_list.at(kalman_robot_idx);

        int camera_robot_idx = 0;
        for (const CameraRobot& camera_robot : single_robot_list) {
            double dist = (kalman_robot.get_pos() - camera_robot.get_pos()).mag();

            // Increase the distance of our cutoff by the velocity
            // This is so the robot doesn't move outside the kalman filter
            // position radius when the robot instantly stops (like in sim)
            if (dist < PARAM_mhkf_radius_cutoff + kalman_robot.get_vel().mag()) {
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
        std::list<CameraRobot>& measurement_robots = applied_robots_list.at(kalman_robot_idx);

        // We had at least one measurement near this Robot
        if (!measurement_robots.empty()) {
            CameraRobot avg_robot = CameraRobot::combine_robots(measurement_robots);
            kalman_robot.predict_and_update(calc_time, avg_robot);

            // There aren't any measurements so just predict
        } else {
            kalman_robot.predict(calc_time);
        }
    }

    // Create kalman robots if one isn't near camera measurement
    int camera_robot_idx = 0;
    for (const CameraRobot& camera_robot : single_robot_list) {
        bool was_used = used_camera_robot.at(camera_robot_idx);

        if (!was_used && single_kalman_robot_list.size() < (unsigned long) PARAM_max_num_kalman_robots) {
            single_kalman_robot_list.emplace_back(camera_id_, calc_time, camera_robot,
                                                  previous_world_robot);
        }

        camera_robot_idx++;
    }
}

void Camera::update_robots_akf(RJ::Time calc_time, const std::list<CameraRobot>& single_robot_list,
                               const WorldRobot& previous_world_robot,
                               std::list<KalmanRobot>& single_kalman_robot_list) {
    // Average everything and add as measuremnet
    CameraRobot avg_robot = CameraRobot::combine_robots(single_robot_list);

    // If we have no existing filters, create a new one from average of
    // everything
    if (single_kalman_robot_list.empty()) {
        single_kalman_robot_list.emplace_back(camera_id_, calc_time, avg_robot,
                                              previous_world_robot);

        return;
    }

    // Kinda cheating, but we are only keeping a single element in the list
    single_kalman_robot_list.front().predict_and_update(calc_time, avg_robot);
}

void Camera::remove_invalid_balls() {
    // Remove all balls that are unhealthy
    kalman_ball_list_.remove_if([](KalmanBall& b) { return b.is_unhealthy(); });
}

void Camera::remove_invalid_robots() {
    // Remove all the robots that are unhealthy
    for (std::list<KalmanRobot>& robot_list : kalman_robot_blue_list_) {
        robot_list.remove_if([](KalmanRobot& r) { return r.is_unhealthy(); });
    }

    for (std::list<KalmanRobot>& robot_list : kalman_robot_yellow_list_) {
        robot_list.remove_if([](KalmanRobot& r) { return r.is_unhealthy(); });
    }
}

void Camera::predict_all_robots(RJ::Time calc_time,
                                std::vector<std::list<KalmanRobot>>& robot_list_list) {
    for (std::list<KalmanRobot>& robot_list : robot_list_list) {
        for (KalmanRobot& robot : robot_list) {
            robot.predict(calc_time);
        }
    }
}

const std::list<KalmanBall>& Camera::get_kalman_balls() const { return kalman_ball_list_; }

const std::vector<std::list<KalmanRobot>>& Camera::get_kalman_robots_yellow() const {
    return kalman_robot_yellow_list_;
}

const std::vector<std::list<KalmanRobot>>& Camera::get_kalman_robots_blue() const {
    return kalman_robot_blue_list_;
}
}  // namespace vision_filter
