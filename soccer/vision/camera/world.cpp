#include "vision/camera/world.hpp"

#include <rj_constants/constants.hpp>

#include "vision/params.hpp"

namespace vision_filter {
DEFINE_NS_FLOAT64(kVisionFilterParamModule, kick::detector, fast_kick_timeout, 1.0,
                  "Only replace fast kick estimates when this much time has "
                  "passed. In seconds.")
DEFINE_NS_FLOAT64(kVisionFilterParamModule, kick::detector, slow_kick_timeout, 0.5,
                  "Only replace slow kick estimates when this much time has "
                  "passed. In seconds.")
DEFINE_NS_FLOAT64(kVisionFilterParamModule, kick::detector, same_kick_timeout, 0.5,
                  "Only replace fast kick estimate with a slow when the two "
                  "times are within this amount.")
using namespace kick::detector;

World::World()
    : last_update_time_{RJ::Time{RJ::Time::duration(0)}},
      cameras_(PARAM_max_num_cameras),
      robots_yellow_(kNumShells, WorldRobot()),
      robots_blue_(kNumShells, WorldRobot()) {}

void World::update_single_camera(RJ::Time calc_time, const CameraFrame& frame) {
    update_with_camera_frame(calc_time, {frame}, false);
}

void World::update_with_camera_frame(RJ::Time calc_time, const std::vector<CameraFrame>& new_frames,
                                     bool update_all) {
    calc_ball_bounce();

    std::vector<bool> camera_updated(PARAM_max_num_cameras, false);

    // TODO: Take only the newest frame if 2 come in for the same camera

    for (const CameraFrame& frame : new_frames) {
        // Make sure camera from frame is created, if not, make it
        if (!cameras_.at(frame.camera_id).get_is_valid()) {
            cameras_.at(frame.camera_id) = Camera(frame.camera_id);
        }

        // Take the non-sorted list from the frame and make a list for the
        // cameras
        std::vector<std::list<CameraRobot>> yellow_team(kNumShells);
        std::vector<std::list<CameraRobot>> blue_team(kNumShells);

        for (const CameraRobot& robot : frame.camera_robots_yellow) {
            yellow_team.at(robot.get_robot_id()).push_back(robot);
        }

        for (const CameraRobot& robot : frame.camera_robots_blue) {
            blue_team.at(robot.get_robot_id()).push_back(robot);
        }

        cameras_.at(frame.camera_id)
            .update_with_frame(calc_time, frame.camera_balls, yellow_team, blue_team, ball_,
                               robots_yellow_, robots_blue_);

        camera_updated.at(frame.camera_id) = true;

        // Update last_update_time_ with the latest t_capture.
        last_update_time_ = std::max(last_update_time_, frame.t_capture);
    }

    if (update_all) {
        for (size_t i = 0; i < cameras_.size(); i++) {
            if (!camera_updated.at(i) && cameras_.at(i).get_is_valid()) {
                cameras_.at(i).update_without_frame(calc_time);
            }
        }
    }

    update_world_objects(calc_time);
    detect_kicks(calc_time);
}

void World::update_without_camera_frame(RJ::Time calc_time) {
    calc_ball_bounce();

    for (Camera& camera : cameras_) {
        if (camera.get_is_valid()) {
            camera.update_without_frame(calc_time);
        }
    }

    update_world_objects(calc_time);
    detect_kicks(calc_time);
}

void World::calc_ball_bounce() {
    for (Camera& camera : cameras_) {
        if (camera.get_is_valid()) {
            camera.process_ball_bounce(robots_yellow_, robots_blue_);
        }
    }
}

void World::update_world_objects(RJ::Time calc_time) {
    // Fill robots_yellow_/blue with what robots we want and remove the rest
    ball_ = WorldBall();

    std::fill(robots_yellow_.begin(), robots_yellow_.end(), WorldRobot());
    std::fill(robots_blue_.begin(), robots_blue_.end(), WorldRobot());

    std::list<KalmanBall> kalman_balls;
    std::vector<std::list<KalmanRobot>> kalman_robots_yellow(kNumShells);
    std::vector<std::list<KalmanRobot>> kalman_robots_blue(kNumShells);

    // Take best kalman filter from every camera and combine them
    for (Camera& camera : cameras_) {
        if (camera.get_is_valid()) {
            std::list<KalmanBall> camera_balls = camera.get_kalman_balls();
            std::vector<std::list<KalmanRobot>> camera_robots_yellow =
                camera.get_kalman_robots_yellow();
            std::vector<std::list<KalmanRobot>> camera_robots_blue =
                camera.get_kalman_robots_blue();

            if (!camera_balls.empty()) {
                // Sort by health of the kalman filter
                camera_balls.sort([](KalmanBall& a, KalmanBall& b) -> bool {
                    return a.get_health() > b.get_health();
                });

                kalman_balls.push_back(camera_balls.front());
            }

            // Take the best kalman filter from the camera
            for (size_t i = 0; i < camera_robots_yellow.size(); i++) {
                if (!camera_robots_yellow.at(i).empty()) {
                    camera_robots_yellow.at(i).sort([](KalmanRobot& a, KalmanRobot& b) -> bool {
                        return a.get_health() > b.get_health();
                    });

                    kalman_robots_yellow.at(i).push_back(camera_robots_yellow.at(i).front());
                }
            }

            // Take the best kalman filter from the camera
            for (size_t i = 0; i < camera_robots_blue.size(); i++) {
                if (!camera_robots_blue.at(i).empty()) {
                    camera_robots_blue.at(i).sort([](KalmanRobot& a, KalmanRobot& b) -> bool {
                        return a.get_health() > b.get_health();
                    });

                    kalman_robots_blue.at(i).push_back(camera_robots_blue.at(i).front());
                }
            }
        }
    }

    // Only replace the invalid result if we have measurements on any camera
    if (!kalman_balls.empty()) {
        ball_ = WorldBall(calc_time, kalman_balls);
    }

    for (size_t i = 0; i < robots_yellow_.size(); i++) {
        if (!kalman_robots_yellow.at(i).empty()) {
            robots_yellow_.at(i) =
                WorldRobot(calc_time, WorldRobot::Team::YELLOW, i, kalman_robots_yellow.at(i));
        }
    }

    for (size_t i = 0; i < robots_blue_.size(); i++) {
        if (!kalman_robots_blue.at(i).empty()) {
            robots_blue_.at(i) =
                WorldRobot(calc_time, WorldRobot::Team::BLUE, i, kalman_robots_blue.at(i));
        }
    }
}

void World::detect_kicks(RJ::Time calc_time) {
    KickEvent fast_event;
    KickEvent slow_event;

    bool is_fast_kick =
        fast_kick_.add_record(calc_time, ball_, robots_yellow_, robots_blue_, fast_event);
    bool is_slow_kick =
        slow_kick_.add_record(calc_time, ball_, robots_yellow_, robots_blue_, &slow_event);

    // If there isn't a kick recorded already
    if (!best_kick_estimate_.get_is_valid()) {
        // Try to use the slow kick as it's a better estimate
        // but take fast kick if there isn't a corrsponding slow kick yet
        if (is_slow_kick) {
            best_kick_estimate_ = slow_event;
        } else if (is_fast_kick) {
            best_kick_estimate_ = fast_event;
        }

        // There is a kick recorded already
    } else {
        const RJ::Seconds time_since_best_event(best_kick_estimate_.get_kick_time() - calc_time);
        const RJ::Seconds same_kick_timeout(PARAM_same_kick_timeout);
        const RJ::Seconds slow_kick_timeout(PARAM_slow_kick_timeout);
        const RJ::Seconds fast_kick_timeout(PARAM_fast_kick_timeout);

        // Try using the slow kick if:
        //      - It refers to the current best kick event (and probably is a
        //      better estimate)
        //      - The old kick timed out and should be updated
        if (is_slow_kick && (time_since_best_event < same_kick_timeout ||
                             time_since_best_event > slow_kick_timeout)) {
            best_kick_estimate_ = slow_event;

            // Try using the fast kick if the old kick timed out
        } else if (is_fast_kick && time_since_best_event > fast_kick_timeout) {
            best_kick_estimate_ = fast_event;

            // Remove the old kick if it's completely time out
        } else if (time_since_best_event > slow_kick_timeout + fast_kick_timeout) {
            best_kick_estimate_ = KickEvent();
        }
    }
}

const WorldBall& World::get_world_ball() const { return ball_; }

const std::vector<WorldRobot>& World::get_robots_yellow() const { return robots_yellow_; }

const std::vector<WorldRobot>& World::get_robots_blue() const { return robots_blue_; }

const KickEvent& World::get_best_kick_estimate() const { return best_kick_estimate_; }
}  // namespace vision_filter
