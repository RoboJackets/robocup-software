#include <rj_constants/constants.hpp>
#include <rj_vision_filter/camera/World.hpp>
#include <rj_vision_filter/params.hpp>

namespace vision_filter {
DEFINE_NS_FLOAT64(kVisionFilterParamModule, kick::detector, fast_kick_timeout,
                  1.0,
                  "Only replace fast kick estimates when this much time has "
                  "passed. In seconds.")
DEFINE_NS_FLOAT64(kVisionFilterParamModule, kick::detector, slow_kick_timeout,
                  0.5,
                  "Only replace slow kick estimates when this much time has "
                  "passed. In seconds.")
DEFINE_NS_FLOAT64(kVisionFilterParamModule, kick::detector, same_kick_timeout,
                  0.5,
                  "Only replace fast kick estimate with a slow when the two "
                  "times are within this amount.")
using namespace kick::detector;

World::World()
    : last_update_time_{RJ::Time{RJ::Time::duration(0)}},
      cameras(PARAM_max_num_cameras),
      robotsYellow(Num_Shells, WorldRobot()),
      robotsBlue(Num_Shells, WorldRobot()) {}

void World::updateWithCameraFrame(RJ::Time calc_time,
                                  const std::vector<CameraFrame>& new_frames) {
    calcBallBounce();

    std::vector<bool> camera_updated(PARAM_max_num_cameras, false);

    // TODO: Take only the newest frame if 2 come in for the same camera

    for (const CameraFrame& frame : new_frames) {
        // Make sure camera from frame is created, if not, make it
        if (!cameras.at(frame.cameraID).getIsValid()) {
            cameras.at(frame.cameraID) = Camera(frame.cameraID);
        }

        // Take the non-sorted list from the frame and make a list for the
        // cameras
        std::vector<std::list<CameraRobot>> yellow_team(Num_Shells);
        std::vector<std::list<CameraRobot>> blue_team(Num_Shells);

        for (const CameraRobot& robot : frame.cameraRobotsYellow) {
            yellow_team.at(robot.getRobotID()).push_back(robot);
        }

        for (const CameraRobot& robot : frame.cameraRobotsBlue) {
            blue_team.at(robot.getRobotID()).push_back(robot);
        }

        cameras.at(frame.cameraID)
            .updateWithFrame(calc_time, frame.cameraBalls, yellow_team,
                             blue_team, ball, robotsYellow, robotsBlue);

        camera_updated.at(frame.cameraID) = true;

        // Update last_update_time_ with the latest tCapture.
        last_update_time_ = std::max(last_update_time_, frame.tCapture);
    }

    for (int i = 0; i < cameras.size(); i++) {
        if (!camera_updated.at(i) && cameras.at(i).getIsValid()) {
            cameras.at(i).updateWithoutFrame(calc_time);
        }
    }

    updateWorldObjects(calc_time);
    detectKicks(calc_time);
}

void World::updateWithoutCameraFrame(RJ::Time calc_time) {
    calcBallBounce();

    for (Camera& camera : cameras) {
        if (camera.getIsValid()) {
            camera.updateWithoutFrame(calc_time);
        }
    }

    updateWorldObjects(calc_time);
    detectKicks(calc_time);
}

void World::calcBallBounce() {
    for (Camera& camera : cameras) {
        if (camera.getIsValid()) {
            camera.processBallBounce(robotsYellow, robotsBlue);
        }
    }
}

void World::updateWorldObjects(RJ::Time calc_time) {
    // Fill robotsYellow/Blue with what robots we want and remove the rest
    ball = WorldBall();

    std::fill(robotsYellow.begin(), robotsYellow.end(), WorldRobot());
    std::fill(robotsBlue.begin(), robotsBlue.end(), WorldRobot());

    std::list<KalmanBall> kalman_balls;
    std::vector<std::list<KalmanRobot>> kalman_robots_yellow(Num_Shells);
    std::vector<std::list<KalmanRobot>> kalman_robots_blue(Num_Shells);

    // Take best kalman filter from every camera and combine them
    for (Camera& camera : cameras) {
        if (camera.getIsValid()) {
            std::list<KalmanBall> camera_balls = camera.getKalmanBalls();
            std::vector<std::list<KalmanRobot>> camera_robots_yellow =
                camera.getKalmanRobotsYellow();
            std::vector<std::list<KalmanRobot>> camera_robots_blue =
                camera.getKalmanRobotsBlue();

            if (!camera_balls.empty()) {
                // Sort by health of the kalman filter
                camera_balls.sort([](KalmanBall& a, KalmanBall& b) -> bool {
                    return a.getHealth() > b.getHealth();
                });

                kalman_balls.push_back(camera_balls.front());
            }

            // Take the best kalman filter from the camera
            for (int i = 0; i < camera_robots_yellow.size(); i++) {
                if (!camera_robots_yellow.at(i).empty()) {
                    camera_robots_yellow.at(i).sort(
                        [](KalmanRobot& a, KalmanRobot& b) -> bool {
                            return a.getHealth() > b.getHealth();
                        });

                    kalman_robots_yellow.at(i).push_back(
                        camera_robots_yellow.at(i).front());
                }
            }

            // Take the best kalman filter from the camera
            for (int i = 0; i < camera_robots_blue.size(); i++) {
                if (!camera_robots_blue.at(i).empty()) {
                    camera_robots_blue.at(i).sort(
                        [](KalmanRobot& a, KalmanRobot& b) -> bool {
                            return a.getHealth() > b.getHealth();
                        });

                    kalman_robots_blue.at(i).push_back(
                        camera_robots_blue.at(i).front());
                }
            }
        }
    }

    // Only replace the invalid result if we have measurements on any camera
    if (!kalman_balls.empty()) {
        ball = WorldBall(calc_time, kalman_balls);
    }

    for (int i = 0; i < robotsYellow.size(); i++) {
        if (!kalman_robots_yellow.at(i).empty()) {
            robotsYellow.at(i) = WorldRobot(calc_time, WorldRobot::Team::YELLOW,
                                            i, kalman_robots_yellow.at(i));
        }
    }

    for (int i = 0; i < robotsBlue.size(); i++) {
        if (!kalman_robots_blue.at(i).empty()) {
            robotsBlue.at(i) = WorldRobot(calc_time, WorldRobot::Team::BLUE, i,
                                          kalman_robots_blue.at(i));
        }
    }
}

void World::detectKicks(RJ::Time calc_time) {
    KickEvent fast_event;
    KickEvent slow_event;

    bool is_fast_kick = fastKick.addRecord(calc_time, ball, robotsYellow,
                                           robotsBlue, fast_event);
    bool is_slow_kick = slowKick.addRecord(calc_time, ball, robotsYellow,
                                           robotsBlue, &slow_event);

    // If there isn't a kick recorded already
    if (!bestKickEstimate.getIsValid()) {
        // Try to use the slow kick as it's a better estimate
        // but take fast kick if there isn't a corrsponding slow kick yet
        if (is_slow_kick) {
            bestKickEstimate = slow_event;
        } else if (is_fast_kick) {
            bestKickEstimate = fast_event;
        }

        // There is a kick recorded already
    } else {
        const RJ::Seconds time_since_best_event(bestKickEstimate.getKickTime() -
                                                calc_time);
        const RJ::Seconds same_kick_timeout(PARAM_same_kick_timeout);
        const RJ::Seconds slow_kick_timeout(PARAM_slow_kick_timeout);
        const RJ::Seconds fast_kick_timeout(PARAM_fast_kick_timeout);

        // Try using the slow kick if:
        //      - It refers to the current best kick event (and probably is a
        //      better estimate)
        //      - The old kick timed out and should be updated
        if (is_slow_kick && (time_since_best_event < same_kick_timeout ||
                             time_since_best_event > slow_kick_timeout)) {
            bestKickEstimate = slow_event;

            // Try using the fast kick if the old kick timed out
        } else if (is_fast_kick && time_since_best_event > fast_kick_timeout) {
            bestKickEstimate = fast_event;

            // Remove the old kick if it's completely time out
        } else if (time_since_best_event >
                   slow_kick_timeout + fast_kick_timeout) {
            bestKickEstimate = KickEvent();
        }
    }
}

const WorldBall& World::getWorldBall() const { return ball; }

const std::vector<WorldRobot>& World::getRobotsYellow() const {
    return robotsYellow;
}

const std::vector<WorldRobot>& World::getRobotsBlue() const {
    return robotsBlue;
}

const KickEvent& World::getBestKickEstimate() const { return bestKickEstimate; }
}  // namespace vision_filter
