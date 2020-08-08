#pragma once

#include <configuration.hpp>
#include <list>
#include <rj_vision_filter/ball/ball_bounce.hpp>
#include <rj_vision_filter/ball/camera_ball.hpp>
#include <rj_vision_filter/ball/kalman_ball.hpp>
#include <rj_vision_filter/ball/world_ball.hpp>
#include <rj_vision_filter/camera/camera_frame.hpp>
#include <rj_vision_filter/robot/camera_robot.hpp>
#include <rj_vision_filter/robot/kalman_robot.hpp>
#include <rj_vision_filter/robot/world_robot.hpp>
#include <vector>

namespace vision_filter {
/**
 * Contains all the kalman balls/robots for the specific camera
 */
class Camera {
public:
    /**
     * Creates an invalid camera
     */
    Camera();

    /**
     * Creates a valid camera with a specific id
     *
     * @param camera_id ID of this camera
     */
    Camera(int camera_id);

    /**
     * Returns whether this camera is valid and initialized correctly
     */
    bool get_is_valid() const;

    /**
     * Tries to predict bounces off the best known estimation of the robots
     *
     * @param yellow_robots List of the yellow world robots in the world class
     * @param blue_robots List of blue world robots in the world class
     */
    void process_ball_bounce(const std::vector<WorldRobot>& yellow_robots,
                           const std::vector<WorldRobot>& blue_robots);

    /**
     * Updates all the filters with the latest camera frame data for this camera
     *
     * @param calc_time Time of this calculation
     * @param ball_list Unsorted list of balls measurements
     * @param yellow_robot_list List of yellow robots sorted by id
     * @param blue_robot_list List of blue robots sorted by id
     * @param previous_world_ball Best idea of current ball pos/vel to init
     * velocity of new filters
     * @param previous_yellow_world_robots Best idea of current robots pos/vel to
     * init velocity of new filters
     * @param previous_blue_world_robots Best idea of current robots pos/vel to
     * init velocity of new filters
     *
     * Note: Call either this OR update_without_frame once an iteration
     */
    void update_with_frame(
        RJ::Time calc_time, const std::vector<CameraBall>& ball_list,
        const std::vector<std::list<CameraRobot>>& yellow_robot_list,
        const std::vector<std::list<CameraRobot>>& blue_robot_list,
        const WorldBall& previous_world_ball,
        const std::vector<WorldRobot>& previous_yellow_world_robots,
        const std::vector<WorldRobot>& previous_blue_world_robots);

    /**
     * Updates all the filters without any new data from this specific camera
     *
     * @param calc_time Time of this calculation
     *
     * Note: Call either this OR update_with_frame once an iteration
     */
    void update_without_frame(RJ::Time calc_time);

    /**
     * @return A list of the kalman balls associated with the camera
     */
    const std::list<KalmanBall>& get_kalman_balls() const;

    /**
     * @return A vector of yellow kalman robot lists
     */
    const std::vector<std::list<KalmanRobot>>& get_kalman_robots_yellow() const;

    /**
     * @return A vector of blue kalman robot lists
     */
    const std::vector<std::list<KalmanRobot>>& get_kalman_robots_blue() const;

private:
    /**
     * MHKF refers to the Multi-Hypothesis Kalman Filter
     *   MHKF averages "near" the current predicted position before using that
     * as the measurement Any measurements not "near" a filter are used as the
     * initial values to create a new filter
     *
     * AKF refers to the Average Kalman Filter
     *   AKF averages all measurements and then uses that as the measurement to
     * the filter
     */

    /**
     * Figures out which update style to use and calls that
     *
     * @param calc_time Time of this calculation
     * @param ball_list Unsorted list of balls measurements
     * @param previous_world_ball Best idea of current ball pos/vel to init
     * velocity of new filters
     */
    void update_balls(RJ::Time calc_time, const std::vector<CameraBall>& ball_list,
                     const WorldBall& previous_world_ball);

    /**
     * Updates ball filters using MHKF style updater
     *
     * @param calc_time Time of this calculation
     * @param ball_list Unsorted list of balls measurements
     * @param previous_world_ball Best idea of current ball pos/vel to init
     * velocity of new filters
     */
    void update_balls_mhkf(RJ::Time calc_time,
                         const std::vector<CameraBall>& ball_list,
                         const WorldBall& previous_world_ball);

    /**
     * Updates ball filters using AKF style updater
     *
     * @param calc_time Time of this calculation
     * @param ball_list Unsorted list of balls measurements
     * @param previous_world_ball Best idea of current ball pos/vel to init
     * velocity of new filters
     */
    void update_balls_akf(RJ::Time calc_time,
                        const std::vector<CameraBall>& ball_list,
                        const WorldBall& previous_world_ball);

    /**
     * Figures out which update style to use and calls that
     *
     * @param calc_time Time of this calculation
     * @param yellow_robot_list List of yellow robots sorted by id
     * @param blue_robot_list List of blue robots sorted by id
     * @param previous_yellow_world_robots Best idea of current robots pos/vel to
     * init velocity of new filters
     * @param previous_blue_world_robots Best idea of current robots pos/vel to
     * init velocity of new filters
     */
    void update_robots(
        RJ::Time calc_time,
        const std::vector<std::list<CameraRobot>>& yellow_robot_list,
        const std::vector<std::list<CameraRobot>>& blue_robot_list,
        const std::vector<WorldRobot>& previous_yellow_world_robots,
        const std::vector<WorldRobot>& previous_blue_world_robots);

    /**
     * Updates robot filters using MHKF style updater
     *
     * @param calc_time Time of this calculation
     * @param single_robot_list List of one robot ID measurements
     * @param previous_world_robot Best idea of current robot pos/vel to init
     * velocity of new filters
     * @param single_kalmanRobotList List of one robot ID's kalman filters
     */
    void update_robots_mhkf(RJ::Time calc_time,
                          const std::list<CameraRobot>& single_robot_list,
                          const WorldRobot& previous_world_robot,
                          std::list<KalmanRobot>& single_kalman_robot_list);

    /**
     * Updates robot filters using AKF style updater
     *
     * @param calc_time Time of this calculation
     * @param single_robot_list List of one robot ID measurements
     * @param previous_world_robot Best idea of current robot pos/vel to init
     * velocity of new filters
     * @param single_kalmanRobotList List of one robot ID's kalman filters
     */
    void update_robots_akf(RJ::Time calc_time,
                         const std::list<CameraRobot>& single_robot_list,
                         const WorldRobot& previous_world_robot,
                         std::list<KalmanRobot>& single_kalman_robot_list);

    /**
     * Removes any invalid kalman balls that may be too old etc
     *
     * Done every iteration to keep things clean
     */
    void remove_invalid_balls();
    void remove_invalid_robots();

    /**
     * Predicts all robots in the given list
     * Simplifies some copy paste
     *
     * @param calc_time Time of this calculation
     * @param robot_list_list Either kalmanRobotYellowList or kalmanRobotBlueList
     */
    static void predict_all_robots(
        RJ::Time calc_time, std::vector<std::list<KalmanRobot>>& robot_list_list);

    bool is_valid_;

    int camera_id_{};
    std::list<KalmanBall> kalman_ball_list_;
    std::vector<std::list<KalmanRobot>> kalman_robot_yellow_list_;
    std::vector<std::list<KalmanRobot>> kalman_robot_blue_list_;
};
}  // namespace vision_filter
