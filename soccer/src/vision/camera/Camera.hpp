#include <list>
#pragma once

#include <Configuration.hpp>
#include <rj_common/Utils.hpp>
#include <vector>

#include "CameraFrame.hpp"
#include "vision/ball/BallBounce.hpp"
#include "vision/ball/CameraBall.hpp"
#include "vision/ball/KalmanBall.hpp"
#include "vision/ball/WorldBall.hpp"
#include "vision/robot/CameraRobot.hpp"
#include "vision/robot/KalmanRobot.hpp"
#include "vision/robot/WorldRobot.hpp"

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
     * @param cameraID ID of this camera
     */
    Camera(int cameraID);

    /**
     * Returns whether this camera is valid and initialized correctly
     */
    bool getIsValid() const;

    /**
     * Tries to predict bounces off the best known estimation of the robots
     *
     * @param yellowRobots List of the yellow world robots in the world class
     * @param blueRobots List of blue world robots in the world class
     */
    void processBallBounce(const std::vector<WorldRobot>& yellowRobots,
                           const std::vector<WorldRobot>& blueRobots);

    /**
     * Updates all the filters with the latest camera frame data for this camera
     *
     * @param calcTime Time of this calculation
     * @param ballList Unsorted list of balls measurements
     * @param yellowRobotList List of yellow robots sorted by id
     * @param blueRobotList List of blue robots sorted by id
     * @param previousWorldBall Best idea of current ball pos/vel to init velocity of new filters
     * @param previousYellowWorldRobots Best idea of current robots pos/vel to init velocity of new filters
     * @param previousBlueWorldRobots Best idea of current robots pos/vel to init velocity of new filters
     *
     * Note: Call either this OR updateWithoutFrame once an iteration
     */
    void updateWithFrame(RJ::Time calcTime,
                         const std::vector<CameraBall>& ballList,
                         const std::vector<std::list<CameraRobot>>& yellowRobotList,
                         const std::vector<std::list<CameraRobot>>& blueRobotList,
                         const WorldBall& previousWorldBall,
                         const std::vector<WorldRobot>& previousYellowWorldRobots,
                         const std::vector<WorldRobot>& previousBlueWorldRobots);

    /**
     * Updates all the filters without any new data from this specific camera
     *
     * @param calcTime Time of this calculation
     *
     * Note: Call either this OR updateWithFrame once an iteration
     */
    void updateWithoutFrame(RJ::Time calcTime);

    /**
     * @return A list of the kalman balls associated with the camera
     */
    const std::list<KalmanBall>& getKalmanBalls() const;

    /**
     * @return A vector of yellow kalman robot lists
     */
    const std::vector<std::list<KalmanRobot>>& getKalmanRobotsYellow() const;

    /**
     * @return A vector of blue kalman robot lists
     */
    const std::vector<std::list<KalmanRobot>>& getKalmanRobotsBlue() const;

    static void createConfiguration(Configuration* cfg);

private:
    /**
     * MHKF refers to the Multi-Hypothesis Kalman Filter
     *   MHKF averages "near" the current predicted position before using that as the measurement
     *   Any measurements not "near" a filter are used as the initial values to create a new filter
     *
     * AKF refers to the Average Kalman Filter
     *   AKF averages all measurements and then uses that as the measurement to the filter
     */

    /**
     * Figures out which update style to use and calls that
     *
     * @param calcTime Time of this calculation
     * @param ballList Unsorted list of balls measurements
     * @param previousWorldBall Best idea of current ball pos/vel to init velocity of new filters
     */
    void updateBalls(RJ::Time calcTime, const std::vector<CameraBall>& ballList,
                     const WorldBall& previousWorldBall);

    /**
     * Updates ball filters using MHKF style updater
     *
     * @param calcTime Time of this calculation
     * @param ballList Unsorted list of balls measurements
     * @param previousWorldBall Best idea of current ball pos/vel to init velocity of new filters
     */
    void updateBallsMHKF(RJ::Time calcTime,
                         const std::vector<CameraBall>& ballList,
                         const WorldBall& previousWorldBall);

    /**
     * Updates ball filters using AKF style updater
     *
     * @param calcTime Time of this calculation
     * @param ballList Unsorted list of balls measurements
     * @param previousWorldBall Best idea of current ball pos/vel to init velocity of new filters
     */
    void updateBallsAKF(RJ::Time calcTime,
                        const std::vector<CameraBall>& ballList,
                        const WorldBall& previousWorldBall);

    /**
     * Figures out which update style to use and calls that
     *
     * @param calcTime Time of this calculation
     * @param yellowRobotList List of yellow robots sorted by id
     * @param blueRobotList List of blue robots sorted by id
     * @param previousYellowWorldRobots Best idea of current robots pos/vel to init velocity of new filters
     * @param previousBlueWorldRobots Best idea of current robots pos/vel to init velocity of new filters
     */
    void updateRobots(RJ::Time calcTime,
                      const std::vector<std::list<CameraRobot>>& yellowRobotList,
                      const std::vector<std::list<CameraRobot>>& blueRobotList,
                      const std::vector<WorldRobot>& previousYellowWorldRobots,
                      const std::vector<WorldRobot>& previousBlueWorldRobots);

    /**
     * Updates robot filters using MHKF style updater
     *
     * @param calcTime Time of this calculation
     * @param singleRobotList List of one robot ID measurements
     * @param previousWorldRobot Best idea of current robot pos/vel to init velocity of new filters
     * @param singleKalmanRobotList List of one robot ID's kalman filters
     */
    void updateRobotsMHKF(RJ::Time calcTime,
                          const std::list<CameraRobot>& singleRobotList,
                          const WorldRobot& previousWorldRobot,
                          std::list<KalmanRobot>& singleKalmanRobotList);

    /**
     * Updates robot filters using AKF style updater
     *
     * @param calcTime Time of this calculation
     * @param singleRobotList List of one robot ID measurements
     * @param previousWorldRobot Best idea of current robot pos/vel to init velocity of new filters
     * @param singleKalmanRobotList List of one robot ID's kalman filters
     */
    void updateRobotsAKF(RJ::Time calcTime,
                         const std::list<CameraRobot>& singleRobotList,
                         const WorldRobot& previousWorldRobot,
                         std::list<KalmanRobot>& singleKalmanRobotList);

    /**
     * Removes any invalid kalman balls that may be too old etc
     *
     * Done every iteration to keep things clean
     */
    void removeInvalidBalls();
    void removeInvalidRobots();

    /**
     * Predicts all robots in the given list
     * Simplifies some copy paste
     *
     * @param calcTime Time of this calculation
     * @param robotListList Either kalmanRobotYellowList or kalmanRobotBlueList
     */
    static void predictAllRobots(
        RJ::Time calcTime, std::vector<std::list<KalmanRobot>>& robotListList);

    bool isValid;

    int cameraID{};
    std::list<KalmanBall> kalmanBallList;
    std::vector<std::list<KalmanRobot>> kalmanRobotYellowList;
    std::vector<std::list<KalmanRobot>> kalmanRobotBlueList;

    // The cutoff radius for when to associate measurements to kalman objects
    static ConfigDouble* MHKF_radius_cutoff;
    // Whether to use MHKF or AKF
    static ConfigBool* use_MHKF;
    // Max number of kalman balls for this specific camera
    static ConfigInt* max_num_kalman_balls;
    // Max number of kalman robots for each robot id for this specific camera
    static ConfigInt* max_num_kalman_robots;
};
