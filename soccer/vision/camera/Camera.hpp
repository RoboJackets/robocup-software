#include <list>
#pragma once

#include <vector>

#include <Utils.hpp>
#include <Configuration.hpp>

#include "vision/ball/CameraBall.hpp"
#include "vision/ball/KalmanBall.hpp"
#include "vision/ball/WorldBall.hpp"
#include "vision/robot/CameraRobot.hpp"
#include "vision/robot/KalmanRobot.hpp"
#include "vision/robot/WorldRobot.hpp"
#include "CameraFrame.hpp"
#include "vision/ball/BallBounce.hpp"

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
    bool getIsValid();

    /**
     * Tries to predict bounces off the best known estimation of the robots
     *
     * @param yellowRobots List of the yellow world robots in the world class
     * @param blueRobots List of blue world robots in the world class
     */
    void processBallBounce(std::vector<WorldRobot> yellowRobots,
                           std::vector<WorldRobot> blueRobots);

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
                         std::vector<CameraBall>& ballList,
                         std::vector<std::list<CameraRobot>>& yellowRobotList,
                         std::vector<std::list<CameraRobot>>& blueRobotList,
                         WorldBall& previousWorldBall,
                         std::vector<WorldRobot>& previousYellowWorldRobots,
                         std::vector<WorldRobot>& previousBlueWorldRobots);

    /**
     * Updates all the filters without any new data from this specific camera
     *
     * @param calcTime Time of this calculation
     *
     * Note: Call either this OR updateWithFrame once an iteration
     */
    void updateWithoutFrame(RJ::Time calcTime);

    std::list<KalmanBall> getKalmanBalls();
    std::vector<std::list<KalmanRobot>> getKalmanRobotsYellow();
    std::vector<std::list<KalmanRobot>> getKalmanRobotsBlue();

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
     */
    void updateBalls(RJ::Time calcTime,
                     std::vector<CameraBall> ballList,
                     WorldBall& previousWorldBall);

    /**
     * Updates ball filters using MHKF style updater
     */
    void updateBallsMHKF(RJ::Time calcTime,
                         std::vector<CameraBall> ballList,
                         WorldBall& previousWorldBall);

    /**
     * Updates ball filters using AKF style updater
     */
    void updateBallsAKF(RJ::Time calcTime,
                        std::vector<CameraBall> ballList,
                        WorldBall& previousWorldBall);

    /**
     * Figures out which update style to use and calls that
     */
    void updateRobots(RJ::Time calcTime,
                      std::vector<std::list<CameraRobot>>& yellowRobotList,
                      std::vector<std::list<CameraRobot>>& blueRobotList,
                      std::vector<WorldRobot>& previousYellowWorldRobots,
                      std::vector<WorldRobot>& previousBlueWorldRobots);

    /**
     * Updates robot filters using MHKF style updater
     */
    void updateRobotsMHKF(RJ::Time calcTime,
                          std::list<CameraRobot>& singleRobotList,
                          WorldRobot& previousWorldRobot,
                          std::list<KalmanRobot>& singleKalmanRobotList);

    /**
     * Updates robot filters using AKF style updater
     */
    void updateRobotsAKF(RJ::Time calcTime,
                         std::list<CameraRobot>& singleRobotList,
                         WorldRobot& previousWorldRobot,
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
     * @param robotListList Either kalmanRobotYellowList or kalmnaRobotBlueList
     */
    void predictAllRobots(RJ::Time calcTime, std::vector<std::list<KalmanRobot>>& robotListList);

    bool isValid;

    int cameraID;
    std::list<KalmanBall> kalmanBallList;
    std::vector<std::list<KalmanRobot>> kalmanRobotYellowList;
    std::vector<std::list<KalmanRobot>> kalmanRobotBlueList;

    static ConfigDouble* MHKF_radius_cutoff;
    static ConfigBool* use_MHKF;
    static ConfigInt* max_num_kalman_balls;
    static ConfigInt* max_num_kalman_robots;
};