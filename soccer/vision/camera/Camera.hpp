#include <list>
#pragma once

#include <vector>

#include <Util.hpp>
#include <Configuration.hpp>

#include "ball/CameraBall.hpp"
#include "ball/KalmanBall.hpp"
#include "ball/WorldBall.hpp"
#include "robot/CameraRobot.hpp"
#include "robot/KalmanRobot.hpp"
#include "robot/WorldBall.hpp"
#include "CameraFrame.hpp"
#include "ball/BallBounce.hpp"

class Camera {
public:
    Camera(int cameraID);

    /**
     * Tries to predict bounces off the best known estimation of the robots
     *
     * @param yellowRobots List of the yellow world robots in the world class
     * @param blueRobots List of blue world robots in the world class
     */
    void processBallBounce(std::vector<WorldRobot> yellowRobots,
                           std::vecotr<WorldRobot> blueRobots);

    /**
     * Updates all the filters with the latest camera frame data for this camera
     *
     * @param calcTime Time of this calculation
     * @param ballList Unsorted list of balls measurements
     * @param yellowRobotList Unsorted list of yellow robots
     * @param blueRobotList Unsorted list of blue robots
     * @param previousWorldBall Best idea of current ball pos/vel to init velocity of new filters
     * @param previousYellowWorldRobots Best idea of current robots pos/vel to init velocity of new filters
     * @param previousBlueWorldRobots Best idea of current robots pos/vel to init velocity of new filters
     *
     * Note: Call either this OR updateWithoutFrame once an iteration
     */
    void updateWithFrame(RJ::Time calcTime,
                         std::vector<CameraBall> ballList,
                         std::vector<CameraRobot> yellowRobotList,
                         std::vector<CameraRobot> blueRobotList,
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

    static void createConfiguration(Configuration* cfg);

private:
    /**
     * MHKF refers to the Multi-Hypothesis Kalman Filter
     *   MHKF averages "near" the current predicted position before using that as the measurement
     *   Any measurements not "near" a filter are used as the initial values to create a new filter
     *
     * AKF refers to the Average Kalman Filter
     *   AKF averages all mesurements and then uses that as the measurement to the filter
     */

    /**
     * Figures out which update style to use and calls that
     */
    void updateBalls(RJ::Time calcTime, std::vector<CameraBall> ballList, WorldBall& previousWorldBall);

    /**
     * Updates ball filters using MHKF style updater
     */
    void updateBallsMHKF(RJ::Time calcTime, std::vector<CameraBall> ballList, WorldBall& previousWorldBall);

    /**
     * Updates ball filters using AKF style updater
     */
    void updateBallsAKF(RJ::Time calcTime, std::vector<CameraBall> ballList, WorldBall& previousWorldBall):

    /**
     * Figures out which update style to use and calls that
     */
    void updateRobots(RJ::Time calcTime, std::vector<CameraRobot> yellowRobotList,
                                         std::vector<CameraRobot> blueRobotList);

    /**
     * Updates robot filters using MHKF style updater
     */
    void updateRobotsMHKF(RJ::Time calcTime, std::vector<CameraRobot> yellowRobotList,
                                             std::vector<CameraRobot> blueRobotList);

    /**
     * Updates robot filters using AKF style updater
     */
    void updateRobotsAKF(RJ::Time calcTime, std::vector<CameraRobot> yellowRobotList,
                                            std:vector<CameraRobot> blueRobotList);

    /**
     * Removes any invalid kalman balls that may be too old etc
     *
     * Done every iteration to keep things clean
     */
    void removeInvalidBalls();
    void removeInvalidRobots();

    int cameraID;
    std::list<KalmanBall> kalmanBallList;
    std::vector<std::list<KalmanRobot>> kalmanRobotYellowList;
    std::vector<std::list<KalmanRobot>> kalmanRobotBlueList;

    ConfigDouble* MHKF_radius_cutoff;
    ConfigBool* use_MHKF;

    // TODO: Use the global max robot id number
    const int maxRobotJerseyNum = 12;
};