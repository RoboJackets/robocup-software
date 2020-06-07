#pragma once

#include <geometry2d/point.h>
#include <geometry2d/pose.h>

#include <Configuration.hpp>
#include <list>

#include "KalmanRobot.hpp"

class KalmanRobot;

class WorldRobot {
public:
    enum Team {
        YELLOW,
        BLUE
    };

    /**
     * Creates an invalid world robot.
     * This is so the World can create a full list of robots without dealing with holes
     * It's a little less efficient, but it makes things much cleaner code-wise
     */
    WorldRobot();

    /**
     * Creates a valid world robot
     *
     * @param robotID The ID of the robot
     * @param team The team color
     * @param kalmanRobots List of kalman robots from each of the cameras to merger
     */
    WorldRobot(RJ::Time calcTime, Team team, int robotID,
               const std::list<KalmanRobot>& kalmanRobots);

    /**
     * @return If the robot actually represents a real robot
     */
    bool getIsValid() const;

    /**
     * @return Enum value representing team color
     */
    Team getTeamColor() const;

    /**
     * @return The robot id
     */
    int getRobotID() const;

    /**
     * @return The best estimated position of the robot
     */
    geometry2d::Point getPos() const;

    /**
     * @return The best estimated heading of the robot
     */
    double getTheta() const;

    /**
     * @return The best estimated pose of the robot
     */
    geometry2d::Pose getPose() const;

    /**
     * @return The best estimated velocity of the robot
     */
    geometry2d::Point getVel() const;

    /**
     * @return The best estimated angular velocity of the robot
     */
    double getOmega() const;

    /**
     * @return The best estimated twist of the robot
     */
    geometry2d::Twist getTwist() const;

    /**
     * @return The average position covariance of the filter including theta
     */
    double getPosCov() const;

    /**
     * @return The average velocity covariance of the filter including omega
     */
    double getVelCov() const;

    /**
     * @return List of all the building kalman robots for this world robot
     */
    const std::list<KalmanRobot>& getRobotComponents() const;

    /**
     * @return Time of creation for the robot estimate
     */
    RJ::Time getTime() const;

    static void createConfiguration(Configuration* cfg);

private:
    Team team;
    int robotID{};
    geometry2d::Pose pose;
    geometry2d::Twist twist;
    double posCov{};
    double velCov{};
    std::list<KalmanRobot> robotComponents;
    RJ::Time time;

    bool isValid;
    static ConfigDouble* robot_merger_power;
};
