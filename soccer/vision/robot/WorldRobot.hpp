#pragma once

#include <list>

#include <Geometry2d/Point.hpp>
#include <Geometry2d/Pose.hpp>
#include <Configuration.hpp>

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
    WorldRobot(RJ::Time calcTime, Team team, int robotID, std::list<KalmanRobot> kalmanRobots);

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
    Geometry2d::Point getPos() const;

    /**
     * @return The best estimated heading of the robot
     */
    double getTheta() const;

    /**
     * @return The best estimated velocity of the robot
     */
    Geometry2d::Point getVel() const;

    /**
     * @return The best estimated angular velocity of the robot
     */
    double getOmega() const;

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
    int robotID;
    Geometry2d::Point pos;
    double theta;
    Geometry2d::Point vel;
    double omega;
    double posCov;
    double velCov;
    std::list<KalmanRobot> robotComponents;
    RJ::Time time;

    bool isValid;
    static ConfigDouble* robot_merger_power;
};