#include <rj_vision_filter/robot/WorldRobot.hpp>
#include <iostream>
#include <cmath>

REGISTER_CONFIGURABLE(WorldRobot)

ConfigDouble* WorldRobot::robot_merger_power;

void WorldRobot::createConfiguration(Configuration* cfg) {
    robot_merger_power = new ConfigDouble(cfg, "VisionFilter/WorldRobot/robot_merger_power", 1.5);
}

WorldRobot::WorldRobot() : isValid(false) {}

WorldRobot::WorldRobot(RJ::Time calcTime, Team team, int robotID,
                       const std::list<KalmanRobot>& kalmanRobots)
    : team(team), robotID(robotID), isValid(true), time(calcTime) {
    // Theta's are converted to rect coords then back to polar to convert
    Geometry2d::Point posCartesianAvg;
    Geometry2d::Point thetaCartesianAvg;
    Geometry2d::Twist twistAvg;

    double totalPosWeight = 0;
    double totalVelWeight = 0;

    // Below 1 would invert the ratio of scaling
    // Above 2 would just be super noisy
    if (*robot_merger_power < 1 || *robot_merger_power > 2) {
        std::cout
             << "WARN: robot_merger_power must be between 1 and 2"
             << std::endl;
    }

    if (kalmanRobots.empty()) {
        std::cout
             << "ERROR: Zero robots are given to the WorldRobot constructor"
             << std::endl;

        pose.position() = posCartesianAvg;
        pose.heading() = 0;
        twist.linear() = twistAvg.linear();
        twist.angular() = twistAvg.angular();
        posCov = 0;
        velCov = 0;
        isValid = false;

        return;
    }

    for (const KalmanRobot& robot : kalmanRobots) {
        // Get the covariance of everything
        // AKA how well we can predict the next measurement
        Geometry2d::Pose poseCov{robot.getPosCov(), robot.getThetaCov()};
        Geometry2d::Twist twistCov{robot.getVelCov(), robot.getOmegaCov()};

        // Std dev of each state
        // Lower std dev gives better idea of true values
        Geometry2d::Pose poseStdDev;
        Geometry2d::Twist twistStdDev;
        poseStdDev.position().x() = std::sqrt(poseCov.position().x());
        poseStdDev.position().y() = std::sqrt(poseCov.position().y());
        twistStdDev.linear().x() = std::sqrt(twistCov.linear().x());
        twistStdDev.linear().y() = std::sqrt(twistCov.linear().y());
        poseStdDev.heading() = std::sqrt(poseCov.heading());
        twistStdDev.angular() = std::sqrt(poseCov.heading());

        // Inversely proportional to how much the filter has been updated
        double filterUncertantity = 1.0 / robot.getHealth();

        // How good of pos/vel estimation in total
        // (This is less efficient than just doing the sqrt(x_cov + y_cov),
        //  but it's a little more clear math-wise)
        double posUncertantity = std::sqrt(poseStdDev.position().magsq() +
                                           std::pow(poseStdDev.heading(), 2));
        double velUncertantity = std::sqrt(poseStdDev.position().magsq() +
                                           std::pow(twistStdDev.angular(), 2));

        double filterPosWeight = std::pow(posUncertantity * filterUncertantity,
                                          -*robot_merger_power);

        double filterVelWeight = std::pow(velUncertantity * filterUncertantity,
                                          -*robot_merger_power);

        posCartesianAvg += filterPosWeight * robot.getPos();
        thetaCartesianAvg +=
            Geometry2d::Point(filterPosWeight * cos(robot.getTheta()),
                              filterPosWeight * sin(robot.getTheta()));
        twistAvg.linear() += filterVelWeight * robot.getVel();
        twistAvg.angular() += filterVelWeight * robot.getOmega();

        totalPosWeight += filterPosWeight;
        totalVelWeight += filterVelWeight;
    }

    posCartesianAvg /= totalPosWeight;
    thetaCartesianAvg /= totalPosWeight;
    twistAvg.linear() /= totalVelWeight;
    twistAvg.angular() /= totalVelWeight;

    pose.position() = posCartesianAvg;
    pose.heading() = atan2(thetaCartesianAvg.y(), thetaCartesianAvg.x());
    twist.linear() = twistAvg.linear();
    twist.angular() = twistAvg.angular();
    posCov = totalPosWeight / kalmanRobots.size();
    velCov = totalVelWeight / kalmanRobots.size();
    robotComponents = kalmanRobots;
}

bool WorldRobot::getIsValid() const {
    return isValid;
}

int WorldRobot::getRobotID() const {
    return robotID;
}

Geometry2d::Point WorldRobot::getPos() const { return pose.position(); }

double WorldRobot::getTheta() const { return pose.heading(); }

Geometry2d::Pose WorldRobot::getPose() const { return pose; }

Geometry2d::Point WorldRobot::getVel() const { return twist.linear(); }

double WorldRobot::getOmega() const { return twist.angular(); }

Geometry2d::Twist WorldRobot::getTwist() const { return twist; }

double WorldRobot::getPosCov() const {
    return posCov;
}

double WorldRobot::getVelCov() const {
    return velCov;
}

const std::list<KalmanRobot>& WorldRobot::getRobotComponents() const {
    return robotComponents;
}

RJ::Time WorldRobot::getTime() const {
    return time;
}