#include "BallBounce.hpp"

#include <algorithm>
#include <cmath>

#include <Constants.hpp>

#include "vision/util/VisionFilterConfig.hpp"

REGISTER_CONFIGURABLE(BallBounce)

ConfigDouble* BallBounce::robot_body_lin_dampen;
ConfigDouble* BallBounce::robot_mouth_lin_dampen;
ConfigDouble* BallBounce::robot_body_angle_dampen;
ConfigDouble* BallBounce::robot_mouth_angle_dampen;

/**
 * Note 0 case returns -1 instead of 0
 * Forced to check with small epsilon since we actually care about 0 being represented
 * correctly as -1. It's ok if the 1.0e-10 doesn't return -1 or 1.
 * 
 * This is mostly so the simple test case have the correct behavior
 * If it lands on the 1.0e-10 boundary the code reacts as if there was no hit
 */
// int sign(double val) { return (1.0e-10 < val) - (val <= 1.0e-10); }

void BallBounce::createConfiguration(Configuration* cfg) {
    robot_body_lin_dampen = new ConfigDouble(cfg, "VisionFilter/Bounce/robot_body_lin_dampen", .9);
    robot_mouth_lin_dampen = new ConfigDouble(cfg, "VisionFilter/Bounce/robot_mouth_lin_dampen", .3);
    robot_body_angle_dampen = new ConfigDouble(cfg, "VisionFilter/Bounce/robot_body_angle_dampen", 0);
    robot_mouth_angle_dampen = new ConfigDouble(cfg, "VisionFilter/Bounce/robot_mouth_angle_dampen", 0);
}

bool BallBounce::CalcBallBounce(const KalmanBall& ball,
                                const std::vector<WorldRobot>& yellowRobots,
                                const std::vector<WorldRobot>& blueRobots,
                                Geometry2d::Point& outNewVel) {

    // Figures out if there is an intersection and what the resulting velocity should be
    auto findEndVel = [&ball, &outNewVel] (const std::vector<WorldRobot>& robots) {
        for (const WorldRobot& robot : robots) {
            if (!robot.getIsValid()) {
                continue;
            }

            // Make sure ball is intersecting next frame
            if (!BallInRobot(ball, robot)) {
                continue;
            }

            std::vector<Geometry2d::Point> intersectPts = PossibleBallIntersectionPts(ball, robot);

            // Doesn't intersect
            if (intersectPts.size() == 0) {
                continue;
            }

            // Tangent to robot, assuming no interaction
            if (intersectPts.size() == 1) {
                continue;
            }

            // intersectPts.size() == 2

            //                        _____
            //                       /     \
            //                      | Robot |
            //                       \_____/
            //                          B
            //                         /|\
            //                        / | \
            //                       /  |  \
            //                      /   |   \
            //                     /    D    \
            //                    A           C
            // Ball moves from A->B
            // Bounces off the robot
            // Moves from B->C
            // Line B<->D is the line of reflection
            //
            //
            // Line B<->D goes from the center of the robot through the point of intersection between the robot and the ball
            // Since the robot is round (with a flat mouth), any time it hits the robot, we can assume that the it will reflect as if
            //   it hit a flat surface

            // We want to make sure that the ball is never inside the robot when doing this math, so we go back in time
            // This doesn't affect the results since we are just doing vectors
            Geometry2d::Point ballPosSafePt = ball.getPos() - ball.getVel().normalized();

            // Find the closest point
            // AKA: the first point the ball will hit off the robot
            // May actually be slightly off because we add the ball radius to the calculation circle
            // Does not account for the mouth just yet
            Geometry2d::Point closestIntersectPt = intersectPts.at(0);
            if ((ballPosSafePt - closestIntersectPt).magsq() > (ballPosSafePt - intersectPts.at(1)).magsq()) {
                closestIntersectPt = intersectPts.at(1);
            }

            // This is super easy, just check intersection with the mouth line
            // If the first point in the circle shell intersect is in the mouth angle range
            // Use the line intersect point instead
            // If there is no line intersect point (within that angle range)
            // then the ball is moving across the mouth of the robot
            // Note: No intersection across mouth is not accounted for
            Geometry2d::Line intersectLine = Geometry2d::Line(intersectPts.at(0), intersectPts.at(1));
            Geometry2d::Point mouthHalfUnitVec = Geometry2d::Point(0, 1).rotate(robot.getTheta());
            Geometry2d::Point mouthCenterPos = Geometry2d::Point(Robot_MouthRadius, 0).rotate(robot.getTheta()) + robot.getPos();
            Geometry2d::Line mouthLine = Geometry2d::Line(mouthCenterPos + mouthHalfUnitVec,
                                                        mouthCenterPos - mouthHalfUnitVec);

            Geometry2d::Point mouthIntersect;
            bool intersects = intersectLine.intersects(mouthLine, &mouthIntersect);

            // The mouth is a chord across the circle.
            // We have the distance of the chord to the center of the circle
            // We also have the radius of the robot
            const double chordHalfLength = pow(Robot_MouthWidth / 2.0, 2);
            bool didHitMouth = false;

            // If the line intersect is inside the mouth chord
            if (intersects && (mouthIntersect - mouthCenterPos).magsq() < chordHalfLength) {
                closestIntersectPt = mouthIntersect;
                didHitMouth = true;
            }

            //                          R
            //                        _____
            //                          B
            //                         /|\
            //                        / | \
            //                       /  |  \
            //                      /   |   \
            //                     /    |    \
            //                    A-----D-----C

            // B->A
            Geometry2d::Point intersectPtBallVector = ballPosSafePt - closestIntersectPt;
            // B->D (Officially R->B, but B->D makes more sense visually)
            Geometry2d::Point robotIntersectPtVector = closestIntersectPt - robot.getPos();
            Geometry2d::Point robotIntersectPtUnitVector = robotIntersectPtVector.normalized();

            // If it hit the mouth, the reflection line is pointing straight out
            if (didHitMouth) {
                robotIntersectPtVector = Geometry2d::Point(1, 0).rotate(robot.getTheta());
                robotIntersectPtUnitVector = robotIntersectPtVector;
            }

            // Project B->A vector onto B->D
            // This is so we can get D->A and D->C later
            double projectionMag = intersectPtBallVector.normalized().dot(robotIntersectPtUnitVector);
            Geometry2d::Point projection = projectionMag * robotIntersectPtUnitVector;

            // A->D, which is the same as D->C
            Geometry2d::Point projectionDiff = projection - intersectPtBallVector;

            Geometry2d::Point intersectPtReflectionVector = projection + projectionDiff;
            Geometry2d::Point intersectPtReflectionUnitVector = intersectPtReflectionVector.normalized();

            // Scale magnitude of velocity by a percentage
            double dampenLinCoeff   = *robot_body_lin_dampen;
            double dampenAngleCoeff = *robot_body_angle_dampen;

            if (didHitMouth) {
                dampenLinCoeff   = *robot_mouth_lin_dampen;
                dampenAngleCoeff = *robot_mouth_angle_dampen;
            }

            //                   C------D
            //                    \     |
            //                F    \    |
            //                  \   \   |
            //                    \  \  |
            //                      \ \ |
            //                        \\|              | y+
            //                          B              |
            //                                         |____ x+
            //
            // Note: Letters correspond to ones above
            //
            // We are trying to increase the angle CBD more when angle CBD is large
            // When angle CBD is 0, we want to keep the same angle

            // We dont want any extra rotation when angle CBD is 0 degrees or 90 degrees
            // Just to simplify implementation, I'm going to do a triangle
            //
            // df*45  -              /  \
            //                    /        \
            //                 /              \
            //  0     -     /                    \
            //
            //             |          |           |
            //            0 deg    45 deg       90 deg
            //
            // df is angle dampen factor
            // y axis represents max angle dampen in terms of degrees
            // x axis is the angle CBD

            // Angle CBD
            double halfReflectAngle = intersectPtReflectionUnitVector.angleBetween(robotIntersectPtUnitVector);
            double direction = robotIntersectPtUnitVector.cross(intersectPtReflectionUnitVector);
            double extraRotationAngle = -std::signbit(direction)*halfReflectAngle;
            extraRotationAngle = std::min(extraRotationAngle, M_PI_2 - extraRotationAngle)*dampenAngleCoeff;

            intersectPtReflectionUnitVector = intersectPtReflectionUnitVector.rotate(extraRotationAngle);

            outNewVel = intersectPtReflectionUnitVector * ball.getVel().mag();

            return true;
        }

        return false;
    };

    bool bounceFound = false;
    bounceFound |= findEndVel(yellowRobots);
    bounceFound |= findEndVel(blueRobots);

    return bounceFound;
}


bool BallBounce::BallInRobot(const KalmanBall& ball, const WorldRobot& robot) {
    Geometry2d::Point nextPos = ball.getPos() + ball.getVel() * *VisionFilterConfig::vision_loop_dt;

    return (robot.getPos() - nextPos).mag() < Robot_Radius + Ball_Radius;
}

std::vector<Geometry2d::Point> BallBounce::PossibleBallIntersectionPts(
        const KalmanBall& ball, const WorldRobot& robot) {
    // http://mathworld.wolfram.com/Circle-LineIntersection.html

    std::vector<Geometry2d::Point> out;

    // ballPos is the ball pos in robot centered coordinates
    // ballVel is the ball vel vector in robot centered coordinates
    // We really just want a line in the direction of ball vel motion
    Geometry2d::Point ballPos = ball.getPos() - robot.getPos();
    Geometry2d::Point ballVel = ball.getPos() + ball.getVel() - robot.getPos();

    // Magnitude of the line
    Geometry2d::Point d = ballVel - ballPos;
    double dr = d.mag();
    // Determinant
    double D = ballPos.x()*ballVel.y() - ballPos.y()*ballVel.x();
    // Assume that two spheres intersection, is similar to the addition of their radius and a point
    double r = Robot_Radius + Ball_Radius;

    // If the ball really isn't moving, just assume no intersection
    // since the math will go to inf
    if (abs(dr) < 1e-5) {
        return out;
    }

    double x1 = D*d.y() + std::signbit(d.y()) * d.x() * sqrt(r*r*dr*dr - D*D);
    x1 /= dr*dr;
    x1 += robot.getPos().x();

    double y1 = -D*d.x() + abs(d.y()) * d.x() * sqrt(r*r*dr*dr - D*D);
    y1 /= dr*dr;
    y1 += robot.getPos().y();

    double x2 = D*d.y() - std::signbit(d.y()) * d.x() * sqrt(r*r*dr*dr - D*D);
    x2 /= dr*dr;
    x2 += robot.getPos().x();

    double y2 = -D*d.x() - abs(d.y()) * sqrt(r*r*dr*dr - D*D);
    y2 /= dr*dr;
    y2 += robot.getPos().y();

    Geometry2d::Point pt1 = Geometry2d::Point(x1, y1);
    Geometry2d::Point pt2 = Geometry2d::Point(x2, y2);

    double delta = r*r*dr*dr - D*D;

    // Add the actual intersection points to the list

    // Tagent
    if (delta == 0) {
        out.push_back(pt1);
    }
    // Two intersection points
    else {
        out.push_back(pt1);
        out.push_back(pt2);
    }
    
    return out;
}