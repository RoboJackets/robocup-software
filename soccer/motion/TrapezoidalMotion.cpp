#include "TrapezoidalMotion.hpp"
#include <cmath>
#include <iostream>
#include "Utils.hpp"

using namespace std;

double Trapezoidal::getTime(double distance, double pathLength, double maxSpeed,
                            double maxAcc, double startSpeed,
                            double finalSpeed) {
    startSpeed = fmin(startSpeed, maxSpeed);
    finalSpeed = fmin(finalSpeed, maxSpeed);
    double rampUpTime = (maxSpeed - startSpeed) / maxAcc;
    double plateauTime;
    double rampDownTime = (finalSpeed - maxSpeed) / -maxAcc;

    double rampUpDist = rampUpTime * (startSpeed + maxSpeed) / 2.0;
    double plateauDist;
    double rampDownDist = rampDownTime * (maxSpeed + finalSpeed) / 2.0;

    if (rampUpDist + rampDownDist > pathLength) {
        // triangle case: we don't ever hit full speed

        // Calculate what max speed we actually reach (it's less than the
        // parameter passed in).
        // We write an equation for pathLength given maxSpeed, then solve for
        // maxSpeed
        // 	rampUpTime = (maxSpeed - startSpeed) / maxAcc;
        // 	rampDownTime = (finalSpeed - maxSpeed) / -maxAcc;
        // 	pathLength = (startSpeed + maxSpeed)/2*rampUpTime
        // 					+ (maxSpeed + finalSpeed)/2*rampDownTime;
        // We then solve for maxSpeed:
        // maxSpeed = sqrt(pathLength*maxAcc + startSpeed*startSpeed +
        // finalSpeed*finalSpeed);
        maxSpeed = sqrt((2 * maxAcc * pathLength + powf(startSpeed, 2) +
                         powf(finalSpeed, 2)) /
                        2.0);

        rampUpTime = (maxSpeed - startSpeed) / maxAcc;
        rampDownTime = (finalSpeed - maxSpeed) / -maxAcc;
        rampUpDist = (startSpeed + maxSpeed) / 2.0 * rampUpTime;
        rampDownDist = (finalSpeed + maxSpeed) / 2.0 * rampDownTime;

        // no plateau
        plateauTime = 0;
        plateauDist = 0;

    } else {
        // trapezoid case: there's a time where we go at maxSpeed for a bit
        plateauDist = pathLength - (rampUpDist + rampDownDist);
        plateauTime = plateauDist / maxSpeed;
    }

    if (distance <= 0) {
        return 0;
    }

    if (abs(distance - (rampUpDist + plateauDist + rampDownDist)) < 0.00001) {
        return rampUpTime + plateauTime + rampDownTime;
    }
    if (distance < rampUpDist) {
        // time calculations
        /*
            1/2*a*t^2 + t*v0 - d = 0
            t = -b +- sqrt(b^2 - 4*a*c)/(2*a)

        */
        double b = startSpeed;
        double a = maxAcc / 2.0;
        double c = -distance;
        double root = sqrt(b * b - 4 * a * c);
        double temp1 = (-b + root) / (2 * a);
        double temp2 = (-b - root) / (2 * a);
        if (std::isnan(root)) {
            debugThrow(
                "TrapezoidalMotion failed. Solution is imaginary");  // TODO
                                                                     // Handle
                                                                     // this
            return rampUpTime;
        }
        if (temp1 > 0 && temp1 < rampUpTime) {
            return temp1;
        }
        return temp2;

    } else if (distance <= rampUpDist + plateauDist) {
        double position = distance - rampUpDist;
        return rampUpTime + position / maxSpeed;
    } else if (distance < rampUpDist + plateauDist + rampDownDist) {
        // time calculations
        /*
            1/2*a*t^2 + t*v0 - d = 0
            t = -b +- sqrt(b^2 - 4*a*c)/(2*a)

        */
        double position = distance - rampUpDist - plateauDist;
        double b = maxSpeed;
        double a = -maxAcc / 2.0;
        double c = -position;
        double root = sqrt(b * b - 4 * a * c);
        double temp1 = (-b + root) / (2 * a);
        double temp2 = (-b - root) / (2 * a);
        if (std::isnan(root)) {
            debugThrow(
                "TrapezoidalMotion failed. Solution is imaginary");  // TODO
                                                                     // Handle
                                                                     // this
            return rampUpTime + plateauTime + rampDownTime;
        }
        if (temp1 > 0 && temp1 < rampDownTime) {
            return rampUpTime + plateauTime + temp1;
        }
        return rampUpTime + plateauTime + temp2;

    } else {
        return rampUpTime + plateauTime + rampDownTime;
    }
}

bool TrapezoidalMotion(double pathLength, double maxSpeed, double maxAcc,
                       double timeIntoLap, double startSpeed, double finalSpeed,
                       double& posOut, double& speedOut) {
    // begin by assuming that there's enough time to get up to full speed
    // we do this by calculating the full ramp-up and ramp-down, then seeing
    // if the distance travelled is too great.  If it's gone too far, this is
    // the "triangle case"

    startSpeed = fmin(startSpeed, maxSpeed);
    finalSpeed = fmin(finalSpeed, maxSpeed);
    double rampUpTime = (maxSpeed - startSpeed) / maxAcc;
    double plateauTime;
    double rampDownTime = (finalSpeed - maxSpeed) / -maxAcc;

    double rampUpDist = rampUpTime * (startSpeed + maxSpeed) / 2.0;
    double plateauDist;
    double rampDownDist = rampDownTime * (maxSpeed + finalSpeed) / 2.0;

    if (rampUpDist + rampDownDist > pathLength) {
        // triangle case: we don't ever hit full speed

        // Calculate what max speed we actually reach (it's less than the
        // parameter passed in).
        // We write an equation for pathLength given maxSpeed, then solve for
        // maxSpeed
        // 	rampUpTime = (maxSpeed - startSpeed) / maxAcc;
        // 	rampDownTime = (finalSpeed - maxSpeed) / -maxAcc;
        // 	pathLength = (startSpeed + maxSpeed)/2*rampUpTime
        // 					+ (maxSpeed + finalSpeed)/2*rampDownTime;
        // We then solve for maxSpeed
        // maxSpeed = sqrt(pathLength*maxAcc + startSpeed*startSpeed +
        // finalSpeed*finalSpeed);
        maxSpeed = sqrt((2 * maxAcc * pathLength + powf(startSpeed, 2) +
                         powf(finalSpeed, 2)) /
                        2.0);

        rampUpTime = (maxSpeed - startSpeed) / maxAcc;
        rampDownTime = (finalSpeed - maxSpeed) / -maxAcc;
        rampUpDist = (startSpeed + maxSpeed) / 2.0 * rampUpTime;
        rampDownDist = (finalSpeed + maxSpeed) / 2.0 * rampDownTime;

        // no plateau
        plateauTime = 0;
        plateauDist = 0;
    } else {
        // trapezoid case: there's a time where we go at maxSpeed for a bit
        plateauDist = pathLength - (rampUpDist + rampDownDist);
        plateauTime = plateauDist / maxSpeed;
    }

    if (timeIntoLap < 0) {
        /// not even started on the path yet
        posOut = 0;
        speedOut = startSpeed;
        return false;
    }
    if (timeIntoLap < rampUpTime) {
        /// on the ramp-up, we're accelerating at @maxAcc
        posOut =
            0.5 * maxAcc * timeIntoLap * timeIntoLap + startSpeed * timeIntoLap;
        speedOut = startSpeed + maxAcc * timeIntoLap;
        return true;
    } else if (timeIntoLap < rampUpTime + plateauTime) {
        /// we're on the plateau
        posOut = rampUpDist + (timeIntoLap - rampUpTime) * maxSpeed;
        speedOut = maxSpeed;
        return true;
    } else if (timeIntoLap < rampUpTime + plateauTime + rampDownTime) {
        /// we're on the ramp down
        double timeIntoRampDown = timeIntoLap - (rampUpTime + plateauTime);
        posOut = 0.5 * (-maxAcc) * timeIntoRampDown * timeIntoRampDown +
                 maxSpeed * timeIntoRampDown + (rampUpDist + plateauDist);
        speedOut = maxSpeed - maxAcc * timeIntoRampDown;
        return true;
    } else {
        /// past the end of the path
        posOut = pathLength;
        speedOut = finalSpeed;
        return false;
    }
}
