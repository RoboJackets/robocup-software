#pragma once

/**
 * This function evaluates a control situation using "bang-bang". The robot
 * accelerates at maxAcceleration until it hits max speed, then decelerates at
 * max acc to the end point.
 *
 * @param[in] 	pathLength The total distance we are trying to travel
 * @param[in] 	timeIntoLap How long since we started moving along the trapezoid
 * @param[in] 	finalSpeed The speed we'd like to be going at the end of the
 * trapezoid
 * @param[out] 	distOut The distance to be at at the given time
 * @param[out] 	speedOut The speed to be at at the given time
 * @return		true if the trapezoid is valid at the given time, false
 * otherwise
 */
bool trapezoidal_motion(double path_length, double max_speed, double max_acc,
                       double time_into_lap, double start_speed, double final_speed,
                       double& pos_out, double& speed_out);

namespace Trapezoidal {
/**
 * Estimates how long it would take to move to a certain distance down a path
 * using Trapezoidal Motion
 *
 * @param distance The certain distance down the path the robot has traveled
 * @param pathLength The total distance we are trying to travel
 * @param maxSpeed The max speed we want to move at during the Trapezoidal
 *     Motion
 * @param maxAcc The max acceleration we want to use during the Trapezoidal
 *     Motion
 * @param startSpeed The speed we're moving at the start of the trapezoid
 * @param finalSpeed The speed we'd like to be going at the end of the trapezoid
 * @return The estimated time it would take for the robot to move that certain
 *     distance down the path
 */
double get_time(double distance, double path_length, double max_speed,
               double max_acc, double start_speed, double final_speed);

}  // namespace Trapezoidal
