#include "trapezoidal_motion.hpp"

#include <cmath>

#include <rj_utils/logging.hpp>

using namespace std;

double Trapezoidal::get_time(double distance, double path_length, double max_speed, double max_acc,
                             double start_speed, double final_speed) {
    start_speed = fmin(start_speed, max_speed);
    final_speed = fmin(final_speed, max_speed);
    double ramp_up_time = (max_speed - start_speed) / max_acc;
    double plateau_time;
    double ramp_down_time = (final_speed - max_speed) / -max_acc;

    double ramp_up_dist = ramp_up_time * (start_speed + max_speed) / 2.0;
    double plateau_dist;
    double ramp_down_dist = ramp_down_time * (max_speed + final_speed) / 2.0;

    if (ramp_up_dist + ramp_down_dist > path_length) {
        // triangle case: we don't ever hit full speed

        // Calculate what max speed we actually reach (it's less than the
        // parameter passed in).
        // We write an equation for path_length given max_speed, then solve for
        // max_speed
        // 	ramp_up_time = (max_speed - start_speed) / max_acc;
        // 	ramp_down_time = (final_speed - max_speed) / -max_acc;
        // 	path_length = (start_speed + max_speed)/2*ramp_up_time
        // 					+ (max_speed + final_speed)/2*ramp_down_time;
        // We then solve for max_speed:
        // max_speed = sqrt(path_length*max_acc + start_speed*start_speed +
        // final_speed*final_speed);
        max_speed =
            sqrt((2 * max_acc * path_length + powf(start_speed, 2) + powf(final_speed, 2)) / 2.0);

        ramp_up_time = (max_speed - start_speed) / max_acc;
        ramp_down_time = (final_speed - max_speed) / -max_acc;
        ramp_up_dist = (start_speed + max_speed) / 2.0 * ramp_up_time;
        ramp_down_dist = (final_speed + max_speed) / 2.0 * ramp_down_time;

        // no plateau
        plateau_time = 0;
        plateau_dist = 0;

    } else {
        // trapezoid case: there's a time where we go at max_speed for a bit
        plateau_dist = path_length - (ramp_up_dist + ramp_down_dist);
        plateau_time = plateau_dist / max_speed;
    }

    if (distance <= 0) {
        return 0;
    }

    if (abs(distance - (ramp_up_dist + plateau_dist + ramp_down_dist)) < 0.00001) {
        return ramp_up_time + plateau_time + ramp_down_time;
    }
    if (distance < ramp_up_dist) {
        // time calculations
        /*
            1/2*a*t^2 + t*v0 - d = 0
            t = -b +- sqrt(b^2 - 4*a*c)/(2*a)

        */
        double b = start_speed;
        double a = max_acc / 2.0;
        double c = -distance;
        double root = sqrt(b * b - 4 * a * c);
        double temp1 = (-b + root) / (2 * a);
        double temp2 = (-b - root) / (2 * a);
        if (std::isnan(root)) {
            // TODO(1576): Handle the case of imaginary solutions.
            rj_utils::debug_throw("TrapezoidalMotion failed. Solution is imaginary");
            return ramp_up_time;
        }
        if (temp1 > 0 && temp1 < ramp_up_time) {
            return temp1;
        }
        return temp2;

    } else if (distance <= ramp_up_dist + plateau_dist) {
        double position = distance - ramp_up_dist;
        return ramp_up_time + position / max_speed;
    } else if (distance < ramp_up_dist + plateau_dist + ramp_down_dist) {
        // time calculations
        /*
            1/2*a*t^2 + t*v0 - d = 0
            t = -b +- sqrt(b^2 - 4*a*c)/(2*a)

        */
        double position = distance - ramp_up_dist - plateau_dist;
        double b = max_speed;
        double a = -max_acc / 2.0;
        double c = -position;
        double root = sqrt(b * b - 4 * a * c);
        double temp1 = (-b + root) / (2 * a);
        double temp2 = (-b - root) / (2 * a);
        if (std::isnan(root)) {
            // TODO(1576) Handle imaginary solutions.
            rj_utils::debug_throw("TrapezoidalMotion failed. Solution is imaginary");
            return ramp_up_time + plateau_time + ramp_down_time;
        }
        if (temp1 > 0 && temp1 < ramp_down_time) {
            return ramp_up_time + plateau_time + temp1;
        }
        return ramp_up_time + plateau_time + temp2;

    } else {
        return ramp_up_time + plateau_time + ramp_down_time;
    }
}

bool Trapezoidal::trapezoidal_motion(double path_length, double max_speed, double max_acc, double time_into_lap,
                        double start_speed, double final_speed, double& pos_out,
                        double& speed_out) {
    // begin by assuming that there's enough time to get up to full speed
    // we do this by calculating the full ramp-up and ramp-down, then seeing
    // if the distance travelled is too great.  If it's gone too far, this is
    // the "triangle case"

    start_speed = fmin(start_speed, max_speed);
    final_speed = fmin(final_speed, max_speed);
    double ramp_up_time = (max_speed - start_speed) / max_acc;
    double plateau_time;
    double ramp_down_time = (final_speed - max_speed) / -max_acc;

    double ramp_up_dist = ramp_up_time * (start_speed + max_speed) / 2.0;
    double plateau_dist;
    double ramp_down_dist = ramp_down_time * (max_speed + final_speed) / 2.0;

    if (ramp_up_dist + ramp_down_dist > path_length) {
        // triangle case: we don't ever hit full speed

        // Calculate what max speed we actually reach (it's less than the
        // parameter passed in).
        // We write an equation for path_length given max_speed, then solve for
        // max_speed
        // 	ramp_up_time = (max_speed - start_speed) / max_acc;
        // 	ramp_down_time = (final_speed - max_speed) / -max_acc;
        // 	path_length = (start_speed + max_speed)/2*ramp_up_time
        // 					+ (max_speed + final_speed)/2*ramp_down_time;
        // We then solve for max_speed
        // max_speed = sqrt(path_length*max_acc + start_speed*start_speed +
        // final_speed*final_speed);
        max_speed =
            sqrt((2 * max_acc * path_length + powf(start_speed, 2) + powf(final_speed, 2)) / 2.0);

        ramp_up_time = (max_speed - start_speed) / max_acc;
        ramp_down_time = (final_speed - max_speed) / -max_acc;
        ramp_up_dist = (start_speed + max_speed) / 2.0 * ramp_up_time;
        ramp_down_dist = (final_speed + max_speed) / 2.0 * ramp_down_time;

        // no plateau
        plateau_time = 0;
        plateau_dist = 0;
    } else {
        // trapezoid case: there's a time where we go at max_speed for a bit
        plateau_dist = path_length - (ramp_up_dist + ramp_down_dist);
        plateau_time = plateau_dist / max_speed;
    }

    if (time_into_lap < 0) {
        /// not even started on the path yet
        pos_out = 0;
        speed_out = start_speed;
        return false;
    }
    if (time_into_lap < ramp_up_time) {
        /// on the ramp-up, we're accelerating at @max_acc
        pos_out = 0.5 * max_acc * time_into_lap * time_into_lap + start_speed * time_into_lap;
        speed_out = start_speed + max_acc * time_into_lap;
        return true;
    } else if (time_into_lap < ramp_up_time + plateau_time) {
        /// we're on the plateau
        pos_out = ramp_up_dist + (time_into_lap - ramp_up_time) * max_speed;
        speed_out = max_speed;
        return true;
    } else if (time_into_lap < ramp_up_time + plateau_time + ramp_down_time) {
        /// we're on the ramp down
        double time_into_ramp_down = time_into_lap - (ramp_up_time + plateau_time);
        pos_out = 0.5 * (-max_acc) * time_into_ramp_down * time_into_ramp_down +
                  max_speed * time_into_ramp_down + (ramp_up_dist + plateau_dist);
        speed_out = max_speed - max_acc * time_into_ramp_down;
        return true;
    } else {
        /// past the end of the path
        pos_out = path_length;
        speed_out = final_speed;
        return false;
    }
}
