#include <algorithm>
#include <cmath>

#include <Geometry2d/Line.hpp>
#include <rj_constants/constants.hpp>
#include <rj_param_utils/param.h>
#include <rj_vision_filter/ball/BallBounce.hpp>
#include <rj_vision_filter/params.hpp>

namespace vision_filter {
DEFINE_NS_FLOAT64(kVisionFilterParamModule, vision_filter::bounce,
                  robot_body_lin_dampen, 0.9,
                  "Linear velocity dampen for bouncing off the circular shell. "
                  "1 means 100% of the velocity is kept after collision. 0 "
                  "means 0% of the velocity is kept after collision.")
DEFINE_NS_FLOAT64(kVisionFilterParamModule, vision_filter::bounce,
                  robot_mouth_lin_dampen, 0.3,
                  "Linear velocity dampen for bouncing off the front mouth. "
                  "1 means 100% of the velocity is kept after collision. 0 "
                  "means 0% of the velocity is kept after collision.")
DEFINE_NS_FLOAT64(kVisionFilterParamModule, vision_filter::bounce,
                  robot_body_angle_dampen, 0.0,
                  "Reflect angle dampen for bouncing off the circular shell. "
                  "1 means 100% of the velocity is kept after collision. 0 "
                  "means 0% of the velocity is kept after collision.")
DEFINE_NS_FLOAT64(kVisionFilterParamModule, vision_filter::bounce,
                  robot_mouth_angle_dampen, 0.0,
                  "Reflect angle dampen for bouncing off the front mouth. "
                  "1 means 100% of the velocity is kept after collision. 0 "
                  "means 0% of the velocity is kept after collision.")
using vision_filter::bounce::PARAM_robot_body_angle_dampen;
using vision_filter::bounce::PARAM_robot_body_lin_dampen;
using vision_filter::bounce::PARAM_robot_mouth_angle_dampen;
using vision_filter::bounce::PARAM_robot_mouth_lin_dampen;

/**
 * Note 0 case returns -1 instead of 0
 * Forced to check with small epsilon since we actually care about 0 being
 * represented correctly as -1. It's ok if the 1.0e-10 doesn't return -1 or 1.
 *
 * This is mostly so the simple test case have the correct behavior
 * If it lands on the 1.0e-10 boundary the code reacts as if there was no hit
 */
int sign(double val) {
    return static_cast<int>(1.0e-10 < val) - static_cast<int>(val <= 1.0e-10);
}

bool BallBounce::CalcBallBounce(const KalmanBall& ball,
                                const std::vector<WorldRobot>& yellow_robots,
                                const std::vector<WorldRobot>& blue_robots,
                                Geometry2d::Point& out_new_vel) {
    // Figures out if there is an intersection and what the resulting velocity
    // should be
    auto find_end_vel = [&ball,
                         &out_new_vel](const std::vector<WorldRobot>& robots) {
        for (const WorldRobot& robot : robots) {
            if (!robot.getIsValid()) {
                continue;
            }

            // Make sure ball is intersecting next frame
            if (!BallInRobot(ball, robot)) {
                continue;
            }

            std::vector<Geometry2d::Point> intersect_pts =
                PossibleBallIntersectionPts(ball, robot);

            // Doesn't intersect
            if (intersect_pts.empty()) {
                continue;
            }

            // Tangent to robot, assuming no interaction
            if (intersect_pts.size() == 1) {
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
            // Line B<->D goes from the center of the robot through the point of
            // intersection between the robot and the ball Since the robot is
            // round (with a flat mouth), any time it hits the robot, we can
            // assume that the it will reflect as if
            //   it hit a flat surface

            // We want to make sure that the ball is never inside the robot when
            // doing this math, so we go back in time This doesn't affect the
            // results since we are just doing vectors
            Geometry2d::Point ball_pos_safe_pt =
                ball.getPos() - ball.getVel().normalized();

            // Find the closest point
            // AKA: the first point the ball will hit off the robot
            // May actually be slightly off because we add the ball radius to
            // the calculation circle Does not account for the mouth just yet
            Geometry2d::Point closest_intersect_pt = intersect_pts.at(0);
            if ((ball_pos_safe_pt - closest_intersect_pt).magsq() >
                (ball_pos_safe_pt - intersect_pts.at(1)).magsq()) {
                closest_intersect_pt = intersect_pts.at(1);
            }

            // This is super easy, just check intersection with the mouth line
            // If the first point in the circle shell intersect is in the mouth
            // angle range Use the line intersect point instead If there is no
            // line intersect point (within that angle range) then the ball is
            // moving across the mouth of the robot Note: No intersection across
            // mouth is not accounted for
            Geometry2d::Line intersect_line =
                Geometry2d::Line(intersect_pts.at(0), intersect_pts.at(1));
            Geometry2d::Point mouth_half_unit_vec =
                Geometry2d::Point(0, 1).rotate(robot.getTheta());
            Geometry2d::Point mouth_center_pos =
                Geometry2d::Point(Robot_MouthRadius, 0)
                    .rotate(robot.getTheta()) +
                robot.getPos();
            Geometry2d::Line mouth_line =
                Geometry2d::Line(mouth_center_pos + mouth_half_unit_vec,
                                 mouth_center_pos - mouth_half_unit_vec);

            Geometry2d::Point mouth_intersect;
            bool intersects =
                intersect_line.intersects(mouth_line, &mouth_intersect);

            // The mouth is a chord across the circle.
            // We have the distance of the chord to the center of the circle
            // We also have the radius of the robot
            const double chord_half_length = pow(Robot_MouthWidth / 2.0, 2);
            bool did_hit_mouth = false;

            // If the line intersect is inside the mouth chord
            if (intersects && (mouth_intersect - mouth_center_pos).magsq() <
                                  chord_half_length) {
                closest_intersect_pt = mouth_intersect;
                did_hit_mouth = true;
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
            Geometry2d::Point intersect_pt_ball_vector =
                ball_pos_safe_pt - closest_intersect_pt;
            // B->D (Officially R->B, but B->D makes more sense visually)
            Geometry2d::Point robot_intersect_pt_vector =
                closest_intersect_pt - robot.getPos();
            Geometry2d::Point robot_intersect_pt_unit_vector =
                robot_intersect_pt_vector.normalized();

            // If it hit the mouth, the reflection line is pointing straight out
            if (did_hit_mouth) {
                robot_intersect_pt_vector =
                    Geometry2d::Point(1, 0).rotate(robot.getTheta());
                robot_intersect_pt_unit_vector = robot_intersect_pt_vector;
            }

            // Project B->A vector onto B->D
            // This is so we can get D->A and D->C later
            double projection_mag = intersect_pt_ball_vector.normalized().dot(
                robot_intersect_pt_unit_vector);
            Geometry2d::Point projection =
                projection_mag * robot_intersect_pt_unit_vector;

            // A->D, which is the same as D->C
            Geometry2d::Point projection_diff =
                projection - intersect_pt_ball_vector;

            Geometry2d::Point intersect_pt_reflection_vector =
                projection + projection_diff;
            Geometry2d::Point intersect_pt_reflection_unit_vector =
                intersect_pt_reflection_vector.normalized();

            // Scale magnitude of velocity by a percentage
            double dampen_lin_coeff = PARAM_robot_body_lin_dampen;
            double dampen_angle_coeff = PARAM_robot_body_angle_dampen;

            if (did_hit_mouth) {
                dampen_lin_coeff = PARAM_robot_mouth_lin_dampen;
                dampen_angle_coeff = PARAM_robot_mouth_angle_dampen;
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
            // We are trying to increase the angle CBD more when angle CBD is
            // large When angle CBD is 0, we want to keep the same angle

            // We dont want any extra rotation when angle CBD is 0 degrees or 90
            // degrees Just to simplify implementation, I'm going to do a
            // triangle
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
            double half_reflect_angle =
                intersect_pt_reflection_unit_vector.angleBetween(
                    robot_intersect_pt_unit_vector);
            double direction = robot_intersect_pt_unit_vector.cross(
                intersect_pt_reflection_unit_vector);
            double extra_rotation_angle = -sign(direction) * half_reflect_angle;
            extra_rotation_angle =
                std::min(extra_rotation_angle, M_PI_2 - extra_rotation_angle) *
                dampen_angle_coeff;

            intersect_pt_reflection_unit_vector =
                intersect_pt_reflection_unit_vector.rotate(
                    extra_rotation_angle);

            out_new_vel =
                intersect_pt_reflection_unit_vector * ball.getVel().mag();

            return true;
        }

        return false;
    };

    bool bounce_found = false;
    bounce_found |= find_end_vel(yellow_robots);
    bounce_found |= find_end_vel(blue_robots);

    return bounce_found;
}

bool BallBounce::BallInRobot(const KalmanBall& ball, const WorldRobot& robot) {
    Geometry2d::Point next_pos =
        ball.getPos() + ball.getVel() * PARAM_vision_loop_dt;

    return (robot.getPos() - next_pos).mag() < Robot_Radius + Ball_Radius;
}

std::vector<Geometry2d::Point> BallBounce::PossibleBallIntersectionPts(
    const KalmanBall& ball, const WorldRobot& robot) {
    // http://mathworld.wolfram.com/Circle-LineIntersection.html

    std::vector<Geometry2d::Point> out;

    // ballPos is the ball pos in robot centered coordinates
    // ballVel is the ball vel vector in robot centered coordinates
    // We really just want a line in the direction of ball vel motion
    Geometry2d::Point ball_pos = ball.getPos() - robot.getPos();
    Geometry2d::Point ball_vel = ball.getPos() + ball.getVel() - robot.getPos();

    // Magnitude of the line
    Geometry2d::Point d = ball_vel - ball_pos;
    double dr = d.mag();
    // Determinant
    double d = ball_pos.x() * ball_vel.y() - ball_pos.y() * ball_vel.x();
    // Assume that two spheres intersection, is similar to the addition of their
    // radius and a point
    double r = Robot_Radius + Ball_Radius;

    // If the ball really isn't moving, just assume no intersection
    // since the math will go to inf
    if (abs(dr) < 1e-5) {
        return out;
    }

    double x1 = d * d.y() + sign(d.y()) * d.x() * sqrt(r * r * dr * dr - d * d);
    x1 /= dr * dr;
    x1 += robot.getPos().x();

    double y1 = -d * d.x() + abs(d.y()) * d.x() * sqrt(r * r * dr * dr - d * d);
    y1 /= dr * dr;
    y1 += robot.getPos().y();

    double x2 = d * d.y() - sign(d.y()) * d.x() * sqrt(r * r * dr * dr - d * d);
    x2 /= dr * dr;
    x2 += robot.getPos().x();

    double y2 = -d * d.x() - abs(d.y()) * sqrt(r * r * dr * dr - d * d);
    y2 /= dr * dr;
    y2 += robot.getPos().y();

    Geometry2d::Point pt1 = Geometry2d::Point(x1, y1);
    Geometry2d::Point pt2 = Geometry2d::Point(x2, y2);

    double delta = r * r * dr * dr - d * d;

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
}  // namespace vision_filter
