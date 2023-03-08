#include "world_state.hpp"
using namespace std;

BallState BallState::predict_at(RJ::Time time) const {
    // If the estimate isn't valid, just return an invalid ball.
    if (!visible) {
        return BallState();
    }

    if (time < timestamp) {
        throw std::runtime_error("Estimated Time can't be before observation time.");
    }

    auto dt = RJ::Seconds(time - timestamp);

    const auto s0 = velocity.mag();

    // If the ball is stopped just return it.
    if (s0 == 0) {
        return *this;
    }

    double speed = 0.0;
    double distance = 0.0;

    double max_time = s0 / soccer::physics::PARAM_ball_decay_constant;
    if (dt.count() >= max_time) {
        speed = 0;
        distance =
            s0 * max_time - pow(max_time, 2) / 2.0 * soccer::physics::PARAM_ball_decay_constant;
    } else {
        speed = s0 - (dt.count() * soccer::physics::PARAM_ball_decay_constant);
        distance =
            s0 * dt.count() - pow(dt.count(), 2) / 2.0 * soccer::physics::PARAM_ball_decay_constant;
    }

    return BallState(position + velocity.normalized(distance), velocity.normalized(speed), time);
}

BallState BallState::predict_in(RJ::Seconds seconds) const {
    return predict_at(timestamp + seconds);
}

RJ::Time BallState::query_time_near(rj_geometry::Point near_to, rj_geometry::Point* out) const {
    // If the ball isn't moving we're as close as we're ever going to get.
    if (velocity.mag() == 0) {
        if (out != nullptr) {
            *out = position;
        }
        return timestamp;
    }

    // Otherwise, find the closest point on the ball's line of travel...
    rj_geometry::Segment segment(position, predict_at(RJ::Time::max()).position);
    rj_geometry::Point nearest = segment.nearest_point(near_to);

    double distance_to_nearest = (position - nearest).mag();

    std::optional<RJ::Seconds> maybe_seconds = query_seconds_to_dist(distance_to_nearest);

    RJ::Seconds seconds;
    if (maybe_seconds.has_value()) {
        seconds = *maybe_seconds;
    } else {
        seconds = query_stop_time();
    }

    if (out != nullptr) {
        *out = predict_in(seconds).position;
    }

    return timestamp + seconds;
}

RJ::Seconds BallState::query_seconds_near(rj_geometry::Point near_to,
                                          rj_geometry::Point* out) const {
    return query_time_near(near_to, out) - timestamp;
}

RJ::Seconds BallState::query_stop_time(rj_geometry::Point* out) const {
    double speed = velocity.mag();

    if (out != nullptr) {
        // vf^2 - vi^2 = 2ad => d = -vi^2 / 2a
        *out = position + velocity.normalized(std::pow(speed, 2) /
                                              (2 * soccer::physics::PARAM_ball_decay_constant));
    }

    // Use the formula for time until zero velocity:
    // 0 = v = vi + at => t = -vi / a
    return RJ::Seconds(speed / soccer::physics::PARAM_ball_decay_constant);
}

rj_geometry::Point BallState::query_stop_position() const {
    rj_geometry::Point point;
    [[maybe_unused]] auto stop_time = query_stop_time(&point);
    return point;
}

std::optional<RJ::Seconds> BallState::query_seconds_to_dist(double distance) const {
    // vf^2 - vi^2 = 2ad => vf = sqrt(vi^2 + 2ad)
    double speed = velocity.mag();
    double vf_sq = std::pow(speed, 2) - 2 * soccer::physics::PARAM_ball_decay_constant * distance;

    // If vf^2 is negative, the ball will never travel the desired distance.
    // Return nullopt.
    if (vf_sq < 0) {
        return std::nullopt;
    }

    // Otherwise, use t = (vf - vi) / a
    return RJ::Seconds(speed - std::sqrt(vf_sq)) / soccer::physics::PARAM_ball_decay_constant;
}

planning::Trajectory BallState::make_trajectory() const {
    using namespace rj_geometry;

    // The trajectory interface fits cubic splines. Luckily, a cubic spline
    // between two instants that can be connected by a constant acceleration
    // (like we have here) will be, and so we can use this for our trajectory.
    // The start point is the current instant in time, and the endpoint is the
    // stopping point.
    planning::RobotInstant instant0;
    instant0.pose = Pose(position, 0);
    instant0.velocity = Twist(velocity, 0);
    instant0.stamp = timestamp;

    Point stop_position;
    RJ::Time stop_time = timestamp + query_stop_time(&stop_position);
    planning::RobotInstant instant1{Pose{stop_position, 0}, Twist::zero(), stop_time};

    return planning::Trajectory({instant0, instant1});
}

/*
Field::Field(float length_m,
            float width_m;
            float border_m;
            float line_width_m;
            float goal_width_m;
            float goal_depth_m;
            float goal_height_m;
            float def_area_short_dist_m;
            float def_area_long_dist_m;
            float center_radius_m;
            float center_diameter_m;
            float goal_flat_m;
            float floor_length_m;
            float floor_width_m;
            float def_area_x_right_coord;
            float def_area_x_left_coord;
            float field_x_right_coord;
            float field_x_left_coord;){
*/

Field::Field() {
    // create subscriptions (to get all the data for the field)? --> FieldDimensions.msg
    // create publishers (to send data to whomever)?

    /*
        field_dimensions_sub_ = this->create_subscription<rj_msgs::msg::FieldDimensions>(
                             "/config/field_dimensions",
                             rclcpp::QoS(1).transient_local(),
                             [this](const rj_msgs::msg::FieldDimensions::SharedPtr length) {  //
       NOLINT this.__length_m = length;
                             },
                             [this](const rj_msgs::msg::FieldDimensions::SharedPtr width) {  //
       NOLINT this.__width_m = width;
                             },
                             [this](const rj_msgs::msg::FieldDimensions::SharedPtr border) {  //
       NOLINT this.__border_m = border;
                             },
                             [this](const rj_msgs::msg::FieldDimensions::SharedPtr line_width) {  //
       NOLINT this.__line_width_m = line_width;
                             },
                             [this](const rj_msgs::msg::FieldDimensions::SharedPtr goal_width) {  //
       NOLINT this.__goal_width_m = goal_width;
                             },
                             [this](const rj_msgs::msg::FieldDimensions::SharedPtr goal_depth) {  //
       NOLINT this.__goal_depth_m = goal_depth;
                             },
                             [this](const rj_msgs::msg::FieldDimensions::SharedPtr goal_height) { //
       NOLINT this.__goal_height_m = goal_height;
                             },
                             [this](const rj_msgs::msg::FieldDimensions::SharedPtr
       penalty_short_dist) {  // NOLINT this.__def_area_short_dist_m = penalty_short_dist;
                             },
                             [this](const rj_msgs::msg::FieldDimensions::SharedPtr
       penalty_long_dist) {  // NOLINT this.__def_area_long_dist_m = penalty_long_dist;
                             },
                             [this](const rj_msgs::msg::FieldDimensions::SharedPtr center_radius) {
       // NOLINT this.__center_radius_m = center_radius;
                             },
                             [this](const rj_msgs::msg::FieldDimensions::SharedPtr center_diameter)
       {  // NOLINT this.__center_diameter_m = center_diameter;
                             },
                             [this](const rj_msgs::msg::FieldDimensions::SharedPtr goal_flat) {  //
       NOLINT this.__goal_flat_m = goal_flat;
                             },
                             [this](const rj_msgs::msg::FieldDimensions::SharedPtr floor_length) {
       // NOLINT this.__floor_length_m = floor_length;
                             },
                             [this](const rj_msgs::msg::FieldDimensions::SharedPtr floor_width) { //
       NOLINT this.__floor_width_m = floor_width;
                             }
                        );
    */

    float __length_m = 0;
    float __width_m = 0;
    float __border_m = 0;
    float __line_width_m = 0;
    float __goal_width_m = 0;
    float __goal_depth_m = 0;
    float __goal_height_m = 0;
    float __def_area_short_dist_m = 0;
    float __def_area_long_dist_m = 0;
    float __center_radius_m = 0;
    float __center_diameter_m = 0;
    float __goal_flat_m = 0;
    float __floor_length_m = 0;
    float __floor_width_m = 0;

    float __def_area_x_right_coord = def_area_long_dist_m / 2;
    float __def_area_x_left_coord = -(def_area_long_dist_m / 2);
    float __field_x_right_coord = width_m / 2;
    float __field_x_left_coord = -(width_m / 2);

    tuple<float, float> top_left, bottom_right;

    boxCoordinates __our_defense_area;
    top_left = make_tuple(__def_area_x_left_coord, __def_area_short_dist_m)
    bottom_right = make_tuple(__def_area_x_right_coord, 0.0));
    __our_defense_area = this.create_box(top_left, bottom_right);

    boxCoordinates __opp_defense_area;
    top_left = make_tuple(__def_area_x_left_coord, __length_m) bottom_right =
        make_tuple(__def_area_x_right_coord, (__length - __def_area_short_dist_m));
    __opp_defense_area = this.create_box(top_left, bottom_right);

    boxCoordinates __field_coordinates;
    top_left = make_tuple(__field_x_left_coord, __length_m) bottom_right =
        make_tuple(__field_x_right_coord, 0.0);
    __opp_defense_area = this.create_box(top_left, bottom_right);
}

tuple<float, float> Field::our_goal_loc() {
    /**
    Convenience function for getting our goal location
    return: the location of our goal - its always (0,0)
    */
    return make_tuple(0.0, 0.0);
}

tuple<float, float> Field::center_field_loc() {
    /**
    Convenience function for getting the center field location
    :return: the location of the center of the field
    */
    return make_tuple(0.0, this.length_m / 2);
}

tuple<float, float> Field::their_goal_loc() {
    /**
    Convenience function for getting the opponents field location
    :return: the location of the opponents goal
    */
    return make_tuple(0.0, this.length_m);
}

vector<tuple<float, float>> Field::our_defense_area_coordinates() {
    /**
    Convenience function for getting our defense area locations
    Note: each coordinate starts from top left and continues normal order
    :return: the list of points for our defense area locations
    */
    return this.__our_defense_area.box_coords_list();
}

vector<tuple<float, float>> Field::opp_defense_area_coordinates() {
    /**
    Convenience function for getting oppenent defense area locations
    Note: each coordinate starts from top left and continues normal order
    :return: the list of points for opponent defense area locations
    */
    return this.__opp_defense_area.box_coords_list();
}

vector<tuple<float, float>> Field::our_goal_post_coordinates() {
    /*
    Convenience function for getting our goal post coordinates
    :return: the list of points for our goal post locations
    */
    vector < tuple<float, float> our_goal_post;

    our_goal_post.push_back(make_tuple(-__goal_width_m / 2, 0.0));
    our_goal_post.push_back(make_tuple(__goal_width_m / 2, 0.0));

    return our_goal_post;
}

vector<tuple<float, float>> Field::their_goal_post_coordinates() {
    /*
    Convenience function for getting their goal post coordinates
    :return: the list of points for their goal post locations
    */
    vector < tuple<float, float> their_goal_post;

    our_goal_post.push_back(make_tuple(-__goal_width_m / 2, __length_m));
    our_goal_post.push_back(make_tuple(__goal_width_m / 2, __length_m));

    return their_goal_post;
}

tuple<float, float> Field::our_left_corner() {
    /*
    :return: the coords of the left corner of our side of the field
    */
    return __field_coordinates[3];
}

tuple<float, float> Field::our_right_corner() {
    /*
    :return: the coords of the right corner of our side of the field
    */
    return __field_coordinates[2];
}

tuple<float, float> Field::their_left_corner() {
    /*
    :return: the coords of the left corner of their side of the field
    */
    return __field_coordinates[0];
}

tuple<float, float> Field::their_right_corner() {
    /*
    :return: the coords of the right corner of their side of the field
    */
    return __field_coordinates[1];
}

float Field::floor_width_m() {
    /*
    :return: width of full field (including borders)
    */
    return this.__width_m + 2 * this.__border_m;
}

float Field::def_area_x_left_coord() {
    /*
    :return: left x coordinate of the defense area
    */
    return this.__def_area_x_left_coord;
}

float Field::def_area_x_right_coord() {
    /*
    :return: right x coordinate of the defense area
    */
    return this.__def_area_x_right_coord;
}

float Field::floor_length_m() {
    /*
    :return: length of full field (including borders)
    */
    return this.__length_m + 2 * this.__border_m;
}

float Field::goal_flat_m() {
    /*
    :return: CHECK ON THIS ONE
    */
    return this.__goal_flat_m;
}

float Field::center_diameter_m() {
    /*
    :return: returns the diameter of the center of the field
    */
    return this.__center_diameter_m;
}

float Field::center_radius_m() {
    /*
    :return: returns the radius of the center of the field
    */
    return this.__center_radius_m;
}

float Field::def_area_long_dist_m() {
    /*
    :return: double check on this one
    */
    return this.__def_area_long_dist_m;
}

float Field::def_area_short_dist_m() {
    /*
    :return: double check on this one
    */
    return this.__def_area_short_dist_m;
}

float Field::border_m() {
    /*
    :return: The size of the border of the field
    */
    return this.__border_m;
}

float Field::line_width_m() {
    /*
    :return: The width of the lines of the field
    */
    return this.__line_width_m;
}

float Field::length_m() {
    /*
    :return: The length of the field in meters
    */
    return this.__length_m;
}

float Field::width_m() {
    /*
    :return: the width of the field in meters
    */
    return this.__width_m;
}

float Field::goal_width_m() {
    /*
    :return: the width of the goals in meters
    */
    return this.__goal_width_m;
}

float Field::goal_depth_m() {
    /*
    :return: the depth of the goals in meters
    */
    return this.__goal_depth_m;
}

float Field::goal_height_m() {
    /*
    :return: the height of the goals in meters
    */
    return this.__goal_height_m;
}

boxCoordinates Field::create_box(tuple<float, float> top_left, tuple<float, float> bottom_right) {
    /*
    Inputs the top_left and bottom right corners of a box and creates a boxCoordinates structure.
    :return: the boxCoordinate struct based on the input.
    */
    boxCoordinates box;
    box.top_left = top_left;
    box.bottom_right = bottom_right;

    return box;
}

vector<tuple<float, float>> boxCoordinates::box_coords_list() {
    /**
    Creates a list of the 4 box coordinates.
    Note: each coordinate starts from top left and continues normal order (clockwise)
    :return: the list of the 4 coordinates (tuples) of the box.
    */

    vector<tuple<float, float>> box_coords;

    box_coords.push_back(this.top_left_coord);
    box_coords.push_back(make_tuple(get<0>(this.bottom_right_coord), get<1>(this.top_left_coord)));
    box_coords.push_back(this.bottom_right_coord);
    box_coordsa.push_back(make_tuple(get<0>(this.top_left_coord), get<1>(this.bottom_right_coord)));

    return box_coords;
}

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/msg/field_dimensions.hpp>

#include "world_state.hpp"

using namespace std;

BallState BallState::predict_at(RJ::Time time) const {
    // If the estimate isn't valid, just return an invalid ball.
    if (!visible) {
        return BallState();
    }

    if (time < timestamp) {
        throw std::runtime_error("Estimated Time can't be before observation time.");
    }

    auto dt = RJ::Seconds(time - timestamp);

    const auto s0 = velocity.mag();

    // If the ball is stopped just return it.
    if (s0 == 0) {
        return *this;
    }

    double speed = 0.0;
    double distance = 0.0;

    double max_time = s0 / soccer::physics::PARAM_ball_decay_constant;
    if (dt.count() >= max_time) {
        speed = 0;
        distance =
            s0 * max_time - pow(max_time, 2) / 2.0 * soccer::physics::PARAM_ball_decay_constant;
    } else {
        speed = s0 - (dt.count() * soccer::physics::PARAM_ball_decay_constant);
        distance =
            s0 * dt.count() - pow(dt.count(), 2) / 2.0 * soccer::physics::PARAM_ball_decay_constant;
    }

    return BallState(position + velocity.normalized(distance), velocity.normalized(speed), time);
}

BallState BallState::predict_in(RJ::Seconds seconds) const {
    return predict_at(timestamp + seconds);
}

RJ::Time BallState::query_time_near(rj_geometry::Point near_to, rj_geometry::Point* out) const {
    // If the ball isn't moving we're as close as we're ever going to get.
    if (velocity.mag() == 0) {
        if (out != nullptr) {
            *out = position;
        }
        return timestamp;
    }

    // Otherwise, find the closest point on the ball's line of travel...
    rj_geometry::Segment segment(position, predict_at(RJ::Time::max()).position);
    rj_geometry::Point nearest = segment.nearest_point(near_to);

    double distance_to_nearest = (position - nearest).mag();

    std::optional<RJ::Seconds> maybe_seconds = query_seconds_to_dist(distance_to_nearest);

    RJ::Seconds seconds;
    if (maybe_seconds.has_value()) {
        seconds = *maybe_seconds;
    } else {
        seconds = query_stop_time();
    }

    if (out != nullptr) {
        *out = predict_in(seconds).position;
    }

    return timestamp + seconds;
}

RJ::Seconds BallState::query_seconds_near(rj_geometry::Point near_to,
                                          rj_geometry::Point* out) const {
    return query_time_near(near_to, out) - timestamp;
}

RJ::Seconds BallState::query_stop_time(rj_geometry::Point* out) const {
    double speed = velocity.mag();

    if (out != nullptr) {
        // vf^2 - vi^2 = 2ad => d = -vi^2 / 2a
        *out = position + velocity.normalized(std::pow(speed, 2) /
                                              (2 * soccer::physics::PARAM_ball_decay_constant));
    }

    // Use the formula for time until zero velocity:
    // 0 = v = vi + at => t = -vi / a
    return RJ::Seconds(speed / soccer::physics::PARAM_ball_decay_constant);
}

rj_geometry::Point BallState::query_stop_position() const {
    rj_geometry::Point point;
    [[maybe_unused]] auto stop_time = query_stop_time(&point);
    return point;
}

std::optional<RJ::Seconds> BallState::query_seconds_to_dist(double distance) const {
    // vf^2 - vi^2 = 2ad => vf = sqrt(vi^2 + 2ad)
    double speed = velocity.mag();
    double vf_sq = std::pow(speed, 2) - 2 * soccer::physics::PARAM_ball_decay_constant * distance;

    // If vf^2 is negative, the ball will never travel the desired distance.
    // Return nullopt.
    if (vf_sq < 0) {
        return std::nullopt;
    }

    // Otherwise, use t = (vf - vi) / a
    return RJ::Seconds(speed - std::sqrt(vf_sq)) / soccer::physics::PARAM_ball_decay_constant;
}

planning::Trajectory BallState::make_trajectory() const {
    using namespace rj_geometry;

    // The trajectory interface fits cubic splines. Luckily, a cubic spline
    // between two instants that can be connected by a constant acceleration
    // (like we have here) will be, and so we can use this for our trajectory.
    // The start point is the current instant in time, and the endpoint is the
    // stopping point.
    planning::RobotInstant instant0;
    instant0.pose = Pose(position, 0);
    instant0.velocity = Twist(velocity, 0);
    instant0.stamp = timestamp;

    Point stop_position;
    RJ::Time stop_time = timestamp + query_stop_time(&stop_position);
    planning::RobotInstant instant1{Pose{stop_position, 0}, Twist::zero(), stop_time};

    return planning::Trajectory({instant0, instant1});
}

/*
Field::Field(float length_m,
            float width_m;
            float border_m;
            float line_width_m;
            float goal_width_m;
            float goal_depth_m;
            float goal_height_m;
            float def_area_short_dist_m;
            float def_area_long_dist_m;
            float center_radius_m;
            float center_diameter_m;
            float goal_flat_m;
            float floor_length_m;
            float floor_width_m;
            float def_area_x_right_coord;
            float def_area_x_left_coord;
            float field_x_right_coord;
            float field_x_left_coord;){
*/

Field::Field() {
    // create subscriptions (to get all the data for the field)? --> FieldDimensions.msg
    // create publishers (to send data to whomever)?

    field_dimensions_sub_ = this->create_subscription<rj_msgs::msg::FieldDimensions>(
        "/config/field_dimensions", rclcpp::QoS(1).transient_local(),
        [this](const rj_msgs::msg::FieldDimensions::SharedPtr length) {  // NOLINT
            this.__length_m = length;
        },
        [this](const rj_msgs::msg::FieldDimensions::SharedPtr width) {  // NOLINT
            this.__width_m = width;
        },
        [this](const rj_msgs::msg::FieldDimensions::SharedPtr border) {  // NOLINT
            this.__border_m = border;
        },
        [this](const rj_msgs::msg::FieldDimensions::SharedPtr line_width) {  // NOLINT
            this.__line_width_m = line_width;
        },
        [this](const rj_msgs::msg::FieldDimensions::SharedPtr goal_width) {  // NOLINT
            this.__goal_width_m = goal_width;
        },
        [this](const rj_msgs::msg::FieldDimensions::SharedPtr goal_depth) {  // NOLINT
            this.__goal_depth_m = goal_depth;
        },
        [this](const rj_msgs::msg::FieldDimensions::SharedPtr goal_height) {  // NOLINT
            this.__goal_height_m = goal_height;
        },
        [this](const rj_msgs::msg::FieldDimensions::SharedPtr penalty_short_dist) {  // NOLINT
            this.__def_area_short_dist_m = penalty_short_dist;
        },
        [this](const rj_msgs::msg::FieldDimensions::SharedPtr penalty_long_dist) {  // NOLINT
            this.__def_area_long_dist_m = penalty_long_dist;
        },
        [this](const rj_msgs::msg::FieldDimensions::SharedPtr center_radius) {  // NOLINT
            this.__center_radius_m = center_radius;
        },
        [this](const rj_msgs::msg::FieldDimensions::SharedPtr center_diameter) {  // NOLINT
            this.__center_diameter_m = center_diameter;
        },
        [this](const rj_msgs::msg::FieldDimensions::SharedPtr goal_flat) {  // NOLINT
            this.__goal_flat_m = goal_flat;
        },
        [this](const rj_msgs::msg::FieldDimensions::SharedPtr floor_length) {  // NOLINT
            this.__floor_length_m = floor_length;
        },
        [this](const rj_msgs::msg::FieldDimensions::SharedPtr floor_width) {  // NOLINT
            this.__floor_width_m = floor_width;
        });

    /*
        float __length_m = 0;
        float __width_m = 0;
        float __border_m = 0;
        float __line_width_m = 0;
        float __goal_width_m = 0;
        float __goal_depth_m = 0;
        float __goal_height_m = 0;
        float __def_area_short_dist_m = 0;
        float __def_area_long_dist_m = 0;
        float __center_radius_m = 0;
        float __center_diameter_m = 0;
        float __goal_flat_m = 0;
        float __floor_length_m = 0;
        float __floor_width_m = 0;
    */

    float __def_area_x_right_coord = __def_area_long_dist_m / 2;
    float __def_area_x_left_coord = -(__def_area_long_dist_m / 2);
    float __field_x_right_coord = __width_m / 2;
    float __field_x_left_coord = -(__width_m / 2);

    tuple<float, float> top_left, bottom_right;

    boxCoordinates __our_defense_area;
    top_left = make_tuple(__def_area_x_left_coord, __def_area_short_dist_m);
    bottom_right = make_tuple(__def_area_x_right_coord, 0.0);
    __our_defense_area = this.create_box(top_left, bottom_right);

    boxCoordinates __opp_defense_area;
    top_left = make_tuple(__def_area_x_left_coord, __length_m);
    bottom_right = make_tuple(__def_area_x_right_coord, (__length - __def_area_short_dist_m));
    __opp_defense_area = this.create_box(top_left, bottom_right);

    boxCoordinates __field_coordinates;
    top_left = make_tuple(__field_x_left_coord, __length_m);
    bottom_right = make_tuple(__field_x_right_coord, 0.0);
    __opp_defense_area = this.create_box(top_left, bottom_right);
}

tuple<float, float> Field::our_goal_loc() {
    /**
    Convenience function for getting our goal location
    return: the location of our goal - its always (0,0)
    */
    return make_tuple(0.0, 0.0);
}

tuple<float, float> Field::center_field_loc() {
    /**
    Convenience function for getting the center field location
    :return: the location of the center of the field
    */
    return make_tuple(0.0, this.length_m / 2);
}

tuple<float, float> Field::their_goal_loc() {
    /**
    Convenience function for getting the opponents field location
    :return: the location of the opponents goal
    */
    return make_tuple(0.0, this.length_m);
}

vector<tuple<float, float>> Field::our_defense_area_coordinates() {
    /**
    Convenience function for getting our defense area locations
    Note: each coordinate starts from top left and continues normal order
    :return: the list of points for our defense area locations
    */
    return this.__our_defense_area.box_coords_list();
}

vector<tuple<float, float>> Field::opp_defense_area_coordinates() {
    /**
    Convenience function for getting oppenent defense area locations
    Note: each coordinate starts from top left and continues normal order
    :return: the list of points for opponent defense area locations
    */
    return this.__opp_defense_area.box_coords_list();
}

vector<tuple<float, float>> Field::our_goal_post_coordinates() {
    /*
    Convenience function for getting our goal post coordinates
    :return: the list of points for our goal post locations
    */
    vector < tuple<float, float> our_goal_post;

    our_goal_post.push_back(make_tuple(-__goal_width_m / 2, 0.0));
    our_goal_post.push_back(make_tuple(__goal_width_m / 2, 0.0));

    return our_goal_post;
}

vector<tuple<float, float>> Field::their_goal_post_coordinates() {
    /*
    Convenience function for getting their goal post coordinates
    :return: the list of points for their goal post locations
    */
    vector < tuple<float, float> their_goal_post;

    our_goal_post.push_back(make_tuple(-__goal_width_m / 2, __length_m));
    our_goal_post.push_back(make_tuple(__goal_width_m / 2, __length_m));

    return their_goal_post;
}

tuple<float, float> Field::our_left_corner() {
    /*
    :return: the coords of the left corner of our side of the field
    */
    return __field_coordinates[3];
}

tuple<float, float> Field::our_right_corner() {
    /*
    :return: the coords of the right corner of our side of the field
    */
    return __field_coordinates[2];
}

tuple<float, float> Field::their_left_corner() {
    /*
    :return: the coords of the left corner of their side of the field
    */
    return __field_coordinates[0];
}

tuple<float, float> Field::their_right_corner() {
    /*
    :return: the coords of the right corner of their side of the field
    */
    return __field_coordinates[1];
}

float Field::floor_width_m() {
    /*
    :return: width of full field (including borders)
    */
    return this.__width_m + 2 * this.__border_m;
}

float Field::def_area_x_left_coord() {
    /*
    :return: left x coordinate of the defense area
    */
    return this.__def_area_x_left_coord;
}

float Field::def_area_x_right_coord() {
    /*
    :return: right x coordinate of the defense area
    */
    return this.__def_area_x_right_coord;
}

float Field::floor_length_m() {
    /*
    :return: length of full field (including borders)
    */
    return this.__length_m + 2 * this.__border_m;
}

float Field::goal_flat_m() {
    /*
    :return: CHECK ON THIS ONE
    */
    return this.__goal_flat_m;
}

float Field::center_diameter_m() {
    /*
    :return: returns the diameter of the center of the field
    */
    return this.__center_diameter_m;
}

float Field::center_radius_m() {
    /*
    :return: returns the radius of the center of the field
    */
    return this.__center_radius_m;
}

float Field::def_area_long_dist_m() {
    /*
    :return: double check on this one
    */
    return this.__def_area_long_dist_m;
}

float Field::def_area_short_dist_m() {
    /*
    :return: double check on this one
    */
    return this.__def_area_short_dist_m;
}

float Field::border_m() {
    /*
    :return: The size of the border of the field
    */
    return this.__border_m;
}

float Field::line_width_m() {
    /*
    :return: The width of the lines of the field
    */
    return this.__line_width_m;
}

float Field::length_m() {
    /*
    :return: The length of the field in meters
    */
    return this.__length_m;
}

float Field::width_m() {
    /*
    :return: the width of the field in meters
    */
    return this.__width_m;
}

float Field::goal_width_m() {
    /*
    :return: the width of the goals in meters
    */
    return this.__goal_width_m;
}

float Field::goal_depth_m() {
    /*
    :return: the depth of the goals in meters
    */
    return this.__goal_depth_m;
}

float Field::goal_height_m() {
    /*
    :return: the height of the goals in meters
    */
    return this.__goal_height_m;
}

boxCoordinates Field::create_box(tuple<float, float> top_left, tuple<float, float> bottom_right) {
    /*
    Inputs the top_left and bottom right corners of a box and creates a boxCoordinates structure.
    :return: the boxCoordinate struct based on the input.
    */
    boxCoordinates box;
    box.top_left = top_left;
    box.bottom_right = bottom_right;

    return box;
}

vector<tuple<float, float>> boxCoordinates::box_coords_list() {
    /**
    Creates a list of the 4 box coordinates.
    Note: each coordinate starts from top left and continues normal order (clockwise)
    :return: the list of the 4 coordinates (tuples) of the box.
    */

    vector<tuple<float, float>> box_coords;

    box_coords.push_back(this.top_left_coord);
    box_coords.push_back(make_tuple(get<0>(this.bottom_right_coord), get<1>(this.top_left_coord)));
    box_coords.push_back(this.bottom_right_coord);
    box_coordsa.push_back(make_tuple(get<0>(this.top_left_coord), get<1>(this.bottom_right_coord)));

    return box_coords;
}
