#pragma once

#include <cfloat>
#include <cmath>

#include <rj_convert/ros_convert.hpp>
#include <rj_geometry/arc.hpp>
#include <rj_geometry/circle.hpp>
#include <rj_geometry/composite_shape.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/line.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/polygon.hpp>
#include <rj_geometry/rect.hpp>
#include <rj_msgs/msg/field_dimensions.hpp>

using namespace std;

/// This class contains constants defining the layout of the field.
/// See the official SSL rules page for a detailed diagram:
/// http://robocupssl.cpe.ku.ac.th/rules:main
struct FieldDimensions {
    using Msg = rj_msgs::msg::FieldDimensions;

    float length() const { return length_; }
    float width() const { return width_; }

    /** the distance from the edge of the field to the border line */
    float border() const { return border_; }

    /** The width of the border lines */
    float line_width() const { return line_width_; }

    float goal_width() const { return goal_width_; }
    float goal_depth() const { return goal_depth_; }
    float goal_height() const { return goal_height_; }

    /** Dimensions of the rectangular penalty zone */
    float penalty_short_dist() const { return penalty_short_dist_; }
    float penalty_long_dist() const { return penalty_long_dist_; }

    /** diameter of the center circle */
    float center_radius() const { return center_radius_; }
    float center_diameter() const { return center_diameter_; }

    /** flat area for defence markings */
    float goal_flat() const { return goal_flat_; }

    float floor_length() const { return floor_length_; }
    float floor_width() const { return floor_width_; }

    /** penalty right and left coords */

    float penalty_x_right_coord() const { return penalty_x_right_coord_; }
    float penalty_x_left_coord() const { return penalty_x_left_coord_; }

    /** field right and left coords */

    float field_x_right_coord() const { return field_x_right_coord_; }
    float field_x_left_coord() const { return field_x_left_coord_; }

    /** width and length of floor */

    float floor_border_width() const { return floor_border_width_; }
    float floor_border_length() const { return floor_border_length_; }

    rj_geometry::Point our_goal_loc() const { return our_goal_loc_; }
    rj_geometry::Point center_field_loc() const { return center_field_loc_; }
    rj_geometry::Point their_goal_loc() const { return their_goal_loc_; }

    rj_geometry::Rect our_penalty_area_coordinates() const { return our_penalty_area_coordinates_; }
    rj_geometry::Rect their_penalty_area_coordinates() const {
        return their_penalty_area_coordinates_;
    }

    rj_geometry::Point our_left_goal_post_coordinate() const {
        return our_left_goal_post_coordinate_;
    }
    rj_geometry::Point our_right_goal_post_coordinate() const {
        return our_right_goal_post_coordinate_;
    }

    rj_geometry::Point their_left_goal_post_coordinate() const {
        return their_left_goal_post_coordinate_;
    }
    rj_geometry::Point their_right_goal_post_coordinate() const {
        return their_right_goal_post_coordinate_;
    }

    rj_geometry::Point our_left_corner() const { return our_left_corner_; }
    rj_geometry::Point our_right_corner() const { return our_right_corner_; }
    rj_geometry::Point their_left_corner() const { return their_left_corner_; }
    rj_geometry::Point their_right_corner() const { return their_right_corner_; }

    rj_geometry::Rect field_coordinates() const { return field_coords_; }

    rj_geometry::Point center_point() const { return center_point_; }

    [[nodiscard]] rj_geometry::Rect our_goal_zone_shape() const { return our_goal_zone_shape_; }
    [[nodiscard]] rj_geometry::Rect their_goal_zone_shape() const { return their_goal_zone_shape_; }

    /*
     * Provides a rect that is a padded version of their goalbox
     * used mostly for movement at the play level
     * exposed to python via constants.Field
     */
    rj_geometry::Rect their_goal_zone_shape_padded(float padding) {
        rj_geometry::Rect tmp = rj_geometry::Rect(their_goal_zone_shape_);
        tmp.pad(padding);
        return tmp;
    };

    rj_geometry::Segment our_goal_segment() const { return our_goal_segment_; }
    rj_geometry::Segment their_goal_segment() const { return their_goal_segment_; }
    rj_geometry::Rect our_half() const { return our_half_; }
    rj_geometry::Rect their_half() const { return their_half_; }
    rj_geometry::Rect field_rect() const { return field_rect_; }

    /*
     * Provides a rect that is a padded version of our goalbox
     * used mostly for movement at the play level
     * exposed to python via constants.Field
     */
    rj_geometry::Rect our_goal_zone_shape_padded(float padding) {
        rj_geometry::Rect tmp = rj_geometry::Rect(our_goal_zone_shape_);
        tmp.pad(padding);
        return tmp;
    };

    std::vector<rj_geometry::Line> field_borders() const { return field_borders_; }

    static const FieldDimensions kSingleFieldDimensions;

    static const FieldDimensions kDoubleFieldDimensions;

    static const FieldDimensions kDefaultDimensions;

    static FieldDimensions current_dimensions;

    FieldDimensions() : FieldDimensions(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0) {}

    /**
     * Parameterized constructor - creates the values and structs to be passed into the
     * FieldDimensions topic.
     */
    FieldDimensions(float length, float width, float border, float line_width, float goal_width,
                    float goal_depth, float goal_height, float penalty_short_dist,
                    float penalty_long_dist, float center_radius, float center_diameter,
                    float goal_flat, float floor_length, float floor_width)
        : length_(length),
          width_(width),
          border_(border),
          line_width_(line_width),
          goal_width_(goal_width),
          goal_depth_(goal_depth),
          goal_height_(goal_height),
          penalty_short_dist_(penalty_short_dist),
          penalty_long_dist_(penalty_long_dist),
          center_radius_(center_radius),
          center_diameter_(center_diameter),
          goal_flat_(goal_flat),
          floor_length_(floor_length),
          floor_width_(floor_width),
          penalty_x_right_coord_(penalty_long_dist / 2),
          penalty_x_left_coord_(-penalty_long_dist / 2),
          field_x_right_coord_(width / 2),
          field_x_left_coord_(-width / 2),
          floor_border_width_(width + 2 * border),
          floor_border_length_(length + 2 * border) {
        update_geometry();
    }

    /**
     * Returns a FieldDimensions struct but with the field size changed by a scalar factor.
     */
    FieldDimensions operator*(float scalar) const {
        return FieldDimensions(length_ * scalar, width_ * scalar, border_ * scalar,
                               line_width_ * scalar, goal_width_ * scalar, goal_depth_ * scalar,
                               goal_height_ * scalar, penalty_short_dist_ * scalar,
                               penalty_long_dist_ * scalar, center_radius_ * scalar,
                               center_diameter_ * scalar, goal_flat_ * scalar,
                               floor_length_ * scalar, floor_width_ * scalar);
    }

    /**
     * Returns a boolean value regarding whether 2 FieldDimensions structs are equal.
     */
    bool operator==(const FieldDimensions& a) const {
        return !(std::abs(length() - a.length()) > FLT_EPSILON ||
                 std::abs(width() - a.width()) > FLT_EPSILON ||
                 std::abs(border() - a.border()) > FLT_EPSILON ||
                 std::abs(line_width() - a.line_width()) > FLT_EPSILON ||
                 std::abs(goal_width() - a.goal_width()) > FLT_EPSILON ||
                 std::abs(goal_depth() - a.goal_depth()) > FLT_EPSILON ||
                 std::abs(goal_height() - a.goal_height()) > FLT_EPSILON ||
                 std::abs(penalty_short_dist() - a.penalty_short_dist()) > FLT_EPSILON ||
                 std::abs(penalty_long_dist() - a.penalty_long_dist()) > FLT_EPSILON ||
                 std::abs(center_radius() - a.center_radius()) > FLT_EPSILON ||
                 std::abs(center_diameter() - a.center_diameter()) > FLT_EPSILON ||
                 std::abs(goal_flat() - a.goal_flat()) > FLT_EPSILON ||
                 std::abs(floor_length() - a.floor_length()) > FLT_EPSILON ||
                 std::abs(floor_width() - a.floor_width()) > FLT_EPSILON);
    }

    bool operator!=(const FieldDimensions& a) const { return !(*this == a); }

    /**
     * Updates the basic geometric values and structs in this struct.
     */
    void update_geometry() {
        our_penalty_area_coordinates_ =
            rj_geometry::Rect(rj_geometry::Point(penalty_x_left_coord_, penalty_short_dist_),
                              rj_geometry::Point(penalty_x_right_coord_, 0.0));
        their_penalty_area_coordinates_ = rj_geometry::Rect(
            rj_geometry::Point(penalty_x_left_coord_, length_),
            rj_geometry::Point(penalty_x_right_coord_, (length_ - penalty_short_dist_)));
        field_coords_ = rj_geometry::Rect(rj_geometry::Point(field_x_left_coord_, length_),
                                          rj_geometry::Point(field_x_right_coord_, 0.0));

        our_goal_loc_ = rj_geometry::Point(0.0, 0.0);
        center_field_loc_ = rj_geometry::Point(0.0, length_ / 2);
        their_goal_loc_ = rj_geometry::Point(0.0, length_);

        our_left_goal_post_coordinate_ = rj_geometry::Point(-goal_width_ / 2, 0.0);
        our_right_goal_post_coordinate_ = rj_geometry::Point(goal_width_ / 2, 0.0);
        their_left_goal_post_coordinate_ = rj_geometry::Point(-goal_width_ / 2, length_);
        their_right_goal_post_coordinate_ = rj_geometry::Point(goal_width_ / 2, length_);

        our_left_corner_ = rj_geometry::Point(field_x_left_coord_, 0.0);
        our_right_corner_ = rj_geometry::Point(field_x_right_coord_, 0.0);
        their_left_corner_ = rj_geometry::Point(field_x_left_coord_, length_);
        their_right_corner_ = rj_geometry::Point(field_x_right_coord_, length_);

        center_point_ = rj_geometry::Point(0.0, length_ / 2.0);

        our_goal_zone_shape_ =
            rj_geometry::Rect(rj_geometry::Point(penalty_long_dist_ / 2, penalty_short_dist_),
                              rj_geometry::Point(-penalty_long_dist_ / 2, 0));

        their_goal_zone_shape_ = rj_geometry::Rect(
            rj_geometry::Point(-penalty_long_dist_ / 2, length_),
            rj_geometry::Point(penalty_long_dist_ / 2, length_ - penalty_short_dist_));

        their_goal_segment_ = rj_geometry::Segment(rj_geometry::Point(goal_width_ / 2.0, length_),
                                                   rj_geometry::Point(-goal_width_ / 2.0, length_));
        our_goal_segment_ = rj_geometry::Segment(rj_geometry::Point(goal_width_ / 2.0, 0),
                                                 rj_geometry::Point(-goal_width_ / 2.0, 0));

        their_half_ = rj_geometry::Rect(rj_geometry::Point(-width_ / 2, length_),
                                        rj_geometry::Point(width_ / 2, length_ / 2));
        our_half_ = rj_geometry::Rect(rj_geometry::Point(-width_ / 2, 0),
                                      rj_geometry::Point(width_ / 2, length_ / 2));

        field_rect_ = rj_geometry::Rect(rj_geometry::Point(-width_ / 2.0, 0),
                                        rj_geometry::Point(width_ / 2.0, length_));

        field_borders_ = {rj_geometry::Line(rj_geometry::Point(-width_ / 2.0, 0),
                                            rj_geometry::Point(-width_ / 2.0, length_)),
                          rj_geometry::Line(rj_geometry::Point(-width_ / 2.0, length_),
                                            rj_geometry::Point(width_ / 2.0, length_)),
                          rj_geometry::Line(rj_geometry::Point(width_ / 2.0, length_),
                                            rj_geometry::Point(width_ / 2.0, 0)),
                          rj_geometry::Line(rj_geometry::Point(width_ / 2.0, 0),
                                            rj_geometry::Point(-width_ / 2.0, 0))};
    }

private:
    float length_;
    float width_;
    float border_;
    float line_width_;
    float goal_width_;
    float goal_depth_;
    float goal_height_;
    float penalty_short_dist_;
    float penalty_long_dist_;
    float center_radius_;
    float center_diameter_;
    float goal_flat_;
    float floor_length_;
    float floor_width_;
    float penalty_x_right_coord_;
    float penalty_x_left_coord_;
    float field_x_right_coord_;
    float field_x_left_coord_;

    float floor_border_width_;
    float floor_border_length_;

    rj_geometry::Point our_goal_loc_;
    rj_geometry::Point center_field_loc_;
    rj_geometry::Point their_goal_loc_;

    rj_geometry::Rect our_penalty_area_coordinates_;
    rj_geometry::Rect their_penalty_area_coordinates_;
    rj_geometry::Point our_left_goal_post_coordinate_;
    rj_geometry::Point our_right_goal_post_coordinate_;
    rj_geometry::Point their_left_goal_post_coordinate_;
    rj_geometry::Point their_right_goal_post_coordinate_;

    rj_geometry::Point our_left_corner_;
    rj_geometry::Point our_right_corner_;
    rj_geometry::Point their_left_corner_;
    rj_geometry::Point their_right_corner_;
    rj_geometry::Rect field_coords_;

    rj_geometry::Point center_point_;
    rj_geometry::Rect our_goal_zone_shape_;
    rj_geometry::Rect their_goal_zone_shape_;
    rj_geometry::Segment our_goal_segment_;
    rj_geometry::Segment their_goal_segment_;
    rj_geometry::Rect our_half_;
    rj_geometry::Rect their_half_;
    rj_geometry::Rect field_rect_;

    std::vector<rj_geometry::Line> field_borders_;
};

namespace rj_convert {

template <>
struct RosConverter<FieldDimensions, FieldDimensions::Msg> {
    /**
     * Converts and returns the FieldDimensions struct into a suitable form for it to be
     * sent through the ROS2 network (a Msg).
     */
    static FieldDimensions::Msg to_ros(const FieldDimensions& from) {
        rj_msgs::msg::FieldDimensions field_message;

        convert_to_ros(from.length(), &field_message.length);
        convert_to_ros(from.width(), &field_message.width);
        convert_to_ros(from.border(), &field_message.border);

        convert_to_ros(from.line_width(), &field_message.line_width);

        convert_to_ros(from.goal_width(), &field_message.goal_width);
        convert_to_ros(from.goal_depth(), &field_message.goal_depth);
        convert_to_ros(from.goal_height(), &field_message.goal_height);

        convert_to_ros(from.penalty_short_dist(), &field_message.penalty_short_dist);
        convert_to_ros(from.penalty_long_dist(), &field_message.penalty_long_dist);

        convert_to_ros(from.center_radius(), &field_message.center_radius);
        convert_to_ros(from.center_diameter(), &field_message.center_diameter);

        convert_to_ros(from.goal_flat(), &field_message.goal_flat);

        convert_to_ros(from.floor_length(), &field_message.floor_length);
        convert_to_ros(from.floor_width(), &field_message.floor_width);

        return field_message;
    }

    /**
     * Converts and returns the FieldDimensions struct from msg form to the original struct
     * form.
     */
    static FieldDimensions from_ros(const FieldDimensions::Msg& from) {
        return FieldDimensions(
            from.length, from.width, from.border, from.line_width, from.goal_width, from.goal_depth,
            from.goal_height, from.penalty_short_dist, from.penalty_long_dist, from.center_radius,
            from.center_diameter, from.goal_flat, from.floor_length, from.floor_width);
    }
};

ASSOCIATE_CPP_ROS(FieldDimensions, FieldDimensions::Msg);

}  // namespace rj_convert
