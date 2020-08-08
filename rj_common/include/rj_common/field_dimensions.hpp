#pragma once

#include <Geometry2d/Arc.hpp>
#include <Geometry2d/Circle.hpp>
#include <Geometry2d/CompositeShape.hpp>
#include <Geometry2d/Line.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Polygon.hpp>
#include <Geometry2d/Rect.hpp>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <rj_convert/ros_convert.hpp>
#include <rj_msgs/msg/field_dimensions.hpp>

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

    Geometry2d::Point center_point() const { return center_point_; }

    [[nodiscard]] Geometry2d::Rect our_goal_zone_shape() const {
        return our_goal_zone_shape_;
    }
    [[nodiscard]] Geometry2d::Rect their_goal_zone_shape() const {
        return their_goal_zone_shape_;
    }

    /*
     * Provides a rect that is a padded version of their goalbox
     * used mostly for movement at the play level
     * exposed to python via constants.Field
     */
    Geometry2d::Rect their_goal_zone_shape_padded(float padding) {
        Geometry2d::Rect tmp = Geometry2d::Rect(their_goal_zone_shape_);
        tmp.pad(padding);
        return tmp;
    };

    Geometry2d::Segment our_goal_segment() const { return our_goal_segment_; }
    Geometry2d::Segment their_goal_segment() const { return their_goal_segment_; }
    Geometry2d::Rect our_half() const { return our_half_; }
    Geometry2d::Rect their_half() const { return their_half_; }
    Geometry2d::Rect field_rect() const { return field_rect_; }

    /*
     * Provides a rect that is a padded version of our goalbox
     * used mostly for movement at the play level
     * exposed to python via constants.Field
     */
    Geometry2d::Rect our_goal_zone_shape_padded(float padding) {
        Geometry2d::Rect tmp = Geometry2d::Rect(our_goal_zone_shape_);
        tmp.pad(padding);
        return tmp;
    };

    std::vector<Geometry2d::Line> field_borders() const { return field_borders_; }

    static const FieldDimensions kSingleFieldDimensions;

    static const FieldDimensions kDoubleFieldDimensions;

    static const FieldDimensions kDefaultDimensions;

    static FieldDimensions current_dimensions;

    FieldDimensions()
        : FieldDimensions(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0) {}

    FieldDimensions(float fl, float fw, float fb, float flw, float gw,
                     float gd, float gh, float psd, float pld, float cr,
                     float cd, float gf, float ffl, float ffw)
        : length_(fl),
          width_(fw),
          border_(fb),
          line_width_(flw),
          goal_width_(gw),
          goal_depth_(gd),
          goal_height_(gh),
          penalty_short_dist_(psd),
          penalty_long_dist_(pld),
          center_radius_(cr),
          center_diameter_(cd),
          goal_flat_(gf),
          floor_length_(ffl),
          floor_width_(ffw) {
        update_geometry();
    }

    FieldDimensions operator*(float scalar) const {
        return FieldDimensions(
            length_ * scalar, width_ * scalar, border_ * scalar,
            line_width_ * scalar, goal_width_ * scalar, goal_depth_ * scalar,
            goal_height_ * scalar, penalty_short_dist_ * scalar,
            penalty_long_dist_ * scalar, center_radius_ * scalar,
            center_diameter_ * scalar, goal_flat_ * scalar, floor_length_ * scalar,
            floor_width_ * scalar);
    }

    bool operator==(const FieldDimensions& a) const {
        return !(
            std::abs(length() - a.length()) > FLT_EPSILON ||
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

    void update_geometry() {
        center_point_ = Geometry2d::Point(0.0, length_ / 2.0);

        our_goal_zone_shape_ = Geometry2d::Rect(
            Geometry2d::Point(penalty_long_dist_ / 2, penalty_short_dist_),
            Geometry2d::Point(-penalty_long_dist_ / 2, 0));

        their_goal_zone_shape_ =
            Geometry2d::Rect(Geometry2d::Point(-penalty_long_dist_ / 2, length_),
                             Geometry2d::Point(penalty_long_dist_ / 2,
                                               length_ - penalty_short_dist_));

        their_goal_segment_ =
            Geometry2d::Segment(Geometry2d::Point(goal_width_ / 2.0, length_),
                                Geometry2d::Point(-goal_width_ / 2.0, length_));
        our_goal_segment_ =
            Geometry2d::Segment(Geometry2d::Point(goal_width_ / 2.0, 0),
                                Geometry2d::Point(-goal_width_ / 2.0, 0));

        their_half_ =
            Geometry2d::Rect(Geometry2d::Point(-width_ / 2, length_),
                             Geometry2d::Point(width_ / 2, length_ / 2));
        our_half_ = Geometry2d::Rect(Geometry2d::Point(-width_ / 2, 0),
                                    Geometry2d::Point(width_ / 2, length_ / 2));

        field_rect_ = Geometry2d::Rect(Geometry2d::Point(-width_ / 2.0, 0),
                                      Geometry2d::Point(width_ / 2.0, length_));

        field_borders_ = {
            Geometry2d::Line(Geometry2d::Point(-width_ / 2.0, 0),
                             Geometry2d::Point(-width_ / 2.0, length_)),
            Geometry2d::Line(Geometry2d::Point(-width_ / 2.0, length_),
                             Geometry2d::Point(width_ / 2.0, length_)),
            Geometry2d::Line(Geometry2d::Point(width_ / 2.0, length_),
                             Geometry2d::Point(width_ / 2.0, 0)),
            Geometry2d::Line(Geometry2d::Point(width_ / 2.0, 0),
                             Geometry2d::Point(-width_ / 2.0, 0))};
    }

    friend std::ostream& operator<<(std::ostream& stream,
                                    const FieldDimensions& fd) {
        stream << "length: " << fd.length() << "\n";
        stream << "width: " << fd.width() << "\n";
        stream << "border: " << fd.border() << "\n";
        stream << "line_width: " << fd.line_width() << "\n";
        stream << "goal_width: " << fd.goal_width() << "\n";
        stream << "goal_depth: " << fd.goal_depth() << "\n";
        stream << "goal_height: " << fd.goal_height() << "\n";
        stream << "penalty_short_dist: " << fd.penalty_short_dist() << "\n";
        stream << "penalty_long_dist: " << fd.penalty_long_dist() << "\n";
        stream << "center_radius: " << fd.center_radius() << "\n";
        stream << "center_diameter: " << fd.center_diameter() << "\n";
        stream << "goal_flat: " << fd.goal_flat() << "\n";
        stream << "floor_length: " << fd.floor_length() << "\n";
        stream << "floor_width: " << fd.floor_width();
        return stream;
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
    float arc_radius_;
    float center_radius_;
    float center_diameter_;
    float goal_flat_;
    float floor_length_;
    float floor_width_;

    Geometry2d::Point center_point_;
    Geometry2d::Rect our_goal_zone_shape_;
    Geometry2d::Rect their_goal_zone_shape_;
    Geometry2d::Segment our_goal_segment_;
    Geometry2d::Segment their_goal_segment_;
    Geometry2d::Rect our_half_;
    Geometry2d::Rect their_half_;
    Geometry2d::Rect field_rect_;

    std::vector<Geometry2d::Line> field_borders_;
};

namespace rj_convert {

template <>
struct RosConverter<FieldDimensions, FieldDimensions::Msg> {
    static FieldDimensions::Msg to_ros(const FieldDimensions& from) {
        return rj_msgs::build<FieldDimensions::Msg>()
            .length(from.length())
            .width(from.width())
            .border(from.border())
            .line_width(from.line_width())
            .goal_width(from.goal_width())
            .goal_depth(from.goal_depth())
            .goal_height(from.goal_height())
            .penalty_short_dist(from.penalty_short_dist())
            .penalty_long_dist(from.penalty_long_dist())
            .center_radius(from.center_radius())
            .center_diameter(from.center_diameter())
            .goal_flat(from.goal_flat())
            .floor_length(from.floor_length())
            .floor_width(from.floor_width());
    }

    static FieldDimensions from_ros(const FieldDimensions::Msg& from) {
        return FieldDimensions(
            from.length, from.width, from.border, from.line_width,
            from.goal_width, from.goal_depth, from.goal_height,
            from.penalty_short_dist, from.penalty_long_dist, from.center_radius,
            from.center_diameter, from.goal_flat, from.floor_length,
            from.floor_width);
    }
};

ASSOCIATE_CPP_ROS(FieldDimensions, FieldDimensions::Msg);

}  // namespace rj_convert