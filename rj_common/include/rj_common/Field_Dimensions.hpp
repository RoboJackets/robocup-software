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
struct Field_Dimensions {
    using Msg = rj_msgs::msg::FieldDimensions;

    float Length() const { return _Length; }
    float Width() const { return _Width; }

    /** the distance from the edge of the field to the border line */
    float Border() const { return _Border; }

    /** The width of the border lines */
    float LineWidth() const { return _LineWidth; }

    float GoalWidth() const { return _GoalWidth; }
    float GoalDepth() const { return _GoalDepth; }
    float GoalHeight() const { return _GoalHeight; }

    /** Dimensions of the rectangular penalty zone */
    float PenaltyShortDist() const { return _PenaltyShortDist; }
    float PenaltyLongDist() const { return _PenaltyLongDist; }

    /** diameter of the center circle */
    float CenterRadius() const { return _CenterRadius; }
    float CenterDiameter() const { return _CenterDiameter; }

    /** flat area for defence markings */
    float GoalFlat() const { return _GoalFlat; }

    float FloorLength() const { return _FloorLength; }
    float FloorWidth() const { return _FloorWidth; }

    Geometry2d::Point CenterPoint() const { return _CenterPoint; }

    [[nodiscard]] Geometry2d::Rect OurGoalZoneShape() const {
        return _OurGoalZoneShape;
    }
    [[nodiscard]] Geometry2d::Rect TheirGoalZoneShape() const {
        return _TheirGoalZoneShape;
    }

    /*
     * Provides a rect that is a padded version of their goalbox
     * used mostly for movement at the play level
     * exposed to python via constants.Field
     */
    Geometry2d::Rect TheirGoalZoneShapePadded(float padding) {
        Geometry2d::Rect tmp = Geometry2d::Rect(_TheirGoalZoneShape);
        tmp.pad(padding);
        return tmp;
    };

    Geometry2d::Segment OurGoalSegment() const { return _OurGoalSegment; }
    Geometry2d::Segment TheirGoalSegment() const { return _TheirGoalSegment; }
    Geometry2d::Rect OurHalf() const { return _OurHalf; }
    Geometry2d::Rect TheirHalf() const { return _TheirHalf; }
    Geometry2d::Rect FieldRect() const { return _FieldRect; }

    /*
     * Provides a rect that is a padded version of our goalbox
     * used mostly for movement at the play level
     * exposed to python via constants.Field
     */
    Geometry2d::Rect OurGoalZoneShapePadded(float padding) {
        Geometry2d::Rect tmp = Geometry2d::Rect(_OurGoalZoneShape);
        tmp.pad(padding);
        return tmp;
    };

    std::vector<Geometry2d::Line> FieldBorders() const { return _FieldBorders; }

    static const Field_Dimensions Single_Field_Dimensions;

    static const Field_Dimensions Double_Field_Dimensions;

    static const Field_Dimensions Default_Dimensions;

    static Field_Dimensions Current_Dimensions;

    Field_Dimensions()
        : Field_Dimensions(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0) {}

    Field_Dimensions(float fl, float fw, float fb, float flw, float gw,
                     float gd, float gh, float psd, float pld, float cr,
                     float cd, float gf, float ffl, float ffw)
        : _Length(fl),
          _Width(fw),
          _Border(fb),
          _LineWidth(flw),
          _GoalWidth(gw),
          _GoalDepth(gd),
          _GoalHeight(gh),
          _PenaltyShortDist(psd),
          _PenaltyLongDist(pld),
          _CenterRadius(cr),
          _CenterDiameter(cd),
          _GoalFlat(gf),
          _FloorLength(ffl),
          _FloorWidth(ffw) {
        updateGeometry();
    }

    Field_Dimensions operator*(float scalar) const {
        return Field_Dimensions(
            _Length * scalar, _Width * scalar, _Border * scalar,
            _LineWidth * scalar, _GoalWidth * scalar, _GoalDepth * scalar,
            _GoalHeight * scalar, _PenaltyShortDist * scalar,
            _PenaltyLongDist * scalar, _CenterRadius * scalar,
            _CenterDiameter * scalar, _GoalFlat * scalar, _FloorLength * scalar,
            _FloorWidth * scalar);
    }

    bool operator==(const Field_Dimensions& a) const {
        return !(
            std::abs(Length() - a.Length()) > FLT_EPSILON ||
            std::abs(Width() - a.Width()) > FLT_EPSILON ||
            std::abs(Border() - a.Border()) > FLT_EPSILON ||
            std::abs(LineWidth() - a.LineWidth()) > FLT_EPSILON ||
            std::abs(GoalWidth() - a.GoalWidth()) > FLT_EPSILON ||
            std::abs(GoalDepth() - a.GoalDepth()) > FLT_EPSILON ||
            std::abs(GoalHeight() - a.GoalHeight()) > FLT_EPSILON ||
            std::abs(PenaltyShortDist() - a.PenaltyShortDist()) > FLT_EPSILON ||
            std::abs(PenaltyLongDist() - a.PenaltyLongDist()) > FLT_EPSILON ||
            std::abs(CenterRadius() - a.CenterRadius()) > FLT_EPSILON ||
            std::abs(CenterDiameter() - a.CenterDiameter()) > FLT_EPSILON ||
            std::abs(GoalFlat() - a.GoalFlat()) > FLT_EPSILON ||
            std::abs(FloorLength() - a.FloorLength()) > FLT_EPSILON ||
            std::abs(FloorWidth() - a.FloorWidth()) > FLT_EPSILON);
    }

    bool operator!=(const Field_Dimensions& a) const { return !(*this == a); }

    void updateGeometry() {
        _CenterPoint = Geometry2d::Point(0.0, _Length / 2.0);

        _OurGoalZoneShape = Geometry2d::Rect(
            Geometry2d::Point(_PenaltyLongDist / 2, _PenaltyShortDist),
            Geometry2d::Point(-_PenaltyLongDist / 2, 0));

        _TheirGoalZoneShape =
            Geometry2d::Rect(Geometry2d::Point(-_PenaltyLongDist / 2, _Length),
                             Geometry2d::Point(_PenaltyLongDist / 2,
                                               _Length - _PenaltyShortDist));

        _TheirGoalSegment =
            Geometry2d::Segment(Geometry2d::Point(_GoalWidth / 2.0, _Length),
                                Geometry2d::Point(-_GoalWidth / 2.0, _Length));
        _OurGoalSegment =
            Geometry2d::Segment(Geometry2d::Point(_GoalWidth / 2.0, 0),
                                Geometry2d::Point(-_GoalWidth / 2.0, 0));

        _TheirHalf =
            Geometry2d::Rect(Geometry2d::Point(-_Width / 2, _Length),
                             Geometry2d::Point(_Width / 2, _Length / 2));
        _OurHalf = Geometry2d::Rect(Geometry2d::Point(-_Width / 2, 0),
                                    Geometry2d::Point(_Width / 2, _Length / 2));

        _FieldRect = Geometry2d::Rect(Geometry2d::Point(-_Width / 2.0, 0),
                                      Geometry2d::Point(_Width / 2.0, _Length));

        _FieldBorders = {
            Geometry2d::Line(Geometry2d::Point(-_Width / 2.0, 0),
                             Geometry2d::Point(-_Width / 2.0, _Length)),
            Geometry2d::Line(Geometry2d::Point(-_Width / 2.0, _Length),
                             Geometry2d::Point(_Width / 2.0, _Length)),
            Geometry2d::Line(Geometry2d::Point(_Width / 2.0, _Length),
                             Geometry2d::Point(_Width / 2.0, 0)),
            Geometry2d::Line(Geometry2d::Point(_Width / 2.0, 0),
                             Geometry2d::Point(-_Width / 2.0, 0))};
    }

    friend std::ostream& operator<<(std::ostream& stream,
                                    const Field_Dimensions& fd) {
        stream << "length: " << fd.Length() << "\n";
        stream << "width: " << fd.Width() << "\n";
        stream << "border: " << fd.Border() << "\n";
        stream << "line_width: " << fd.LineWidth() << "\n";
        stream << "goal_width: " << fd.GoalWidth() << "\n";
        stream << "goal_depth: " << fd.GoalDepth() << "\n";
        stream << "goal_height: " << fd.GoalHeight() << "\n";
        stream << "penalty_short_dist: " << fd.PenaltyShortDist() << "\n";
        stream << "penalty_long_dist: " << fd.PenaltyLongDist() << "\n";
        stream << "center_radius: " << fd.CenterRadius() << "\n";
        stream << "center_diameter: " << fd.CenterDiameter() << "\n";
        stream << "goal_flat: " << fd.GoalFlat() << "\n";
        stream << "floor_length: " << fd.FloorLength() << "\n";
        stream << "floor_width: " << fd.FloorWidth();
        return stream;
    }

private:
    float _Length;
    float _Width;
    float _Border;
    float _LineWidth;
    float _GoalWidth;
    float _GoalDepth;
    float _GoalHeight;
    float _PenaltyShortDist;
    float _PenaltyLongDist;
    float _ArcRadius;
    float _CenterRadius;
    float _CenterDiameter;
    float _GoalFlat;
    float _FloorLength;
    float _FloorWidth;

    Geometry2d::Point _CenterPoint;
    Geometry2d::Rect _OurGoalZoneShape;
    Geometry2d::Rect _TheirGoalZoneShape;
    Geometry2d::Segment _OurGoalSegment;
    Geometry2d::Segment _TheirGoalSegment;
    Geometry2d::Rect _OurHalf;
    Geometry2d::Rect _TheirHalf;
    Geometry2d::Rect _FieldRect;

    std::vector<Geometry2d::Line> _FieldBorders;
};

namespace rj_convert {

template <>
struct RosConverter<Field_Dimensions, Field_Dimensions::Msg> {
    static Field_Dimensions::Msg to_ros(const Field_Dimensions& from) {
        return rj_msgs::build<Field_Dimensions::Msg>()
            .length(from.Length())
            .width(from.Width())
            .border(from.Border())
            .line_width(from.LineWidth())
            .goal_width(from.GoalWidth())
            .goal_depth(from.GoalDepth())
            .goal_height(from.GoalHeight())
            .penalty_short_dist(from.PenaltyShortDist())
            .penalty_long_dist(from.PenaltyLongDist())
            .center_radius(from.CenterRadius())
            .center_diameter(from.CenterDiameter())
            .goal_flat(from.GoalFlat())
            .floor_length(from.FloorLength())
            .floor_width(from.FloorWidth());
    }

    static Field_Dimensions from_ros(const Field_Dimensions::Msg& from) {
        return Field_Dimensions(
            from.length, from.width, from.border, from.line_width,
            from.goal_width, from.goal_depth, from.goal_height,
            from.penalty_short_dist, from.penalty_long_dist, from.center_radius,
            from.center_diameter, from.goal_flat, from.floor_length,
            from.floor_width);
    }
};

}  // namespace rj_convert