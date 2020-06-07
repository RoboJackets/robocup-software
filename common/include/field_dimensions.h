#pragma once

#include <geometry2d/arc.h>
#include <geometry2d/circle.h>
#include <geometry2d/composite_shape.h>
#include <geometry2d/line.h>
#include <geometry2d/point.h>
#include <geometry2d/polygon.h>
#include <geometry2d/rect.h>

#include <cfloat>
#include <cmath>
#include <iostream>
#include <rj_robocup/msg/field_dimensions.hpp>

using FieldDimensionsMsg = rj_robocup::msg::FieldDimensions;

/// This class contains constants defining the layout of the field.
/// See the official SSL rules page for a detailed diagram:
/// http://robocupssl.cpe.ku.ac.th/rules:main
struct FieldDimensions {
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

  geometry2d::Point CenterPoint() const { return _CenterPoint; }

  geometry2d::Rect OurGoalZoneShape() const { return _OurGoalZoneShape; }
  geometry2d::Rect TheirGoalZoneShape() const { return _TheirGoalZoneShape; }

  /*
   * Provides a rect that is a padded version of their goalbox
   * used mostly for movement at the play level
   * exposed to python via constants.Field
   */
  geometry2d::Rect TheirGoalZoneShapePadded(float padding) {
    geometry2d::Rect tmp = geometry2d::Rect(_TheirGoalZoneShape);
    tmp.pad(padding);
    return tmp;
  };

  geometry2d::Segment OurGoalSegment() const { return _OurGoalSegment; }
  geometry2d::Segment TheirGoalSegment() const { return _TheirGoalSegment; }
  geometry2d::Rect OurHalf() const { return _OurHalf; }
  geometry2d::Rect TheirHalf() const { return _TheirHalf; }
  geometry2d::Rect FieldRect() const { return _FieldRect; }

  /*
   * Provides a rect that is a padded version of our goalbox
   * used mostly for movement at the play level
   * exposed to python via constants.Field
   */
  geometry2d::Rect OurGoalZoneShapePadded(float padding) {
    geometry2d::Rect tmp = geometry2d::Rect(_OurGoalZoneShape);
    tmp.pad(padding);
    return tmp;
  };

  std::vector<geometry2d::Line> FieldBorders() const { return _FieldBorders; }

  static const FieldDimensions Single_FieldDimensions;

  static const FieldDimensions Double_FieldDimensions;

  static const FieldDimensions Default_Dimensions;

  static FieldDimensions Current_Dimensions;

  FieldDimensions()
      : FieldDimensions(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0) {}

  FieldDimensions(float fl, float fw, float fb, float flw, float gw, float gd,
                  float gh, float psd, float pld, float cr, float cd, float gf,
                  float ffl, float ffw)
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

  FieldDimensions operator*(float scalar) const {
    return FieldDimensions(_Length * scalar, _Width * scalar, _Border * scalar,
                           _LineWidth * scalar, _GoalWidth * scalar,
                           _GoalDepth * scalar, _GoalHeight * scalar,
                           _PenaltyShortDist * scalar,
                           _PenaltyLongDist * scalar, _CenterRadius * scalar,
                           _CenterDiameter * scalar, _GoalFlat * scalar,
                           _FloorLength * scalar, _FloorWidth * scalar);
  }

  bool operator==(const FieldDimensions& a) const {
    return !(std::abs(Length() - a.Length()) > FLT_EPSILON ||
             std::abs(Width() - a.Width()) > FLT_EPSILON ||
             std::abs(Border() - a.Border()) > FLT_EPSILON ||
             std::abs(LineWidth() - a.LineWidth()) > FLT_EPSILON ||
             std::abs(GoalWidth() - a.GoalWidth()) > FLT_EPSILON ||
             std::abs(GoalDepth() - a.GoalDepth()) > FLT_EPSILON ||
             std::abs(GoalHeight() - a.GoalHeight()) > FLT_EPSILON ||
             std::abs(PenaltyShortDist() - a.PenaltyShortDist()) >
                 FLT_EPSILON ||
             std::abs(PenaltyLongDist() - a.PenaltyLongDist()) > FLT_EPSILON ||
             std::abs(CenterRadius() - a.CenterRadius()) > FLT_EPSILON ||
             std::abs(CenterDiameter() - a.CenterDiameter()) > FLT_EPSILON ||
             std::abs(GoalFlat() - a.GoalFlat()) > FLT_EPSILON ||
             std::abs(FloorLength() - a.FloorLength()) > FLT_EPSILON ||
             std::abs(FloorWidth() - a.FloorWidth()) > FLT_EPSILON);
  }

  bool operator!=(const FieldDimensions& a) const { return !(*this == a); }

  void updateGeometry() {
    _CenterPoint = geometry2d::Point(0.0, _Length / 2.0);

    _OurGoalZoneShape = geometry2d::Rect(
        geometry2d::Point(_PenaltyLongDist / 2, _PenaltyShortDist),
        geometry2d::Point(-_PenaltyLongDist / 2, 0));

    _TheirGoalZoneShape = geometry2d::Rect(
        geometry2d::Point(-_PenaltyLongDist / 2, _Length),
        geometry2d::Point(_PenaltyLongDist / 2, _Length - _PenaltyShortDist));

    _TheirGoalSegment =
        geometry2d::Segment(geometry2d::Point(_GoalWidth / 2.0, _Length),
                            geometry2d::Point(-_GoalWidth / 2.0, _Length));
    _OurGoalSegment =
        geometry2d::Segment(geometry2d::Point(_GoalWidth / 2.0, 0),
                            geometry2d::Point(-_GoalWidth / 2.0, 0));

    _TheirHalf = geometry2d::Rect(geometry2d::Point(-_Width / 2, _Length),
                                  geometry2d::Point(_Width / 2, _Length / 2));
    _OurHalf = geometry2d::Rect(geometry2d::Point(-_Width / 2, 0),
                                geometry2d::Point(_Width / 2, _Length / 2));

    _FieldRect = geometry2d::Rect(geometry2d::Point(-_Width / 2.0, 0),
                                  geometry2d::Point(_Width / 2.0, _Length));

    _FieldBorders = {
        geometry2d::Line(geometry2d::Point(-_Width / 2.0, 0),
                         geometry2d::Point(-_Width / 2.0, _Length)),
        geometry2d::Line(geometry2d::Point(-_Width / 2.0, _Length),
                         geometry2d::Point(_Width / 2.0, _Length)),
        geometry2d::Line(geometry2d::Point(_Width / 2.0, _Length),
                         geometry2d::Point(_Width / 2.0, 0)),
        geometry2d::Line(geometry2d::Point(_Width / 2.0, 0),
                         geometry2d::Point(-_Width / 2.0, 0))};
  }

  [[nodiscard]] inline FieldDimensionsMsg toMsg() const {
    FieldDimensionsMsg msg{};

    msg.length = _Length;
    msg.width = _Width;
    msg.border = _Border;
    msg.line_width = _LineWidth;
    msg.goal_width = _GoalWidth;
    msg.goal_depth = _GoalDepth;
    msg.goal_height = _GoalHeight;
    msg.penalty_short_dist = _PenaltyShortDist;
    msg.penalty_long_dist = _PenaltyLongDist;
    msg.arc_radius = _ArcRadius;
    msg.center_radius = _CenterRadius;
    msg.center_diameter = _CenterDiameter;
    msg.goal_flat = _GoalFlat;
    msg.floor_length = _FloorLength;
    msg.floor_width = _FloorWidth;

    msg.center_point = _CenterPoint.toMsg();
    msg.our_goal_zone_shape = _OurGoalZoneShape.toMsg();
    msg.their_goal_zone_shape = _TheirGoalZoneShape.toMsg();
    msg.our_goal_segment = _OurGoalSegment.toMsg();
    msg.their_goal_segment = _TheirGoalSegment.toMsg();
    msg.our_half = _OurHalf.toMsg();
    msg.their_half = _TheirHalf.toMsg();
    msg.field_rect = _FieldRect.toMsg();

    msg.field_borders = {
        _FieldBorders[0].toMsg(),
        _FieldBorders[1].toMsg(),
        _FieldBorders[2].toMsg(),
        _FieldBorders[3].toMsg(),
    };

    return msg;
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

  geometry2d::Point _CenterPoint;
  geometry2d::Rect _OurGoalZoneShape;
  geometry2d::Rect _TheirGoalZoneShape;
  geometry2d::Segment _OurGoalSegment;
  geometry2d::Segment _TheirGoalSegment;
  geometry2d::Rect _OurHalf;
  geometry2d::Rect _TheirHalf;
  geometry2d::Rect _FieldRect;

  std::vector<geometry2d::Line> _FieldBorders;
};
