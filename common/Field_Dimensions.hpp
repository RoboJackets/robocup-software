#pragma once

#include <iostream>
#include <cfloat>
#include <cmath>

#include <Geometry2d/Arc.hpp>
#include <Geometry2d/Circle.hpp>
#include <Geometry2d/CompositeShape.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Polygon.hpp>
#include <Geometry2d/Rect.hpp>

/// This class contains constants defining the layout of the field.
/// See the official SSL rules page for a detailed diagram:
/// http://robocupssl.cpe.ku.ac.th/rules:main
struct Field_Dimensions {
    float Length() const { return _Length; }
    float Width() const { return _Width; }

    /** the distance from the edge of the field to the border line */
    float Border() const { return _Border; }

    /** The width of the border lines */
    float LineWidth() const { return _LineWidth; }

    float GoalWidth() const { return _GoalWidth; }
    float GoalDepth() const { return _GoalDepth; }
    float GoalHeight() const { return _GoalHeight; }

    /** Distance of the penalty marker from the goal line */
    float PenaltyDist() const { return _PenaltyDist; }
    float PenaltyDiam() const { return _PenaltyDiam; }

    /** Radius of the goal arcs */
    float ArcRadius() const { return _ArcRadius; }

    /** diameter of the center circle */
    float CenterRadius() const { return _CenterRadius; }
    float CenterDiameter() const { return _CenterDiameter; }

    /** flat area for defence markings */
    float GoalFlat() const { return _GoalFlat; }

    float FloorLength() const { return _FloorLength; }
    float FloorWidth() const { return _FloorWidth; }

    Geometry2d::Point CenterPoint() const { return _CenterPoint; }
    Geometry2d::CompositeShape OurGoalZoneShape() const {
        return _OurGoalZoneShape;
    }
    Geometry2d::CompositeShape TheirGoalZoneShape() const {
        return _TheirGoalZoneShape;
    }
    Geometry2d::Segment OurGoalSegment() const { return _OurGoalSegment; }
    Geometry2d::Segment TheirGoalSegment() const { return _TheirGoalSegment; }
    Geometry2d::Rect OurHalf() const { return _OurHalf; }
    Geometry2d::Rect TheirHalf() const { return _TheirHalf; }
    Geometry2d::Rect FieldRect() const { return _FieldRect; }

    std::vector<Geometry2d::Line> FieldBorders() const { return _FieldBorders; }

    static const Field_Dimensions Single_Field_Dimensions;

    static const Field_Dimensions Double_Field_Dimensions;

    static const Field_Dimensions Default_Dimensions;

    static Field_Dimensions Current_Dimensions;

    Field_Dimensions()
        : Field_Dimensions(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0) {}

    Field_Dimensions(float fl, float fw, float fb, float flw, float gw,
                     float gd, float gh, float pd, float pdiam, float ar,
                     float cr, float cd, float gf, float ffl, float ffw)
        : _Length(fl),
          _Width(fw),
          _Border(fb),
          _LineWidth(flw),
          _GoalWidth(gw),
          _GoalDepth(gd),
          _GoalHeight(gh),
          _PenaltyDist(pd),
          _PenaltyDiam(pdiam),
          _ArcRadius(ar),
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
            _GoalHeight * scalar, _PenaltyDist * scalar, _PenaltyDiam * scalar,
            _ArcRadius * scalar, _CenterRadius * scalar,
            _CenterDiameter * scalar, _GoalFlat * scalar, _FloorLength * scalar,
            _FloorWidth * scalar);
    }

    bool operator==(const Field_Dimensions& a) const {
        return !(std::abs(Length() - a.Length()) > FLT_EPSILON ||
                 std::abs(Width() - a.Width()) > FLT_EPSILON ||
                 std::abs(Border() - a.Border()) > FLT_EPSILON ||
                 std::abs(LineWidth() - a.LineWidth()) > FLT_EPSILON ||
                 std::abs(GoalWidth() - a.GoalWidth()) > FLT_EPSILON ||
                 std::abs(GoalDepth() - a.GoalDepth()) > FLT_EPSILON ||
                 std::abs(GoalHeight() - a.GoalHeight()) > FLT_EPSILON ||
                 std::abs(PenaltyDist() - a.PenaltyDist()) > FLT_EPSILON ||
                 std::abs(PenaltyDiam() - a.PenaltyDiam()) > FLT_EPSILON ||
                 std::abs(ArcRadius() - a.ArcRadius()) > FLT_EPSILON ||
                 std::abs(CenterRadius() - a.CenterRadius()) > FLT_EPSILON ||
                 std::abs(CenterDiameter() - a.CenterDiameter()) >
                     FLT_EPSILON ||
                 std::abs(GoalFlat() - a.GoalFlat()) > FLT_EPSILON ||
                 std::abs(FloorLength() - a.FloorLength()) > FLT_EPSILON ||
                 std::abs(FloorWidth() - a.FloorWidth()) > FLT_EPSILON);
    }

    bool operator!=(const Field_Dimensions& a) const { return !(*this == a); }

    void updateGeometry() {
        _CenterPoint = Geometry2d::Point(0.0, _Length / 2.0);

        _OurGoalZoneShape = Geometry2d::CompositeShape();
        _OurGoalZoneShape.add(std::make_shared<Geometry2d::Circle>(
            Geometry2d::Point(-_GoalFlat / 2.0, 0), _ArcRadius));
        _OurGoalZoneShape.add(std::make_shared<Geometry2d::Circle>(
            Geometry2d::Point(_GoalFlat / 2.0, 0), _ArcRadius));
        _OurGoalZoneShape.add(std::make_shared<Geometry2d::Rect>(
            Geometry2d::Point(-_GoalFlat / 2.0, _ArcRadius),
            Geometry2d::Point(_GoalFlat / 2.0, 0)));

        _TheirGoalZoneShape = Geometry2d::CompositeShape();
        _TheirGoalZoneShape.add(std::make_shared<Geometry2d::Circle>(
            Geometry2d::Point(-_GoalFlat / 2.0, _Length), _ArcRadius));
        _TheirGoalZoneShape.add(std::make_shared<Geometry2d::Circle>(
            Geometry2d::Point(_GoalFlat / 2.0, _Length), _ArcRadius));
        _TheirGoalZoneShape.add(std::make_shared<Geometry2d::Rect>(
            Geometry2d::Point(-_GoalFlat / 2.0, _Length),
            Geometry2d::Point(_GoalFlat / 2.0, _Length - _ArcRadius)));

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

private:
    float _Length;
    float _Width;
    float _Border;
    float _LineWidth;
    float _GoalWidth;
    float _GoalDepth;
    float _GoalHeight;
    float _PenaltyDist;
    float _PenaltyDiam;
    float _ArcRadius;
    float _CenterRadius;
    float _CenterDiameter;
    float _GoalFlat;
    float _FloorLength;
    float _FloorWidth;

    Geometry2d::Point _CenterPoint;
    Geometry2d::CompositeShape _OurGoalZoneShape;
    Geometry2d::CompositeShape _TheirGoalZoneShape;
    Geometry2d::Segment _OurGoalSegment;
    Geometry2d::Segment _TheirGoalSegment;
    Geometry2d::Rect _OurHalf;
    Geometry2d::Rect _TheirHalf;
    Geometry2d::Rect _FieldRect;

    std::vector<Geometry2d::Line> _FieldBorders;
};
