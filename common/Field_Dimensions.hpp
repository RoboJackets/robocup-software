#pragma once

#include <iostream>

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
    inline float Length() const { return _Length; }
    inline float Width() const { return _Width; }
    inline float Border() const { return _Border; }

    inline float LineWidth() const { return _LineWidth; }

    inline float GoalWidth() const { return _GoalWidth; }
    inline float GoalDepth() const { return _GoalDepth; }
    inline float GoalHeight() const { return _GoalHeight; }

    /** Distance of the penalty marker from the goal line */
    inline float PenaltyDist() const { return _PenaltyDist; }
    inline float PenaltyDiam() const { return _PenaltyDiam; }

    /** Radius of the goal arcs */
    inline float ArcRadius() const { return _ArcRadius; }

    /** diameter of the center circle */
    inline float CenterRadius() const { return _CenterRadius; }
    inline float CenterDiameter() const { return _CenterDiameter; }

    /** flat area for defence markings */
    inline float GoalFlat() const { return _GoalFlat; }

    inline float FloorLength() const { return _FloorLength; }
    inline float FloorWidth() const { return _FloorWidth; }

    inline Geometry2d::Point CenterPoint() const { return _CenterPoint; }
    inline Geometry2d::CompositeShape OurGoalZoneShape() const {
        return _OurGoalZoneShape;
    }
    inline Geometry2d::CompositeShape TheirGoalShape() const {
        return _TheirGoalShape;
    }
    inline Geometry2d::Segment OurGoalSegment() const {
        return _OurGoalSegment;
    }
    inline Geometry2d::Segment TheirGoalSegment() const {
        return _TheirGoalSegment;
    }
    inline Geometry2d::Rect OurHalf() const { return _OurHalf; }
    inline Geometry2d::Rect TheirHalf() const { return _TheirHalf; }

    static const Field_Dimensions Single_Field_Dimensions;

    static const Field_Dimensions Double_Field_Dimensions;

    static Field_Dimensions Current_Dimensions;

    Field_Dimensions()
        : Field_Dimensions(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0) {
        updateGeometry();
    }

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

        _TheirGoalShape = Geometry2d::CompositeShape();
        _TheirGoalShape.add(std::make_shared<Geometry2d::Circle>(
            Geometry2d::Point(-_GoalFlat / 2.0, _Length), _ArcRadius));
        _TheirGoalShape.add(std::make_shared<Geometry2d::Circle>(
            Geometry2d::Point(_GoalFlat / 2.0, _Length), _ArcRadius));
        _TheirGoalShape.add(std::make_shared<Geometry2d::Rect>(
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
    Geometry2d::CompositeShape _TheirGoalShape;
    Geometry2d::Segment _OurGoalSegment;
    Geometry2d::Segment _TheirGoalSegment;
    Geometry2d::Rect _OurHalf;
    Geometry2d::Rect _TheirHalf;
};
