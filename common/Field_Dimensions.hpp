#pragma once

#include <iostream>

struct Field_Dimensions {
  const float &Length;
  const float &Width;
  const float &Border;

  const float &LineWidth;

  const float &GoalWidth;
  const float &GoalDepth;
  const float &GoalHeight;

/** Distance of the penalty marker from the goal line */
  const float &PenaltyDist;
  const float &PenaltyDiam;

/** Radius of the goal arcs */
  const float &ArcRadius;

/** diameter of the center circle */
  const float &CenterRadius;
  const float &CenterDiameter;

/** flat area for defence markings */
  const float &GoalFlat;

  const float &FloorLength;
  const float &FloorWidth;

  static const Field_Dimensions Single_Field_Dimensions;

  static const Field_Dimensions Double_Field_Dimensions;

  static Field_Dimensions Current_Dimensions;

  Field_Dimensions() : Field_Dimensions(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0)
  {
  }

  Field_Dimensions(float fl, float fw, float fb, float flw, float gw, float gd, float gh, float pd, float pdiam, float ar, float cr, float cd, float gf, float ffl, float ffw)
      : Length(_Length),
        Width(_Width),
        Border(_Border),
        LineWidth(_LineWidth),
        GoalWidth(_GoalWidth),
        GoalDepth(_GoalDepth),
        GoalHeight(_GoalHeight),
        PenaltyDist(_PenaltyDist),
        PenaltyDiam(_PenaltyDiam),
        ArcRadius(_ArcRadius),
        CenterRadius(_CenterRadius),
        CenterDiameter(_CenterDiameter),
        GoalFlat(_GoalFlat),
        FloorLength(_FloorLength),
        FloorWidth(_FloorWidth),
        _Length(fl),
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
        _FloorWidth(ffw)
  {
  }

  Field_Dimensions(const Field_Dimensions & other)
      : Field_Dimensions( other._Length,
                            other._Width,
                            other._Border,
                            other._LineWidth,
                            other._GoalWidth,
                            other._GoalDepth,
                            other._GoalHeight,
                            other._PenaltyDist,
                            other._PenaltyDiam,
                            other._ArcRadius,
                            other._CenterRadius,
                            other._CenterDiameter,
                            other._GoalFlat,
                            other._FloorLength,
                            other._FloorWidth)
  {
  }

  Field_Dimensions & operator=(const Field_Dimensions & other) {
    _Length = other._Length;
    _Width = other._Width;
    _Border = other._Border;
    _LineWidth = other._LineWidth;
    _GoalWidth = other._GoalWidth;
    _GoalDepth = other._GoalDepth;
    _GoalHeight = other._GoalHeight;
    _PenaltyDist = other._PenaltyDist;
    _PenaltyDiam = other._PenaltyDiam;
    _ArcRadius = other._ArcRadius;
    _CenterRadius = other._CenterRadius;
    _CenterDiameter = other._CenterDiameter;
    _GoalFlat = other._GoalFlat;
    _FloorLength = other._FloorLength;
    _FloorWidth = other._FloorWidth;
    return *this;
  }

  Field_Dimensions operator*(float scalar) const {
    return Field_Dimensions(
        _Length*scalar,
        _Width*scalar,
        _Border*scalar,
        _LineWidth*scalar,
        _GoalWidth*scalar,
        _GoalDepth*scalar,
        _GoalHeight*scalar,
        _PenaltyDist*scalar,
        _PenaltyDiam*scalar,
        _ArcRadius*scalar,
        _CenterRadius*scalar,
        _CenterDiameter*scalar,
        _GoalFlat*scalar,
        _FloorLength*scalar,
        _FloorWidth*scalar
    );
  }

  float Get_Length() { return _Length; }
  float Get_Width() { return _Width; }
  float Get_Border() { return _Border; }
  float Get_LineWidth() { return _LineWidth; }
  float Get_GoalWidth() { return _GoalWidth; }
  float Get_GoalDepth() { return _GoalDepth; }
  float Get_GoalHeight() { return _GoalHeight; }
  float Get_PenaltyDist() { return _PenaltyDist; }
  float Get_PenaltyDiam() { return _PenaltyDiam; }
  float Get_ArcRadius() { return _ArcRadius; }
  float Get_CenterRadius() { return _CenterRadius; }
  float Get_CenterDiameter() { return _CenterDiameter; }
  float Get_GoalFlat() { return _GoalFlat; }
  float Get_FloorLength() { return _FloorLength; }
  float Get_FloorWidth() { return _FloorWidth; }

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
};