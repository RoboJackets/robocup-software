#pragma once

#include <iostream>

struct Field_Dimensions_t {
  const float &Field_Length;
  const float &Field_Width;
  const float &Field_Border;

  const float &Field_LineWidth;

  const float &Field_GoalWidth;
  const float &Field_GoalDepth;
  const float &Field_GoalHeight;

/** Distance of the penalty marker from the goal line */
  const float &Field_PenaltyDist;
  const float &Field_PenaltyDiam;

/** Radius of the goal arcs */
  const float &Field_ArcRadius;

/** diameter of the center circle */
  const float &Field_CenterRadius;
  const float &Field_CenterDiameter;

/** flat area for defence markings */
  const float &Field_GoalFlat;

  const float &Floor_Length;
  const float &Floor_Width;

  Field_Dimensions_t() : Field_Dimensions_t(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0)
  {
    std::cout << "Default const." << std::endl;
  }

  Field_Dimensions_t(float fl, float fw, float fb, float flw, float gw, float gd, float gh, float pd, float pdiam, float ar, float cr, float cd, float gf, float ffl, float ffw)
      : Field_Length(_Field_Length),
        Field_Width(_Field_Width),
        Field_Border(_Field_Border),
        Field_LineWidth(_Field_LineWidth),
        Field_GoalWidth(_Field_GoalWidth),
        Field_GoalDepth(_Field_GoalDepth),
        Field_GoalHeight(_Field_GoalHeight),
        Field_PenaltyDist(_Field_PenaltyDist),
        Field_PenaltyDiam(_Field_PenaltyDiam),
        Field_ArcRadius(_Field_ArcRadius),
        Field_CenterRadius(_Field_CenterRadius),
        Field_CenterDiameter(_Field_CenterDiameter),
        Field_GoalFlat(_Field_GoalFlat),
        Floor_Length(_Floor_Length),
        Floor_Width(_Floor_Width),
        _Field_Length(fl),
        _Field_Width(fw),
        _Field_Border(fb),
        _Field_LineWidth(flw),
        _Field_GoalWidth(gw),
        _Field_GoalDepth(gd),
        _Field_GoalHeight(gh),
        _Field_PenaltyDist(pd),
        _Field_PenaltyDiam(pdiam),
        _Field_ArcRadius(ar),
        _Field_CenterRadius(cr),
        _Field_CenterDiameter(cd),
        _Field_GoalFlat(gf),
        _Floor_Length(ffl),
        _Floor_Width(ffw)
  {
    std::cout << "Const.\t" << fl << std::endl;
  }

  Field_Dimensions_t(const Field_Dimensions_t& other)
      : Field_Dimensions_t( other._Field_Length,
                            other._Field_Width,
                            other._Field_Border,
                            other._Field_LineWidth,
                            other._Field_GoalWidth,
                            other._Field_GoalDepth,
                            other._Field_GoalHeight,
                            other._Field_PenaltyDist,
                            other._Field_PenaltyDiam,
                            other._Field_ArcRadius,
                            other._Field_CenterRadius,
                            other._Field_CenterDiameter,
                            other._Field_GoalFlat,
                            other._Floor_Length,
                            other._Floor_Width)
  {
    std::cout << "Copy const." << std::endl;
  }

  Field_Dimensions_t& operator=(const Field_Dimensions_t& other) {
    _Field_Length = other._Field_Length;
    _Field_Width = other._Field_Width;
    _Field_Border = other._Field_Border;
    _Field_LineWidth = other._Field_LineWidth;
    _Field_GoalWidth = other._Field_GoalWidth;
    _Field_GoalDepth = other._Field_GoalDepth;
    _Field_GoalHeight = other._Field_GoalHeight;
    _Field_PenaltyDist = other._Field_PenaltyDist;
    _Field_PenaltyDiam = other._Field_PenaltyDiam;
    _Field_ArcRadius = other._Field_ArcRadius;
    _Field_CenterRadius = other._Field_CenterRadius;
    _Field_CenterDiameter = other._Field_CenterDiameter;
    _Field_GoalFlat = other._Field_GoalFlat;
    _Floor_Length = other._Floor_Length;
    _Floor_Width = other._Floor_Width;
    return *this;
  }

private:
  float _Field_Length;
  float _Field_Width;
  float _Field_Border;
  float _Field_LineWidth;
  float _Field_GoalWidth;
  float _Field_GoalDepth;
  float _Field_GoalHeight;
  float _Field_PenaltyDist;
  float _Field_PenaltyDiam;
  float _Field_ArcRadius;
  float _Field_CenterRadius;
  float _Field_CenterDiameter;
  float _Field_GoalFlat;
  float _Floor_Length;
  float _Floor_Width;
};