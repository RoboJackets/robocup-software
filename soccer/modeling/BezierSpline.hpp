
// Takes a set of points describing a probably jagged path and applies a cubic
// bezier on top of it.  It outputs a vector of locations, curvatures, and
// directions evaluated at fixed distance steps along the path.
// @param spacing The arc length distance between each output value
// @param positionsOut Positions along the bezier evenly spaced @spacing distance apart
// @param directionsOut Unit vectors pointing in the direction of the bezier at each point in the @positionsOut vector
// @param curvaturesOut A vector of the bezier's curvature values
void BezierInterpolation(const std::vector<Geometry2d::Point>& points,
    const Geometry2d::Point& startVel,
    const Geometry2d::Point& endVel,
    float spacing,
    std::vector<Geometry2d::Point>* positionsOut,
    std::vector<Geometry2d::Point>* directionsOut,
    std::std::vector<float>* curvaturesOut);
