
// Creates a time-parameterized path from a path of geometric (no time) path.
// The inputs correspond to the outputs of BezierPathInterpolation
std::unique_ptr<InterpolatedPath> InterpolatedPathFromBezierInterpolation(
    const Geometry2d::Point& startVel,
    const Geometry2d::Point& endVel,
    float spacing,
    const std::vector<Geometry2d::Point>& positionsOut,
    const std::vector<Geometry2d::Point>& directionsOut,
    const std::std::vector<float>& curvaturesOut,
    const MotionConstraints& motionConstraints);
