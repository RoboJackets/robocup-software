#pragma once

/**
 * Utilities for handling bezier curves
 */

#include <planning/BezierPath.hpp>

namespace Planning {

/** evaluates a Bezier curve */
Geometry2d::Point evaluateBezier(float t,
		const std::vector<Geometry2d::Point>& controls,
		const std::vector<float>& coeffs);

/** evaluates the derivative of a Bezier curve */
Geometry2d::Point
evaluateBezierVelocity(float t,
		const std::vector<Geometry2d::Point>& controls,
		const std::vector<float>& coeffs);

/** generate an interpolated bezier curve */
BezierPath createBezierPath(const std::vector<Geometry2d::Point>& controls);

/** determines the length of a Bezier curve */
float bezierLength(const std::vector<Geometry2d::Point>& controls,
		const std::vector<float>& coeffs);

/** calculates binomial coefficients */
int binomialCoefficient(int n, int k);
int factorial(int n);

} // \namespace planning
