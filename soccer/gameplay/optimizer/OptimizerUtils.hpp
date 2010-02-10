/*
 * OptimizerUtils.hpp
 *
 *  Created on: Dec 10, 2009
 *      Author: alexgc
 */

#pragma once

#include <Vector.h>
#include <Point.hpp>
#include <sstream>

inline std::string genKey(int idx, const std::string& type) {
	std::ostringstream oss;
	oss << idx << "_" << type;
	return oss.str();
}

inline Geometry2d::Point vec2pt(const Vector& vec) {
	return Geometry2d::Point(vec(0), vec(1));
}

inline Vector pt2vec(const Geometry2d::Point& pt) {
	return gtsam::Vector_(2, pt.x, pt.y);
}

