#include <stdexcept>

#include "bezier.hpp"

using namespace std;
using namespace Geometry2d;

Geometry2d::Point
Planning::evaluateBezier(float t,
		const std::vector<Geometry2d::Point>& controls,
		const std::vector<float>& coeffs) {

	size_t n = controls.size();
	float j = 1.0 - t;
	Point pt;
	for (size_t k = 0; k<n; ++k) {
		pt += controls.at(k) * pow(j, n-1-k) * pow(t, k) * coeffs.at(k);
	}
	return pt;
}

Geometry2d::Point
Planning::evaluateBezierVelocity(float t,
		const std::vector<Geometry2d::Point>& controls,
		const std::vector<float>& coeffs) {

	int n = controls.size();
	float j = 1.0 - t;
	Point pt;
	for (int k = 0; k<n; ++k) {
		pt += controls.at(k) * -1 * (n-1-k) * pow(j, n-1-k-1) * pow(t, k) * coeffs.at(k);
		pt += controls.at(k) * pow(j, n-1-k) * k * pow(t, k-1) * coeffs.at(k);
	}
	return pt;
}

Planning::Path createBezierPath(const std::vector<Geometry2d::Point>& controls) {
	Planning::Path interp;
	size_t degree = controls.size();

	// generate coefficients
	vector<float> coeffs;
	for (size_t i=0; i<degree; ++i) {
		coeffs.push_back(Planning::binomialCoefficient(degree-1, i));
	}

	size_t nrPoints = 20;
	float inc = 1.0/nrPoints;
	for (size_t t = 1; t<nrPoints-1; ++t)
		interp.points.push_back(Planning::evaluateBezier(t*inc, controls, coeffs));
	interp.points.push_back(controls.at(controls.size() -1));

	return interp;
}

float Planning::bezierLength(const std::vector<Geometry2d::Point>& controls,
					const std::vector<float>& coeffs) {
	// linear interpolation of points
	vector<Point> interp;
	interp.push_back(controls.at(0));
	size_t nrPoints = 20;
	float inc = 1.0/nrPoints;
	for (size_t t = 1; t<nrPoints-1; ++t)
		interp.push_back(Planning::evaluateBezier(t*inc, controls, coeffs));
	interp.push_back(controls.at(controls.size() -1));

	// find the distance
	float length = 0.0;
	for (size_t i = 1; i<interp.size(); ++i)
		length += interp.at(i).distTo(interp.at(i-1));

	return length;
}

int Planning::factorial(int n) {
	if ( n == 1) return 1;
	return n * Planning::factorial(n-1);
}

int Planning::binomialCoefficient(int n, int k) {
	if (k > n) throw invalid_argument("K greater than N in binomialCoefficient()!");
	if (k == n || k == 0 ) return 1;
	if (k == 1 || k == n-1) return n;

	return Planning::factorial(n)/(Planning::factorial(k)*Planning::factorial(n-k));
}
