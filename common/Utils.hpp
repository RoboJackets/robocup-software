
#pragma once

#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <deque>
#include <vector>
#include <stdexcept>
#include <typeinfo>
#include <QString>
#include "Geometry2d/Point.hpp"
#include "Constants.hpp"

// Adjusts an angle in degrees to fit in [-180, 180].
static inline float fixAngleDegrees(float a)
{
	if (a < -180)
	{
		return a + 360;
	} else if (a > 180)
	{
		return a - 360;
	} else {
		return a;
	}
}

static inline float fixAngleRadians(float a)
{
	if (a < -M_PI)
	{
		return a + 2 * M_PI;
	} else if (a > M_PI)
	{
		return a - 2 * M_PI;
	} else {
		return a;
	}
}

/** Checks whether or not the given ball is in the defense area. */
static inline bool ballIsInGoalieBox(Geometry2d::Point point) {
	if (abs(point.x) < Field_GoalFlat / 2.0f) // Ball is in center (rectangular) portion of defensive bubble
	{
		if (point.y > 0 && point.y < Field_ArcRadius) {
			return true;
		} else {
			return false;
		}
	} else if (abs(point.x) < (Field_ArcRadius + Field_GoalFlat / 2.0f)) // Ball is in one of the side (arc) portions of defensive bubble
	{
		double adjusted_x = abs(point.x) - (Field_GoalFlat / 2.0f);
		double max_y = sqrt( (Field_ArcRadius * Field_ArcRadius) - (adjusted_x * adjusted_x));
		if (point.y > 0 && point.y <= max_y) {
			return true;
		} else {
			return false;
		}
	}
	return false;
}



static Geometry2d::Point fromOursToTheirs(Geometry2d::Point &pt) {
	Geometry2d::Point c;
	c.y = Field_Length - pt.y;
	c.x = -pt.x;

	return c;
}

static bool ballIsInTheirGoalieBox(Geometry2d::Point &pt) {
	Geometry2d::Point converted = fromOursToTheirs(pt);
	return ballIsInGoalieBox(converted);
}



/** Handles saturation of a bounded value */
template<class T>
float clamp(T value, T min, T max)
{
	if (value > max)
	{
		return max;
	}
	else if (value < min)
	{
		return min;
	}
	return value;
}

/** returns the local system timestamp */
static inline uint64_t timestamp()
{
	struct timeval time;
	gettimeofday(&time, 0);

	return (uint64_t)time.tv_sec * 1000000 + (uint64_t)time.tv_usec;
}

// Removes all entries in a std::map which associate to the given value.
template<class Map_Type, class Data_Type>
void map_remove(Map_Type &map, Data_Type &value)
{
	typename Map_Type::iterator i = map.begin();
	while (i != map.end())
	{
		typename Map_Type::iterator next = i;
		++next;

		if (i->second == value)
		{
			map.erase(i);
		}

		i = next;
	}
}

// If <key> exists in <map>, returns map[key].
// If not, returns 0.
template<typename Map>
typename Map::mapped_type map_lookup(const Map &map, typename Map::key_type key)
{
	typename Map::const_iterator i = map.find(key);
	if (i != map.end())
	{
		return i->second;
	} else {
		return 0;
	}
}

/**
 * A basic FIR filter of variable type with float coeffs
 *
 * Note, coefficients will be normalized so that inputs
 * will always be scaled appropriately
 */
template<typename T>
class FIRFilter {
public:
	typedef std::vector<float> Coeffs;

	FIRFilter(const T& zero, size_t nrTaps) : _zero(zero) {
		if (nrTaps == 0)
			throw std::invalid_argument("FIR Filter: number of taps must be greater than zero");

		// initialize
		_taps.assign(nrTaps, _zero);
		_coeffs.assign(nrTaps, 0.0f);
		_coeffs[0] = 1.0;
	}

	T filter(const T& x) {
		_taps.push_front(x);
		_taps.pop_back();

		T y = _zero;
		for (size_t i = 0; i<_taps.size(); ++i)
			y += _taps.at(i) * _coeffs.at(i);

		return y;
	}

	/** reinitializes the coeffs and taps to new values */
	void setCoeffs(const Coeffs& coeffs) {
		size_t nrTaps = coeffs.size();
		if (nrTaps == 0)
			throw std::invalid_argument("FIR Filter: number of coeffs must be greater than zero");

		_taps.assign(nrTaps, _zero);
		_coeffs.assign(nrTaps, 0.0);

		// find the normalizer
		float normalizer = 0.0;
		for (size_t i = 0; i<coeffs.size(); ++i)
			normalizer += coeffs.at(i);

		// set the normalized coefficients
		for (size_t i=0; i<coeffs.size(); ++i)
			_coeffs[i] = coeffs.at(i) / normalizer;
	}

protected:
	T _zero;
	std::deque<T> _taps;
	Coeffs _coeffs;
};

// An output iterator which throws an exception when it's used.
// This is used for example to determine if the output of set_difference would be non-empty without
// calculating all of it.
template<typename T>
class ExceptionIterator: public std::iterator<std::output_iterator_tag, void, void, void, void>
{
public:
	ExceptionIterator &operator=(const T &value)
	{
		throw std::exception();
	}

	ExceptionIterator &operator*()
	{
		return *this;
	}

	ExceptionIterator &operator++()
				{
		return *this;
				}

	ExceptionIterator &operator++(int)
				{
		return *this;
				}
};

// Sets str to the name of a class.
// Use it like this:
//		Object *obj = new Object();
//		QString name = typeName(typeid(*obj));
// The returned name is in the usual C++ format: "Namespace::Namespace::Class"
QString typeName(const std::type_info &info);

// Like typeName, but only returns the final class name part.
QString className(const std::type_info &info);
