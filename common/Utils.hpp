// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et
#pragma once

#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <stdexcept>

namespace Utils
{
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
	
	/** returns the vision timestamp */
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
}
