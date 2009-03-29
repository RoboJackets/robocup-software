// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

/** list of common utility commands that are shared across several modules */
#ifndef __UTILS_HPP
#define __UTILS_HPP

#include <sys/time.h>
#include <stdint.h>

namespace Utils
{
	/** returns the vision timestamp */
	uint64_t timestamp()
	{
		struct timeval time;
		gettimeofday(&time, 0);
		
		return (uint64_t)time.tv_sec * 1000000 + (uint64_t)time.tv_usec;
	}
	
}

#endif //__UTILS_HPP
