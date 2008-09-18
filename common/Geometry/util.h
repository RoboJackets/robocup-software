#ifndef _UTIL_H_
#define _UTIL_H_

// Multiply by degrees to get radians
#define DEG_TO_RAD	(M_PI / 180.0)

template<typename T>
static inline T sign(T f)
{
	if (f < 0)
	{
		return -1;
	} else if (f > 0)
	{
		return 1;
	} else {
		return 0;
	}
}

#endif // _UTIL_H_
