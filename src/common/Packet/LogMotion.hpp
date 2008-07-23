#ifndef LOG_MOTION_HPP
#define LOG_MOTION_HPP

#include "Ports.hpp"
#include <Geometry/Point2d.hpp>

namespace Packet
{
	typedef struct
	{
		static const int Type = LogMotionPort;
		
		typedef struct Robot
		{
            Robot() : distRemaining(0), valid(false) {}
			
			Geometry::Point2d currPos;
			Geometry::Point2d destPos;
			
			/** path planner direction */
			Geometry::Point2d pdir;
			/** path planner distance left */
			float distRemaining;
			
			bool valid;
		} __attribute__((__packed__)) Robot;
		
		/** vision time */
		uint64_t timestamp;
		
		Robot robots[5];
		
	} __attribute__((__packed__)) LogMotion;
}

#endif /* LOG_MOTION_HPP */
