#ifndef VISION_DATA_HPP 
#define VISION_DATA_HPP

#include "Ports.hpp"

#include <Geometry/Point2d.hpp>

#include <sys/time.h>
#include <stdint.h>

namespace Packet
{
	typedef struct VisionData
	{
		static const int Type = VisionPort;
			
		typedef struct Ball
		{
			Ball(float x=0, float y=0)
			{
				this->pos.x = x;
				this->pos.y = y;
				this->valid = false;
			}
			
			Geometry::Point2d pos;
			Geometry::Point2d vel;
			
			bool valid;
		}  __attribute__((__packed__)) Ball;
		
		typedef struct Robot
		{
			Robot(float x=0, float y=0, float theta=0) 
			{
				this->pos.x = x;
				this->pos.y = y;
				this->theta = theta;
				this->valid = false;
			}
			
			Geometry::Point2d pos;
			Geometry::Point2d vel;
			float theta;
			
			bool valid;
		}  __attribute__((__packed__)) Robot;
		
		VisionData() {}
		~VisionData() {};
		
		uint64_t timestamp;
		
		Ball ball;
		Robot self[5];
		Robot opp[5];
		
		void timeNow()
		{ 
			struct timeval time;
			gettimeofday(&time, 0); 
			
			timestamp = ((uint64_t)time.tv_sec)*1000000 + time.tv_usec;
		}
	} __attribute__((__packed__)) VisionData;
}

#endif
