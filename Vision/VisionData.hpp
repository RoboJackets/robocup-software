#ifndef VISIONDATA_HPP_
#define VISIONDATA_HPP_

#include <vector>
#include <stdint.h>

class VisionData
{
	public:
		class Point
		{
			public:
				Point() : x(0), y(0) {}
				
				float x;
				float y;
		};
		
		class Robot
		{
			public:
				Robot() : x(0), y(0), theta(0) {}
				
				/** global position */
				Point pos;
				
				/** +- 180 degrees */
				float theta;
				
				/** shell id */
				uint8_t id;
		};
		
		class Ball
		{
			public:
				Ball() : x(0), y(0) {}
				
				/** global position */
				Point pos;
		};
		
		VisionData();
		
	private:
		
		std::vector<Ball> balls;
		
		std::vector<Robot> blue;
		std::vector<Robot> yellow;
		
		/** microseconds */
		uint64_t timestamp;
};

#endif /* VISIONDATA_HPP_ */
