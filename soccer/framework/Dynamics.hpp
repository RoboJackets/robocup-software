#pragma once

#include <framework/SystemState.hpp>

#include <boost/shared_ptr.hpp>

class OurRobot;

//FIXME - This class is just conversion and calculation.  Put it with the data.

namespace Planning
{
	class Dynamics
	{
		public:      
			typedef struct DynamicsInfo
			{
				DynamicsInfo()
				{
					velocity = 0;
					acceleration = 0;
					deceleration = 0;
				}
				
				float velocity;
				float acceleration;
				float deceleration;
			} DynamicsInfo;
		
			Dynamics(OurRobot *robot);
			
			/** calculate the possible dynamics for an angle @a angle 
			 *  assuming the particular angular velocity @a w
			 *  Note: @a angle should be in robot space */
			DynamicsInfo info(const float angle, const float w) const;
			
			/** return the ideal travel time for the length */
			float travelTime(const float length) const;
			
		private:
			OurRobot* _self;
	};
}
