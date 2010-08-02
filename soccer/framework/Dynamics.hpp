#pragma once

#include <framework/ConfigFile.hpp>

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
		
		public:
			Dynamics();
			
			/** calculate the possible dynamics for an angle @a angle 
			 *  assuming the particular angular velocity @a w
			 *  Note: @a angle should be in robot space */
			DynamicsInfo info(const float angle, const float w) const;
			
			/** return the ideal travel time for the length */
			float travelTime(const float length) const;
			
			void setConfig(ConfigFile::Robot::Motion cfg);
			
		private:
			ConfigFile::Robot::Motion::Dynamics _deg0;
			ConfigFile::Robot::Motion::Dynamics _deg45;
			
			ConfigFile::Robot::Motion::Dynamics _rotation;
	};
}
