#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include "Entity.hpp"

namespace Physics
{
	class Robot : public Entity
	{
		private:
			typedef enum
			{
				Ready,
				Kicking,
				Recharging
			} KickState;
		
		public:
			Robot(dWorldID world, dSpaceID space, unsigned int id);
			~Robot();

			void paint();

			/** return the angle of the robot +/-180 in world space */
			float theta() const;

			/** Set the movement for the robot */
			void motors(int8_t m1, int8_t m2, int8_t m3, int8_t m4);

			virtual void internal();

			/** set kick stength for next kick */
			void kick(uint8_t k) { _kick = k; }
			bool charged() const { return _charged; }
			
			void roller(int8_t roller) { _roller = roller; }
			
			bool ballPresent() const { return _ballPresent; }
			void ballPresent(bool b) { _ballPresent = b; }

			virtual void setPosition(float x=0, float y=0, float z=0);
			void setTheta(float t);

			unsigned int id() const { return _id; }

		private:
			unsigned int _id;

			dGeomID _rollerGeom;
			dGeomID _rollerGeomID;
			dBodyID _rollerBodyID;
			dMass _rollerMass;

			dGeomID _kickerGeomID;
			dBodyID _kickerBodyID;

			dJointID joint;
			dJointID jointM;
			dJointID jointK;

			/** robot will kick once if true, will not reset */
			uint8_t _kick;
			KickState _kickState;
			unsigned int _kickCount;
			bool _charged;
			
			/** the robot has the ball when this is true */
			bool _ballPresent;
			
			int8_t _roller;

			/** speed of the 4 motors */
			int8_t _motors[4];

            GLUquadric *_quadric;

			/// robot parameters ///
			static const float RollerLength;
			static const float RollerDiameter;
			static const float RollerHeight;
			static const float RollerDist;

			static const float WheelDiameter;
			static const float WheelThickness;

			static const float KickerLength;
			static const float KickerWidth;
			static const float KickerThickness;
			static const float KickerTravel;
			static const float KickerDist;
			static const float KickerHeight;
	};
}

#endif /*ROBOT_HPP_*/
