#pragma once

#include "Entity.hpp"

#include <protobuf/RadioTx.pb.h>
#include <protobuf/RadioRx.pb.h>

class Ball;

class Robot : public Entity
{
	public:
		typedef enum {
			rev2010,
			rev2008
		} Rev;

		Robot(Env* env, unsigned int id, Robot::Rev rev);
		~Robot();
		
		/** @return the world angle */
		float getAngle() const;
		
		virtual void position(float x, float y); 
		
		/** set control data */
		void radioTx(const Packet::RadioTx::Robot *data);
		/** get robot information data */
		Packet::RadioRx radioRx() const;

		/** assigned shell number */
		unsigned int shell;

	private:
		/** Functions to initialize physical objects */
		void initRoller();
		void initKicker();
		void initWheels();
		bool ballSense(const Ball *ball) const;

		NxConvexMesh* cylinder(const float length, const float radius,
				const unsigned int sides);

	private:
		NxActor* _roller;
		NxActor* _kicker;
		
		NxRevoluteJoint* _rollerJoint;
		
		NxActor* _wheels[4];
		NxRevoluteJoint* _motors[4];
		NxD6Joint* _kickerJoint;
		
		Rev _rev;

		/** kicker charge status */
		uint64_t _lastKicked;
		const static uint64_t RechargeTime = 6000000; // six seconds

		/** center of roller from ground */
		const static float RollerHeight = .03;
		/** center of roller from center of robot */
		const static float RollerOffset = .065;
		/** roller length */
		const static float RollerLength = .07;
		/** radius of the roller */
		const static float RollerRadius = .01;

		/** width of the kicker face */
		const static float KickerFaceWidth = .05;
		/** height of the kicker face */
		const static float KickerFaceHeight = .005;
		/** depth of the kicker plate */
		const static float KickerLength = .03;
};
