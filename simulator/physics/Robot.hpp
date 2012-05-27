#pragma once

#include "Entity.hpp"

#include <protobuf/RadioTx.pb.h>
#include <protobuf/RadioRx.pb.h>


class Ball;

class Robot : public Entity
{
public:
	typedef enum {
		rev2011,
		rev2008
	} RobotRevision;

	Robot(Environment* env, unsigned int id, Robot::RobotRevision rev, const Geometry2d::Point& startPos);
	virtual ~Robot();

	/** @return the world angle */
	virtual float getAngle() const;

	// Entity interface
	virtual void position(float x, float y); // world coords
	virtual void velocity(float x, float y, float w); // body coords
	virtual Geometry2d::Point getPosition() const;

	/** set control data */
	void radioTx(const Packet::RadioTx::Robot *data);

	/** get robot information data */
	Packet::RadioRx radioRx() const;

	/** assigned shell number */
	unsigned int shell;

	int visibility;
	bool ballSensorWorks;
	bool chargerWorks;

	const RobotRevision& revision() const { return _rev; }

public:

	/** Functions to initialize physical objects */
	virtual void initPhysics(const bool& blue);
	virtual void initRoller();
	virtual void initKicker();
	virtual void initWheels();
	bool ballSense(const Ball *ball) const;

protected:

	RobotRevision _rev;

	// state info
	Geometry2d::Point _startPos;

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
