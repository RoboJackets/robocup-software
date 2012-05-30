#pragma once

#include <BulletDynamics/Dynamics/btActionInterface.h>
#include "SimEngine.hpp"

class Robot;
class Ball;
class btPairCachingGhostObject;

///RobotBallController is an object that handles dribbling and kicking/chipping for the parent robot.
///It uses a ghost object to test for collisions with the ball.
class RobotBallController : public btActionInterface
{
protected:
	btPairCachingGhostObject* _ghostObject;

	btVector3 _localMouthPos;

	Robot* _parent;
	Ball* _ball;

	///links to the engine
	SimEngine *_simEngine;

public:
	RobotBallController(Robot* robot);
	~RobotBallController();

	void initPhysics();

	///btActionInterface interface
	virtual void updateAction( btCollisionWorld* collisionWorld,btScalar deltaTime){
		detectBall(collisionWorld);
		dribblerStep();
		kickerStep();
	}

	///btActionInterface interface
	void	debugDraw(btIDebugDraw* debugDrawer) {};

	bool detectBall(btCollisionWorld* collisionWorld);
	void dribblerStep();
	void kickerStep();

	void syncMotionState(const btTransform& centerOfMassWorldTrans);
};
