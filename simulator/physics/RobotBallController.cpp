#include "RobotBallController.hpp"
#include "Robot.hpp"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include <math.h>
//DEBUG
#include <stdio.h>

//static constants (loosely guessed)
static const float MouthWidth = 0.075*scaling;
static const float MouthHeight = 0.06*scaling;//based off ball diam
static const float MouthLength = 0.005*scaling;

RobotBallController::RobotBallController(Robot* robot) :
	_ghostObject(0),_localMouthPos(0,0,0),_parent(robot), _simEngine(robot->getSimEngine())
{
}

RobotBallController::~RobotBallController()
{
}

void RobotBallController::initPhysics()
{
	_ghostObject = new btPairCachingGhostObject();

	///Create mouth shape for collision detection
	btVector3 mouthHalfDim = btVector3(MouthWidth/2.f,MouthHeight/2.f,MouthLength/2.f);
	btBoxShape* mouthShape = new btBoxShape(mouthHalfDim);
	_simEngine->addCollisionShape(mouthShape);

	_localMouthPos = btVector3(0,MouthHeight/2.f,MouthLength/2.f);
	float mrad = asin((Sim_Robot_MouthWidth/2.f)/Sim_Robot_Radius); //angle from mouth center to corner
	float distToMouth = Sim_Robot_Radius*cos(mrad);
	_localMouthPos[2] += distToMouth;

	btTransform ghostTr;
	_parent->getWorldTransform(ghostTr);
	btVector3 origin = ghostTr.getOrigin();
	origin.setY(0);
	origin += _localMouthPos.rotate(ghostTr.getRotation().getAxis(),ghostTr.getRotation().getAngle());
	ghostTr.setOrigin(origin);

	_ghostObject->setWorldTransform(ghostTr);
	_ghostObject->setCollisionShape(mouthShape);
	_ghostObject->setCollisionFlags (btCollisionObject::CF_NO_CONTACT_RESPONSE);

	_simEngine->dynamicsWorld()->addCollisionObject(_ghostObject,btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::SensorTrigger);

	_simEngine->dynamicsWorld()->addAction(this);
}


bool RobotBallController::detectBall ( btCollisionWorld* collisionWorld)
{
	btManifoldArray   manifoldArray;
	btBroadphasePairArray& pairArray = _ghostObject->getOverlappingPairCache()->getOverlappingPairArray();
	int numPairs = pairArray.size();

	for (int i=0;i<numPairs;i++)
	{
		manifoldArray.clear();

		const btBroadphasePair& pair = pairArray[i];

		//unless we manually perform collision detection on this pair, the contacts are in the dynamics world paircache:
		btBroadphasePair* collisionPair = _simEngine->dynamicsWorld()->getPairCache()->findPair(pair.m_pProxy0,pair.m_pProxy1);
		if (!collisionPair)
			continue;

		if (collisionPair->m_algorithm)
			collisionPair->m_algorithm->getAllContactManifolds(manifoldArray);

		for (int j=0;j<manifoldArray.size();j++)
		{
			btPersistentManifold* manifold = manifoldArray[j];
			btScalar directionSign = manifold->getBody0() == _ghostObject ? btScalar(-1.0) : btScalar(1.0);
			for (int p=0;p<manifold->getNumContacts();p++)
			{
				const btManifoldPoint&pt = manifold->getContactPoint(p);
				if (pt.getDistance()<0.f)
				{
					const btVector3& ptA = pt.getPositionWorldOnA();
					const btVector3& ptB = pt.getPositionWorldOnB();
					const btVector3& normalOnB = pt.m_normalWorldOnB;
					/// work here

					printf("Ball detected!\n");

				}
			}
		}
	}

	return true;
}

void RobotBallController::dribblerStep()
{
}

void RobotBallController::kickerStep()
{
}

void RobotBallController::syncMotionState(const btTransform& centerOfMassWorldTrans)
{
	btTransform syncTr = centerOfMassWorldTrans;
	btVector3 syncOrigin = syncTr.getOrigin();
	syncOrigin.setY(0);
	syncOrigin += _localMouthPos.rotate(syncTr.getRotation().getAxis(),syncTr.getRotation().getAngle());
	syncTr.setOrigin(syncOrigin);
	_ghostObject->setWorldTransform(syncTr);
}
