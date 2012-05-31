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
	_ball = 0;

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

		btCollisionObject* obj = (btCollisionObject*) collisionPair->m_pProxy0->m_clientObject;
		if(collisionPair->m_pProxy0->m_clientObject == _ghostObject)
			obj = (btCollisionObject*) collisionPair->m_pProxy1->m_clientObject;

		btRigidBody* ball = btRigidBody::upcast(obj);

		if(!ball)
			continue;

		if(ball->getCollisionShape()->getShapeType() != SPHERE_SHAPE_PROXYTYPE)
			continue;


		_ball = ball;

		//Can use penetration dist to more accurately determine when ball is in contact
		/*if (collisionPair->m_algorithm)
			collisionPair->m_algorithm->getAllContactManifolds(manifoldArray);

		for (int j=0;j<manifoldArray.size();j++)
		{
			printf("Manifold #%d\n",j);

			btPersistentManifold* manifold = manifoldArray[j];
			btScalar directionSign = manifold->getBody0() == _ghostObject ? btScalar(-1.0) : btScalar(1.0);

			printf("directionSign = %d\n",directionSign);

			for (int p=0;p<manifold->getNumContacts();p++)
			{
				const btManifoldPoint&pt = manifold->getContactPoint(p);

				printf("Contact point #%d\n",p);
				printf("localPointA = (%5.3f,%5.3f,%5.3f)\n",pt.m_localPointA[0],pt.m_localPointA[1],pt.m_localPointA[2]);
				printf("localPointB = (%5.3f,%5.3f,%5.3f)\n",pt.m_localPointB[0],pt.m_localPointB[1],pt.m_localPointB[2]);
				printf("positionWorldOnA = (%5.3f,%5.3f,%5.3f)\n",pt.m_positionWorldOnA[0],pt.m_positionWorldOnA[1],pt.m_positionWorldOnA[2]);
				printf("positionWorldOnB = (%5.3f,%5.3f,%5.3f)\n",pt.m_positionWorldOnB[0],pt.m_positionWorldOnB[1],pt.m_positionWorldOnB[2]);
				printf("normalWorldOnB = (%5.3f,%5.3f,%5.3f)\n",pt.m_normalWorldOnB[0],pt.m_normalWorldOnB[1],pt.m_normalWorldOnB[2]);
				printf("Penetration dist = %5.3f\n",pt.getDistance()*directionSign);

				if (pt.getDistance()<0.f)
				{
					const btVector3& ptA = pt.getPositionWorldOnA();
					const btVector3& ptB = pt.getPositionWorldOnB();
					const btVector3& normalOnB = pt.m_normalWorldOnB;
					/// work here

				}
			}
		}*/


	}

	return true;
}

void RobotBallController::dribblerStep()
{
	if(_ball){

//#define USE_DIRECT_PLACEMENT 1
#ifdef USE_DIRECT_PLACEMENT

		btTransform robotTr;
		_parent->getWorldTransform(robotTr);
		btVector3 robotPos = robotTr.getOrigin();
		robotPos[1] = 0;

		btTransform sensorTr = _ghostObject->getWorldTransform();
		btVector3 sensorPos = sensorTr.getOrigin();
		sensorPos[1] = 0;

		//ball point in front of robot
		btVector3 forward = (sensorPos-robotPos).normalize()*((sensorPos-robotPos).length()+Sim_Ball_Radius) + robotPos;

		btVector3 ballPos = forward;
		ballPos[1] = Sim_Ball_Radius;

		btTransform ballTr = _ball->getWorldTransform();
		ballTr.setOrigin(ballPos);
		_ball->setWorldTransform(ballTr);
#endif

//This appears to work the best
#define USE_BACKWARDS_FORCE 1
#ifdef USE_BACKWARDS_FORCE

		float BackwardsForce = 30;

		btTransform robotTr;
		_parent->getWorldTransform(robotTr);
		btVector3 robotPos = robotTr.getOrigin();
		robotPos[1] = 0; // set height

		btTransform sensorTr;
		sensorTr = _ghostObject->getWorldTransform();
		btVector3 sensorPos = sensorTr.getOrigin();
		sensorPos[1] = 0;

		btVector3 target = robotPos+(sensorPos-robotPos)*0.99f;

		btVector3 ballPos = _ball->getCenterOfMassPosition();
		ballPos[1] = 0;

//		btVector3 backDir = (robotPos - sensorPos).normalize();
		btVector3 backDir = (target - ballPos).normalize();

		//oppose the rotation?

		btVector3 backForce = BackwardsForce*backDir;

		_ball->applyCentralForce(backForce);

		_ball->setAngularVelocity(_ball->getAngularVelocity()*0);
#endif

//#define USE_BACKWARDS_TORQUE 1
#ifdef USE_BACKWARDS_TORQUE

		float BackwardsTorque = 10;

		btTransform robotTr;
		_parent->getWorldTransform(robotTr);
		btVector3 robotPos = robotTr.getOrigin();
		robotPos[1] = 0; // set height

		btVector3 ballPos = _ball->getCenterOfMassPosition();
		ballPos[1] = 0;

		btVector3 forwardDir = (ballPos - robotPos).normalize();

		btVector3 backTorque = forwardDir.cross(btVector3(0,1,0))*BackwardsTorque;

		_ball->applyTorque(backTorque);

#endif

	}
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
