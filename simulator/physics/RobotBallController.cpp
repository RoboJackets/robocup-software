#include "RobotBallController.hpp"
#include "Robot.hpp"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include <math.h>
#include <Utils.hpp>
// DEBUG
#include <stdio.h>

// static constants (guessed with science)
static const float MouthWidth = 0.075 * scaling;
static const float MouthHeight = 0.06 * scaling;  // based off ball diam
static const float MouthLength = 0.005 * scaling;

static const float MaxKickVelocity = 8 * scaling;  // m/s
static const float MaxChipVelocity = 3 * scaling;
static const float ChipAngle = 0.34906585;  // 20 degree

RobotBallController::RobotBallController(Robot* robot)
    : _ghostObject(nullptr),
      _localMouthPos(0, 0, 0),
      _parent(robot),
      _ball(nullptr),
      _simEngine(robot->getSimEngine()) {
    ballSensorWorks = true;
    chargerWorks = true;
    _lastKicked = 0;

    _kick = false;
    _chip = false;
    _dribble = 0;
}

RobotBallController::~RobotBallController() {}

void RobotBallController::initPhysics() {
    _ghostObject = new btPairCachingGhostObject();

    /// Create mouth shape for collision detection
    btVector3 mouthHalfDim =
        btVector3(MouthWidth / 2.f, MouthHeight / 2.f, MouthLength / 2.f);
    btBoxShape* mouthShape = new btBoxShape(mouthHalfDim);
    _simEngine->addCollisionShape(mouthShape);

    _localMouthPos = btVector3(0, MouthHeight / 2.f, MouthLength / 2.f);
    float mrad = asin((Sim_Robot_MouthWidth / 2.f) /
                      Sim_Robot_Radius);  // angle from mouth center to corner
    float distToMouth = Sim_Robot_Radius * cos(mrad);
    _localMouthPos[2] += distToMouth;

    btTransform ghostTr;
    _parent->getWorldTransform(ghostTr);
    btVector3 origin = ghostTr.getOrigin();
    origin.setY(0);
    origin += _localMouthPos.rotate(ghostTr.getRotation().getAxis(),
                                    ghostTr.getRotation().getAngle());
    ghostTr.setOrigin(origin);

    _ghostObject->setWorldTransform(ghostTr);
    _ghostObject->setCollisionShape(mouthShape);
    _ghostObject->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);

    _simEngine->dynamicsWorld()->addCollisionObject(
        _ghostObject, btBroadphaseProxy::DefaultFilter,
        btBroadphaseProxy::SensorTrigger);

    _simEngine->dynamicsWorld()->addAction(this);
}

bool RobotBallController::detectBall(btCollisionWorld* collisionWorld) {
    _ball = nullptr;

    btManifoldArray manifoldArray;
    btBroadphasePairArray& pairArray =
        _ghostObject->getOverlappingPairCache()->getOverlappingPairArray();
    int numPairs = pairArray.size();

    for (int i = 0; i < numPairs; i++) {
        manifoldArray.clear();

        const btBroadphasePair& pair = pairArray[i];

        // unless we manually perform collision detection on this pair, the
        // contacts are in the dynamics world paircache:
        btBroadphasePair* collisionPair =
            _simEngine->dynamicsWorld()->getPairCache()->findPair(
                pair.m_pProxy0, pair.m_pProxy1);
        if (!collisionPair) continue;

        btCollisionObject* obj =
            (btCollisionObject*)collisionPair->m_pProxy0->m_clientObject;
        if (collisionPair->m_pProxy0->m_clientObject == _ghostObject)
            obj = (btCollisionObject*)collisionPair->m_pProxy1->m_clientObject;

        btRigidBody* ball = btRigidBody::upcast(obj);

        if (!ball) continue;

        if (ball->getCollisionShape()->getShapeType() != SPHERE_SHAPE_PROXYTYPE)
            continue;

        _ball = ball;
    }

    return true;
}

void RobotBallController::dribblerStep() {
    if (_ball && _dribble) {
//#define USE_DIRECT_PLACEMENT 1
#ifdef USE_DIRECT_PLACEMENT

        btTransform robotTr;
        _parent->getWorldTransform(robotTr);
        btVector3 robotPos = robotTr.getOrigin();
        robotPos[1] = 0;

        btTransform sensorTr = _ghostObject->getWorldTransform();
        btVector3 sensorPos = sensorTr.getOrigin();
        sensorPos[1] = 0;

        // ball point in front of robot
        btVector3 forward =
            (sensorPos - robotPos).normalize() *
                ((sensorPos - robotPos).length() + Sim_Ball_Radius) +
            robotPos;

        btVector3 ballPos = forward;
        ballPos[1] = Sim_Ball_Radius;

        btTransform ballTr = _ball->getWorldTransform();
        ballTr.setOrigin(ballPos);
        _ball->setWorldTransform(ballTr);
#endif

// This appears to work the best
#define USE_BACKWARDS_FORCE 1
#ifdef USE_BACKWARDS_FORCE

        float BackwardsForce = 30;

        btTransform robotTr;
        _parent->getWorldTransform(robotTr);
        btVector3 robotPos = robotTr.getOrigin();
        robotPos[1] = 0;  // set height

        btTransform sensorTr;
        sensorTr = _ghostObject->getWorldTransform();
        btVector3 sensorPos = sensorTr.getOrigin();
        sensorPos[1] = 0;

        btVector3 target = robotPos + (sensorPos - robotPos) * 0.7f;

        btVector3 ballPos = _ball->getCenterOfMassPosition();
        ballPos[1] = 0;

        //		btVector3 backDir = (robotPos - sensorPos).normalize();
        btVector3 backDir = (target - ballPos).normalize();

        // oppose the rotation?

        btVector3 backForce = BackwardsForce * backDir;

        _ball->applyCentralForce(backForce);

        _ball->setAngularVelocity(_ball->getAngularVelocity() * 0);
#endif

//#define USE_BACKWARDS_TORQUE 1
#ifdef USE_BACKWARDS_TORQUE

        float BackwardsTorque = 10;

        btTransform robotTr;
        _parent->getWorldTransform(robotTr);
        btVector3 robotPos = robotTr.getOrigin();
        robotPos[1] = 0;  // set height

        btVector3 ballPos = _ball->getCenterOfMassPosition();
        ballPos[1] = 0;

        btVector3 forwardDir = (ballPos - robotPos).normalize();

        btVector3 backTorque =
            forwardDir.cross(btVector3(0, 1, 0)) * BackwardsTorque;

        _ball->applyTorque(backTorque);

#endif
    }
}

void RobotBallController::kickerStep() {
    if (!_ball) return;
    if (_kick && (timestamp() - _lastKicked) > RechargeTime && chargerWorks) {
        btVector3 dir =
            _parent->getRigidBody()->getWorldTransform().getOrigin();
        dir -= _ghostObject->getWorldTransform().getOrigin();
        dir *= -1;
        dir[1] = 0;
        dir = dir.normalize();

        printf("Robot %d %s at (%5.3f,%5.3f) with power %lu, speed %5.3f \n",
               _parent->shell, _chip ? "chipped" : "kicked", dir[1] / scaling,
               dir[0] / scaling, _kick, _kickSpeed / scaling);

        if (_chip) {
            btVector3 axis = dir.cross(btVector3(0, 1, 0));
            dir = dir.rotate(axis, ChipAngle);
        }
        _ball->clearForces();
        _ball->setLinearVelocity(dir * _kickSpeed);

        _kick = 0;
        _chip = false;

        _lastKicked = timestamp();
    }
}

void RobotBallController::syncMotionState(
    const btTransform& centerOfMassWorldTrans) {
    btTransform syncTr = centerOfMassWorldTrans;
    btVector3 syncOrigin = syncTr.getOrigin();
    syncOrigin.setY(0);
    syncOrigin += _localMouthPos.rotate(syncTr.getRotation().getAxis(),
                                        syncTr.getRotation().getAngle());
    syncTr.setOrigin(syncOrigin);
    _ghostObject->setWorldTransform(syncTr);
}

void RobotBallController::prepareKick(uint64_t power, bool chip) {
    if (!power) {
        _kick = power;
        _chip = false;
        _kickSpeed = 0;
    }
    if ((timestamp() - _lastKicked) > RechargeTime && chargerWorks) {
        _kick = power;
        // determine the kick speed
        _chip = chip;  // && _rev == rev2011;
        if (_chip) {
            _kickSpeed = ((int)power) / 255.0f * MaxChipVelocity;
        } else {
            _kickSpeed = ((int)power) / 255.0f * MaxKickVelocity;
        }
    }
}

void RobotBallController::prepareDribbler(uint64_t dribble) {
    _dribble = dribble;
}

bool RobotBallController::hasBall() { return _ball != nullptr; }

bool RobotBallController::getKickerStatus() {
    return (timestamp() - _lastKicked) > RechargeTime ? 1 : 0;
}
