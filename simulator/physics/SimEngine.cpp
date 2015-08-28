/*
 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the
 use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:

 1. The origin of this software must not be misrepresented; you must not claim
 that you wrote the original software. If you use this software in a product, an
 acknowledgment in the product documentation would be appreciated but is not
 required.
 2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */

#include <physics/SimEngine.hpp>
#include <stdio.h>
#include "RobotMotionState.hpp"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"

using namespace std;

SimEngine::SimEngine()
    : _dynamicsWorld(nullptr),
      _stepping(true),
      _singleStep(false),
      _debugMode(0),
      _defaultContactProcessingThreshold(BT_LARGE_FLOAT) {}

void SimEngine::initPhysics() {
    _collisionConfiguration = new btDefaultCollisionConfiguration();
    _dispatcher = new btCollisionDispatcher(_collisionConfiguration);
    btVector3 worldMin(-1000, -1000, -1000);
    btVector3 worldMax(1000, 1000, 1000);
    _overlappingPairCache = new btAxisSweep3(worldMin, worldMax);
    _constraintSolver = new btSequentialImpulseConstraintSolver();
    _dynamicsWorld =
        new btDiscreteDynamicsWorld(_dispatcher, _overlappingPairCache,
                                    _constraintSolver, _collisionConfiguration);
    //
    _dynamicsWorld->getSolverInfo().m_splitImpulse = true;
    // We use ghost objects simulate the mouth of the robot, see
    // RobotBallController
    btGhostPairCallback* ghostPairCallback = new btGhostPairCallback();
    _dynamicsWorld->getPairCache()->setInternalGhostPairCallback(
        ghostPairCallback);
}

SimEngine::~SimEngine() {
    // cleanup in the reverse order of creation/initialization

    // remove the rigidbodies from the dynamics world and delete them
    for (int i = _dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
        btCollisionObject* obj = _dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState()) {
            delete body->getMotionState();
        }
        _dynamicsWorld->removeCollisionObject(obj);
        delete obj;
    }

    // delete collision shapes
    for (int j = 0; j < _collisionShapes.size(); j++) {
        btCollisionShape* shape = _collisionShapes[j];
        delete shape;
    }

    // delete dynamics world
    delete _dynamicsWorld;

    // delete solver
    delete _constraintSolver;

    // delete broadphase
    delete _overlappingPairCache;

    // delete dispatcher
    delete _dispatcher;

    delete _collisionConfiguration;
}

void SimEngine::setDebug(const btIDebugDraw::DebugDrawModes& flag) {
    if (_debugMode & flag)
        _debugMode = _debugMode & (~flag);
    else
        _debugMode |= flag;
}

btRigidBody* SimEngine::localCreateRigidBody(float mass,
                                             const btTransform& startTransform,
                                             btCollisionShape* shape) {
    btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

    // rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic) shape->calculateLocalInertia(mass, localInertia);

// using motionstate is recommended, it provides interpolation capabilities, and
// only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
    btDefaultMotionState* myMotionState =
        new btDefaultMotionState(startTransform);

    btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape,
                                                   localInertia);

    btRigidBody* body = new btRigidBody(cInfo);
    body->setContactProcessingThreshold(_defaultContactProcessingThreshold);
#else
    btRigidBody* body = new btRigidBody(mass, 0, shape, localInertia);
    body->setWorldTransform(startTransform);
#endif  //
    _dynamicsWorld->addRigidBody(body);

    return body;
}

btRigidBody* SimEngine::localCreateRobot(float mass,
                                         const btTransform& startTransform,
                                         btCollisionShape* shape) {
    btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

    // rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic) shape->calculateLocalInertia(mass, localInertia);

    // using motionstate is recommended, it provides interpolation capabilities,
    // and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new RobotMotionState(startTransform);

    btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape,
                                                   localInertia);

    btRigidBody* body = new btRigidBody(cInfo);
    body->setContactProcessingThreshold(_defaultContactProcessingThreshold);

    _dynamicsWorld->addRigidBody(body);

    return body;
}

void SimEngine::addCollisionShape(btCollisionShape* shape) {
    _collisionShapes.push_back(shape);
}

btScalar SimEngine::getDeltaTimeMicroseconds() {
    btScalar dt = (btScalar)_clock.getTimeMicroseconds();
    _clock.reset();
    return dt;
}

btClock* SimEngine::getClock() { return &_clock; }

void SimEngine::stepSimulation() {
    float dt = getDeltaTimeMicroseconds() * 0.000001f;
    if (_dynamicsWorld) {
        // during idle mode, just run 1 simulation step maximum
        int maxSimSubSteps = 2;
        _dynamicsWorld->stepSimulation(dt, maxSimSubSteps);
    }
}

void SimEngine::debugDrawWorld() {
    if (_dynamicsWorld) _dynamicsWorld->debugDrawWorld();
}

void SimEngine::addVehicle(btActionInterface* vehicle) {
    if (_dynamicsWorld) _dynamicsWorld->addVehicle(vehicle);
}
