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
#pragma once

#include <btBulletDynamicsCommon.h>
#include <LinearMath/btQuickprof.h>
#include <physics/PhysicsConstants.hpp>

class btCollisionShape;
class btDynamicsWorld;
class btRigidBody;
class btActionInterface;

class SimEngine {
protected:
    btAlignedObjectArray<btCollisionShape*> _collisionShapes;
    class btBroadphaseInterface* _overlappingPairCache;
    class btCollisionDispatcher* _dispatcher;
    class btConstraintSolver* _constraintSolver;
    class btDefaultCollisionConfiguration* _collisionConfiguration;
    btClock _clock;
    btDynamicsWorld* _dynamicsWorld;
    bool _stepping;
    bool _singleStep;
    int _debugMode;
    btScalar _defaultContactProcessingThreshold;

public:
    /** creates a new engine - does not initialize until initPhysics() */
    SimEngine();
    ~SimEngine();

    /** actually initializes physics objects */
    void initPhysics();

    // access
    btDynamicsWorld* dynamicsWorld() const { return _dynamicsWorld; }
    int debugMode() const { return _debugMode; }
    void debugMode(int mode) { _debugMode = mode; }

    void setDebug(const btIDebugDraw::DebugDrawModes& flag);

    void addCollisionShape(btCollisionShape* shape);

    btRigidBody* localCreateRigidBody(float mass,
                                      const btTransform& startTransform,
                                      btCollisionShape* shape);

    btRigidBody* localCreateRobot(float mass, const btTransform& startTransform,
                                  btCollisionShape* shape);

    btScalar getDeltaTimeMicroseconds();

    /** Key function for advancing the simulation forward in time */
    void stepSimulation();

    btClock* getClock();

    void debugDrawWorld();

    void addVehicle(btActionInterface* vehicle);
};
