#include "Ball.hpp"
#include "Env.hpp"

#include <Constants.hpp>

#include <stdio.h>

Ball::Ball(Env* env) :
	Entity(env)
{
	//create entity
	NxBodyDesc bodyDesc;
	bodyDesc.setToDefault();
	bodyDesc.mass = Constants::Ball::Mass;

	NxSphereShapeDesc sphereDesc;
	
	sphereDesc.radius = Constants::Ball::Radius;
	sphereDesc.mass = Constants::Ball::Mass;
	
	NxSimpleTriangleMesh mesh;
	mesh.numVertices = 1;
	
	NxVec3 vert;
	vert.z = .5f;
	mesh.points = &vert;
	
	sphereDesc.ccdSkeleton = env->_physicsSDK->createCCDSkeleton(mesh);

	// These materials apparently don't have any effect
	NxMaterialDesc materialDesc;
	materialDesc.restitution = 0.1f; // 0.4f;
	materialDesc.staticFriction = 100.0f; //20.0f;
	materialDesc.dynamicFriction = 100.0f; //20.0f;
	materialDesc.frictionCombineMode = NX_CM_MAX;

	NxMaterial* newMaterial = _scene.createMaterial(materialDesc);
	sphereDesc.materialIndex = newMaterial->getMaterialIndex();

	//setup the ball actor
	NxActorDesc actorDesc;
	actorDesc.setToDefault();

	actorDesc.shapes.pushBack(&sphereDesc);
	actorDesc.body = &bodyDesc;
	//actorDesc.density = 1.0f;
	actorDesc.globalPose.t = NxVec3(0.0f, 0.0f, 0.5f); //Position at the origin.

	//this creates an actor in the scene
	_actor = _scene.createActor(actorDesc);
	
	//_actor->setLinearDamping (0.1f);
	_actor->setAngularDamping (2.0f); // changed from 3 -> 2 to better match what we see on the field.
	_actor->setMaxAngularVelocity (200.0f);

	if (!_actor)
	{
		printf("Failed to create actor\n");
		//TODO throw exception
	}
}

Ball::~Ball()
{
	
}

void Ball::position(float x, float y)
{
	_actor->setGlobalPosition(NxVec3(x, y, Constants::Ball::Radius/2.0 + 0.01f));
}

void Ball::velocity(float x, float y)
{
    _actor->setLinearVelocity(NxVec3(x, y, 0));
    
    // Need to do this because the body may get an unexpected linear velocity from a collision (even with the ground)
    _actor->setAngularVelocity(NxVec3(0, 0, 0));
}
