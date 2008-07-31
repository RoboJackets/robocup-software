#include "Ball.hpp"

#include <Constants.hpp>

#include <stdio.h>

Ball::Ball(NxScene& scene) :
    Entity(scene)
{
    //create entity
    NxBodyDesc bodyDesc;
    bodyDesc.setToDefault();
    bodyDesc.mass = Constants::Ball::Mass;
    
    NxSphereShapeDesc sphereDesc;
    
    //TODO change to constant
    sphereDesc.radius = Constants::Ball::Radius;
    sphereDesc.mass = Constants::Ball::Mass;
    
    NxMaterialDesc materialDesc;
    materialDesc.restitution = 0.4f;
    materialDesc.staticFriction = 0.6f;
    materialDesc.dynamicFriction = 0.5f;

    NxMaterial* newMaterial=scene.createMaterial(materialDesc);
    sphereDesc.materialIndex= newMaterial->getMaterialIndex();
    
    //setup the ball actor
    NxActorDesc actorDesc;
    actorDesc.setToDefault();
    
    actorDesc.shapes.pushBack(&sphereDesc);
    actorDesc.body = &bodyDesc;
    //actorDesc.density = 1.0f;
    actorDesc.globalPose.t = NxVec3(0.0f,0.0f,0.5f); //Position at the origin.

    //this creates an actor in the scene
    _actor = scene.createActor(actorDesc);
    
    if (!_actor)
    {
        printf("Failed to create actor\n");
        //TODO throw exception
    }
    
    //create new glu graphics quadric
    _quadric = gluNewQuadric();
}

Ball::~Ball()
{
    gluDeleteQuadric(_quadric);
}

void Ball::paint() const
{
    NxVec3 pos = _actor->getGlobalPosition();
    
    //orange
    glColor3ub(0xff, 0x90, 0);
  
    //draw ball
    glPushMatrix();
        glTranslatef(pos.x, pos.y, pos.z);
        //TODO change to constant
        gluSphere(_quadric, Constants::Ball::Radius, 10, 10); //radius, slices, stacks
    glPopMatrix();
}
