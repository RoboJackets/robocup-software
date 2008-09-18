#include "Field.hpp"

#include <Constants.hpp>

Field::Field(NxScene& scene) :
    Entity(scene)
{
    //field actor has no body, it is static
    NxActorDesc floorDesc;
    floorDesc.setToDefault();

    //floor is just an infinite plane
    NxPlaneShapeDesc planeDesc;
    planeDesc.normal = NxVec3(0, 0, 1);
    planeDesc.d = 0;
    
    floorDesc.shapes.pushBack(&planeDesc);
    
    //TODO add goal shapes
    //TODO add material
    
    _actor = scene.createActor(floorDesc);
}

Field::~Field()
{
    
}

void Field::paint() const
{
    const NxVec3 pos = _actor->getGlobalPosition();
    
    glPushMatrix();
    glTranslatef(pos.x - Constants::Floor::Length/2.0f, 
            pos.y - Constants::Floor::Width/2.0f, pos.z);
        
    //floor color
    glColor3ub(0, 150, 0);
    
    glBegin(GL_QUADS);
        glVertex2f(0, 0);
        glVertex2f(0, Constants::Floor::Width);
        glVertex2f(Constants::Floor::Length, Constants::Floor::Width);
        glVertex2f(Constants::Floor::Length, 0);
    glEnd();
    
    //glTranslatef(FIELD_DEADSPACE, FIELD_DEADSPACE, 0);
    
    glPopMatrix();
}
