#include "Field.hpp"

#include <Constants.hpp>
#include <vector>

using namespace std;

NxBoxShapeDesc box(float w, float h, float x, float y)
{
	NxBoxShapeDesc box;
	box.dimensions = NxVec3(w/2, h/2, Field_GoalHeight/2.0);
	box.localPose.t = NxVec3(x, y, Field_GoalHeight/2.0);
	
	return box;
}

Field::Field(Env* env) :
	Entity(env)
{
	//field actor has no body, it is static
	NxActorDesc floorDesc;

	//floor is just an infinite plane
	NxPlaneShapeDesc planeDesc;
	planeDesc.normal = NxVec3(0, 0, 1);
	planeDesc.d = 0;

	/*
	NxMaterialDesc materialDesc;
	materialDesc.restitution = 0.0f;
	materialDesc.staticFriction = 0.5f;
	materialDesc.dynamicFriction = 0.4f;

	NxMaterial* newMaterial = scene.createMaterial(materialDesc);
	planeDesc.materialIndex = newMaterial->getMaterialIndex();
	*/
	floorDesc.shapes.pushBack(&planeDesc);
	
	NxBoxShapeDesc rear = box(.001, Field_GoalWidth, 
			-Field_Length/2.0 - Field_GoalDepth, 0);
	
	NxBoxShapeDesc side1 = box(Field_GoalDepth, .001, 
			-Field_Length/2.0 - Field_GoalDepth/2.0, 
			Field_GoalWidth/2.0);
	
	NxBoxShapeDesc side2 = side1;
	side2.localPose.t.y *= -1.0f;
	
	floorDesc.shapes.pushBack(&rear);
	floorDesc.shapes.pushBack(&side1);
	floorDesc.shapes.pushBack(&side2);
	
	NxBoxShapeDesc rear2 = rear;
	rear2.localPose.t.x *= -1.0f;
	
	NxBoxShapeDesc side21 = side1;
	side21.localPose.t.x *= -1.0f;
	
	NxBoxShapeDesc side22 = side2;
	side22.localPose.t.x *= -1.0f;
	
	floorDesc.shapes.pushBack(&rear2);
	floorDesc.shapes.pushBack(&side21);
	floorDesc.shapes.pushBack(&side22);
	
	NxBoxShapeDesc wall1 = box(Floor_Length, .001, 0, Floor_Width/2.0);
	NxBoxShapeDesc wall2 = wall1;
	wall2.localPose.t.y *= -1.0f;
	
	NxBoxShapeDesc wall3 = box(.001, Floor_Width, Floor_Length/2.0, 0);
	NxBoxShapeDesc wall4 = wall3;
	wall4.localPose.t.x *= -1.0f;
	
	floorDesc.shapes.pushBack(&wall1);
	floorDesc.shapes.pushBack(&wall2);
	floorDesc.shapes.pushBack(&wall3);
	floorDesc.shapes.pushBack(&wall4);
	
	_actor = _scene.createActor(floorDesc);
}

Field::~Field()
{
	
}
