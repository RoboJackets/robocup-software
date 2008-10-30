#include "Field.hpp"

#include <Constants.hpp>
#include <vector>

using namespace std;

NxBoxShapeDesc box(float w, float h, float x, float y)
{
	NxBoxShapeDesc box;
	box.dimensions = NxVec3(w/2, h/2, Constants::Field::GoalHeight/2.0);
	box.localPose.t = NxVec3(x, y, Constants::Field::GoalHeight/2.0);
	
	return box;
}

Field::Field(NxScene& scene) :
	Entity(scene)
{
	//field actor has no body, it is static
	NxActorDesc floorDesc;

	//floor is just an infinite plane
	NxPlaneShapeDesc planeDesc;
	planeDesc.normal = NxVec3(0, 0, 1);
	planeDesc.d = 0;

	floorDesc.shapes.pushBack(&planeDesc);
	
	NxBoxShapeDesc rear = box(.005, Constants::Field::GoalWidth, 
			-Constants::Field::Length/2.0 - Constants::Field::GoalDepth, 0);
	
	NxBoxShapeDesc side1 = box(Constants::Field::GoalDepth, .005, 
			-Constants::Field::Length/2.0 - Constants::Field::GoalDepth/2.0, 
			Constants::Field::GoalWidth/2.0);
	
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
	
	NxBoxShapeDesc wall1 = box(Constants::Floor::Length, .005, 0, Constants::Floor::Width/2.0);
	NxBoxShapeDesc wall2 = wall1;
	wall2.localPose.t.y *= -1.0f;
	
	NxBoxShapeDesc wall3 = box(.005, Constants::Floor::Width, Constants::Floor::Length/2.0, 0);
	NxBoxShapeDesc wall4 = wall3;
	wall4.localPose.t.x *= -1.0f;
	
	floorDesc.shapes.pushBack(&wall1);
	floorDesc.shapes.pushBack(&wall2);
	floorDesc.shapes.pushBack(&wall3);
	floorDesc.shapes.pushBack(&wall4);
	
	_actor = scene.createActor(floorDesc);
}

Field::~Field()
{
	
}
