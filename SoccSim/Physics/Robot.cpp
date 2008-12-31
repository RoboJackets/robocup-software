#include "Robot.hpp"
#include <NxCooking.h>

#include <Constants.hpp>

#include "Env.hpp"
#include "MemoryStream.hpp"

#define ROLLER  0
#define KICKER  0
#define CHIPPER 0

using namespace Geometry;

Robot::Robot(NxScene& scene) :
	Entity(scene)
{
	for (int i = 0; i < 4; ++i)
	{
		vels[i] = 0;
	}

	//robot has a kicker, chipper, roller and body

	//create entity
	NxBodyDesc bodyDesc;
	bodyDesc.mass = 5.0f;

	//TODO fixme, use real shell mesh
	NxConvexShapeDesc shapeDesc;
	shapeDesc.meshData = cylinder(Constants::Robot::Height,
	        Constants::Robot::Radius, 20);

	shapeDesc.localPose.t = NxVec3(0.0f, 0.0f, Constants::Robot::Height/2.0);

	//setup the ball actor
	NxActorDesc actorDesc;

	actorDesc.shapes.pushBack(&shapeDesc);
	actorDesc.body = &bodyDesc;

	//this creates an actor in the scene
	_actor = scene.createActor(actorDesc);

	if (!_actor)
	{
		printf("Failed to create actor\n");
		//TODO throw exception
	}

	_roller = 0;
	_kicker = 0;

	_wheels[0] = _wheels[1] = _wheels[2] = _wheels[3] = 0;
	_motors[0] = _motors[1] = _motors[2] = _motors[3] = 0;


#if (ROLLER)
	initRoller();
#endif

#if (KICKER)
	initKicker();
#endif

	initWheels();
}

Robot::~Robot()
{
	if (_roller)
	{
		_scene.releaseActor(*_roller);
		_roller = 0;
	}
}

void Robot::initRoller()
{
	NxBodyDesc bodyDesc;
	bodyDesc.mass = 0.01f; //TODO fixme
	NxConvexShapeDesc shapeDesc;
	shapeDesc.meshData = cylinder(Robot::RollerLength, Robot::RollerRadius, 20);

	NxMaterialDesc materialDesc;
	materialDesc.restitution = 0.4f;
	materialDesc.staticFriction = 0.5f;
	materialDesc.dynamicFriction = 0.5f;

	NxMaterial* newMaterial = _scene.createMaterial(materialDesc);
	shapeDesc.materialIndex = newMaterial->getMaterialIndex();

	NxActorDesc actorDesc;
	actorDesc.shapes.pushBack(&shapeDesc);
	actorDesc.body = &bodyDesc;

	actorDesc.globalPose.t
	        = NxVec3(Robot::RollerOffset, 0, Robot::RollerHeight);
	actorDesc.globalPose.M.rotX(M_PI / 2.0);

	if (!actorDesc.isValid())
	{
		printf("not valid\n");
		return;
	}

	_roller = _scene.createActor(actorDesc);

	//setup a joint to the roller
	NxRevoluteJointDesc revDesc;

	revDesc.actor[0] = _actor;
	revDesc.actor[1] = _roller;

	revDesc.setGlobalAxis(NxVec3(0, 1, 0)); //The direction of the axis the bodies revolve around.
	revDesc.setGlobalAnchor(NxVec3(RollerOffset, 0, RollerHeight)); //Reference point that the axis passes through.

	revDesc.motor.maxForce = 5.0f;
	revDesc.motor.velTarget = 10.0f;
	revDesc.flags |= NX_RJF_MOTOR_ENABLED;

	NxRevoluteJoint* revJoint = (NxRevoluteJoint *) _scene.createJoint(revDesc);
	printf("%p\n", revJoint);
}

void Robot::initKicker()
{
	NxBodyDesc bodyDesc;
	bodyDesc.mass = 0.01f;

	NxBoxShapeDesc shapeDesc;
	shapeDesc.dimensions.set(Robot::KickerLength / 2.0, Robot::KickerFaceWidth
	        / 2.0, Robot::KickerFaceHeight / 2.0f);
	NxActorDesc actorDesc;
	actorDesc.shapes.pushBack(&shapeDesc);
	actorDesc.body = &bodyDesc;

	actorDesc.globalPose.t = NxVec3(0, 0, .01);
	//actorDesc.globalPose.M.rotX(M_PI/2.0);

	if (!actorDesc.isValid())
	{
		printf("not valid\n");
		return;
	}

	_kicker = _scene.createActor(actorDesc);

	NxD6JointDesc kickJointDesc;

	kickJointDesc.actor[0] = _actor;
	kickJointDesc.actor[1] = _kicker;

	kickJointDesc.setGlobalAxis(NxVec3(1, 0, 0)); //The direction of the axis the bodies revolve around.
	kickJointDesc.setGlobalAnchor(NxVec3(0, 0, 0.01)); //Reference point that the axis passes through.

	kickJointDesc.twistMotion = NX_D6JOINT_MOTION_LOCKED;//Create a fixed joint.
	kickJointDesc.swing1Motion = NX_D6JOINT_MOTION_LOCKED;
	kickJointDesc.swing2Motion = NX_D6JOINT_MOTION_LOCKED;

	kickJointDesc.xMotion = NX_D6JOINT_MOTION_LIMITED;
	kickJointDesc.yMotion = NX_D6JOINT_MOTION_LOCKED;
	kickJointDesc.zMotion = NX_D6JOINT_MOTION_LOCKED;

	kickJointDesc.linearLimit.value = .06f;
	kickJointDesc.linearLimit.damping = 0;
	kickJointDesc.linearLimit.restitution = 0;

	float kickVel = .75 / 255.0 * 50.0;

	kickJointDesc.maxForce = 10.0f;
	kickJointDesc.driveLinearVelocity = NxVec3(kickVel, 0, 0);

	kickJointDesc.xDrive.forceLimit = 10.0f;
	kickJointDesc.xDrive.driveType = NX_D6JOINT_DRIVE_VELOCITY;

	NxD6Joint* kickerJoint = (NxD6Joint *) _scene.createJoint(kickJointDesc);
	printf("%p\n", kickerJoint);
}

void Robot::initWheels()
{
	NxVec3 loc[4];
	loc[0] = NxVec3(1, 1, 0);
	loc[1] = NxVec3(1, -1, 0);
	loc[2] = NxVec3(-1, -1, 0);
	loc[3] = NxVec3(-1, 1, 0);

	for (int i = 0; i < 4; ++i)
	{
		loc[i].x *= .055;
		loc[i].y *= .055;
		NxBodyDesc bodyDesc;
		bodyDesc.mass = 5.0f;

#if 1
		NxConvexShapeDesc shapeDesc;
		shapeDesc.meshData = cylinder(.01, .025, 20);
#else
		NxSphereShapeDesc shapeDesc;
		shapeDesc.radius = 0.0254;
#endif

		NxMaterialDesc materialDesc;
		materialDesc.restitution = 0;

		materialDesc.staticFriction = 0;
		materialDesc.dynamicFriction = 0;
		materialDesc.staticFrictionV = 0.9f;
		materialDesc.dynamicFrictionV = 0.7f;

		materialDesc.frictionCombineMode = NX_CM_AVERAGE;
		materialDesc.dirOfAnisotropy.set(1, 0, 0);
		materialDesc.flags = NX_MF_ANISOTROPIC;

		NxMaterial* newMaterial = _scene.createMaterial(materialDesc);
		shapeDesc.materialIndex = newMaterial->getMaterialIndex();

		NxActorDesc actorDesc;
		actorDesc.shapes.pushBack(&shapeDesc);
		actorDesc.body = &bodyDesc;

		NxVec3 eulerAngle = NxVec3(90, 0, 45);

		if (i % 2 == 0)
		{
			eulerAngle.z = -eulerAngle.z;
		}

		// Quaternion
		NxQuat q1, q2, q3;
		q1.fromAngleAxis(eulerAngle.x, NxVec3(1, 0, 0));
		q2.fromAngleAxis(eulerAngle.y, NxVec3(0, 1, 0));
		q3.fromAngleAxis(eulerAngle.z, NxVec3(0, 0, 1));

		NxQuat q;

		q = q3 * q2 * q1; // Use global axes

		actorDesc.globalPose.M.fromQuat(q);
		actorDesc.globalPose.t = loc[i];

		if (!actorDesc.isValid())
		{
			printf("not valid\n");
			return;
		}

		_wheels[i] = _scene.createActor(actorDesc);

		if (_kicker)
		{
			_scene.setActorPairFlags(*_wheels[i], *_kicker, NX_IGNORE_PAIR);
		}

		//_scene.setActorPairFlags(*_wheels[i], *_actor, NX_IGNORE_PAIR);

		NxRevoluteJointDesc revDesc;

		revDesc.actor[0] = _actor;
		revDesc.actor[1] = _wheels[i];

		NxVec3 axel = loc[i];
		axel.z = 0;

		revDesc.setGlobalAxis(axel); //The direction of the axis the bodies revolve around.
		revDesc.setGlobalAnchor(loc[i]); //Reference point that the axis passes through.

		revDesc.motor.maxForce = 0.05f;

		revDesc.motor.velTarget = 0;
		revDesc.flags |= NX_RJF_MOTOR_ENABLED;

		_motors[i] = (NxRevoluteJoint *) _scene.createJoint(revDesc);
	}
}

void Robot::step()
{
	if (_motors[0])
	{
		for (int i = 0; i < 4; ++i)
		{
			NxMotorDesc motorDesc;
			_motors[i]->getMotor(motorDesc);

			motorDesc.velTarget = 100.0f * vels[i] / 255.0f;
			printf("%f\n", motorDesc.velTarget);

			_motors[i]->setMotor(motorDesc);
		}
	}
}

NxConvexMesh* Robot::cylinder(const float length, const float radius,
        const unsigned int sides)
{
	static NxCookingInterface *gCooking = NxGetCookingLib(
	        NX_PHYSICS_SDK_VERSION);
	gCooking->NxInitCooking();

	const NxU32 vertCount = (sides + 1) * 2;
	const unsigned int offset = vertCount / 2;

	NxVec3 cyl[vertCount];

	const float increment = 2 * M_PI / sides;

	float rad = 0;
	float x = 0;
	float y = 0;
	for (unsigned int i = 0; i < offset; ++i)
	{
		cyl[i] = NxVec3(x, y, -length / 2.0);

		rad += increment;
		x = cos(rad) * radius;
		y = sin(rad) * radius;

		cyl[i + offset] = cyl[i];
		cyl[i + offset].z += length;
	}

	// Create descriptor for convex mesh
	NxConvexMeshDesc convexDesc;
	convexDesc.numVertices = vertCount;
	convexDesc.pointStrideBytes = sizeof(NxVec3);
	convexDesc.points = cyl;
	convexDesc.flags = NX_CF_COMPUTE_CONVEX;

	//MemoryWriteBuffer buf;
	MemoryStream buff;
	bool status = gCooking->NxCookConvexMesh(convexDesc, buff);

	if (!status)
	{
		printf("cooking failed\n");
		return 0;
	}

	return Env::_physicsSDK->createConvexMesh(buff);
}

void Robot::position(float x, float y)
{
	NxVec3 newPos(x, y, Constants::Robot::Height/2.0);
	NxVec3 delta = newPos - _actor->getGlobalPosition();

	for (int i=0 ; i<4 ; ++i)
	{
		NxVec3 wp = _wheels[i]->getGlobalPosition();
		wp.x += delta.x;
		wp.y += delta.y;

		_wheels[i]->setGlobalPosition(wp);
	}

	_actor->setGlobalPosition(newPos);
}

Point2d* Robot::getPosition()
{
    Point2d* currPos = new Point2d();
    currPos->x = (float)_actor->getGlobalPosition().x;
    currPos->y = (float)_actor->getGlobalPosition().y;

    return currPos;
}
