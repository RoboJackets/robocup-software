#include "Robot.hpp"
#include "Ball.hpp"
#include "Env.hpp"
#include "MemoryStream.hpp"

#include <NxCooking.h>

#include <Utils.hpp>
#include <Constants.hpp>

#include <boost/foreach.hpp>

#define ROLLER  1
#define KICKER  0
#define CHIPPER 0

using namespace Geometry2d;

Geometry2d::Point toPoint(const NxVec3 &v)
{
    return Geometry2d::Point(v.x, v.y);
}

Robot::Robot(Env* env, unsigned int id,  Robot::Rev rev) :
	Entity(env), shell(id), _rev(rev), _lastKicked(0)
{
    _kickerJoint = 0;
    _rollerJoint = 0;
    
	//create entity
	NxBodyDesc bodyDesc;
	bodyDesc.mass = 5.0f;

	//TODO fixme, use real shell mesh
	#if 1
	NxConvexShapeDesc shapeDesc;
	shapeDesc.meshData = cylinder(Robot_Height,
	        Robot_Radius, 20);
	
	shapeDesc.localPose.t = NxVec3(0.0f, 0.0f, Robot_Height/2.0);
	#endif
	//NxBoxShapeDesc shapeDesc;
	//shapeDesc.dimensions.set(0.2f,0.2f,0.2f);
	//shapeDesc.localPose.t = NxVec3(0.0f, 0.0f, .2);

	//setup the ball actor
	NxActorDesc actorDesc;
	actorDesc.globalPose.t = NxVec3(0.0f, 0.0f, 0.0f);
	
	NxMaterialDesc materialDesc;
	materialDesc.restitution = 0;

	materialDesc.staticFriction = .15;
	materialDesc.dynamicFriction = .11;
	
	NxMaterial* newMaterial = _scene.createMaterial(materialDesc);
	shapeDesc.materialIndex = newMaterial->getMaterialIndex();
	
	actorDesc.shapes.pushBack(&shapeDesc);
	actorDesc.body = &bodyDesc;
	
	//this creates an actor in the scene
	_actor = _scene.createActor(actorDesc);
	assert(_actor);

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

	//initWheels();
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

	assert(actorDesc.isValid());

	_roller = _scene.createActor(actorDesc);

	//setup a joint to the roller
	NxRevoluteJointDesc revDesc;

	revDesc.actor[0] = _actor;
	revDesc.actor[1] = _roller;

	revDesc.setGlobalAxis(NxVec3(0, 1, 0)); //The direction of the axis the bodies revolve around.
	revDesc.setGlobalAnchor(NxVec3(RollerOffset, 0, RollerHeight)); //Reference point that the axis passes through.

	revDesc.motor.maxForce = 5.0f;
	revDesc.motor.velTarget = 0; //10.0f;
	revDesc.flags |= NX_RJF_MOTOR_ENABLED;

	_rollerJoint = (NxRevoluteJoint *) _scene.createJoint(revDesc);
	assert(_rollerJoint);
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

	actorDesc.globalPose.t = NxVec3(0.05, 0, .01);
	//actorDesc.globalPose.M.rotX(M_PI/2.0);
	assert(actorDesc.isValid());

	_kicker = _scene.createActor(actorDesc);

	NxD6JointDesc kickJointDesc;

	kickJointDesc.actor[0] = _actor;
	kickJointDesc.actor[1] = _kicker;

	kickJointDesc.setGlobalAxis(NxVec3(1, 0, 0));           //The direction of the axis the bodies move along
	kickJointDesc.setGlobalAnchor(actorDesc.globalPose.t);  // Center of the range of motion

	kickJointDesc.twistMotion = NX_D6JOINT_MOTION_LOCKED;
	kickJointDesc.swing1Motion = NX_D6JOINT_MOTION_LOCKED;
	kickJointDesc.swing2Motion = NX_D6JOINT_MOTION_LOCKED;

	kickJointDesc.xMotion = NX_D6JOINT_MOTION_LIMITED;
	kickJointDesc.yMotion = NX_D6JOINT_MOTION_LOCKED;
	kickJointDesc.zMotion = NX_D6JOINT_MOTION_LOCKED;

	kickJointDesc.linearLimit.value = .04f;
	kickJointDesc.linearLimit.damping = 0;
	kickJointDesc.linearLimit.restitution = 0;

	kickJointDesc.maxForce = 10.0f;

	kickJointDesc.xDrive.forceLimit = 10.0f;
	kickJointDesc.xDrive.driveType = NX_D6JOINT_DRIVE_VELOCITY;

	_kickerJoint = (NxD6Joint *) _scene.createJoint(kickJointDesc);
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
		bodyDesc.mass = 0.5f;

#if 1
		NxConvexShapeDesc shapeDesc;
		shapeDesc.meshData = cylinder(.01, .0254, 48);
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
		assert(actorDesc.isValid());

		_wheels[i] = _scene.createActor(actorDesc);

		if (_kicker)
		{
			_scene.setActorPairFlags(*_wheels[i], *_kicker, NX_IGNORE_PAIR);
		}

		//_scene.setActorPairFlags(*_wheels[i], *_actor, NX_IGNORE_PAIR);

		NxRevoluteJointDesc revDesc;

		revDesc.actor[0] = _actor;
		revDesc.actor[1] = _wheels[i];

		NxVec3 axle = loc[i];
		axle.z = 0;

		revDesc.setGlobalAxis(axle); //The direction of the axis the bodies revolve around.
		revDesc.setGlobalAnchor(loc[i]); //Reference point that the axis passes through.

		revDesc.motor.maxForce = 0.05f;

		revDesc.motor.velTarget = 0;
		revDesc.flags |= NX_RJF_MOTOR_ENABLED;

		_motors[i] = (NxRevoluteJoint *) _scene.createJoint(revDesc);
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

	const float increment = 330 / sides;

	float angle = 30;
	float x = 0;
	float y = 0;
	for (unsigned int i = 0; i < offset; ++i)
	{
		cyl[i] = NxVec3(x, y, -length / 2.0);

		x = cos(angle * M_PI / 180.0) * radius;
		y = sin(angle * M_PI / 180.0) * radius;
        angle += increment;

		cyl[i + offset] = cyl[i];
		cyl[i + offset].z += length;
	}

	// Create descriptor for convex mesh
	NxConvexMeshDesc convexDesc;
	convexDesc.numVertices = vertCount;
	convexDesc.pointStrideBytes = sizeof(NxVec3);
	convexDesc.points = cyl;
	convexDesc.flags = NX_CF_COMPUTE_CONVEX;

	MemoryStream buff;
	assert(gCooking->NxCookConvexMesh(convexDesc, buff));

	return _env->_physicsSDK->createConvexMesh(buff);
}

void Robot::position(float x, float y)
{
	NxVec3 newPos(x, y, Robot_Height/2.0);
#if 0
    NxVec3 delta = newPos - _actor->getGlobalPosition();
	for (int i=0 ; i<4 ; ++i)
	{
		NxVec3 wp = _wheels[i]->getGlobalPosition();
		wp.x += delta.x;
		wp.y += delta.y;

		_wheels[i]->setGlobalPosition(wp);
	}
#endif
	_actor->setGlobalPosition(newPos);
    _roller->setGlobalPosition(newPos + NxVec3(Robot::RollerOffset, 0, Robot::RollerHeight));
}

void Robot::velocity(float x, float y, float w)
{
    _actor->setLinearVelocity(NxVec3(x, y, 0));
    
    // Need to do this because the body may get an unexpected linear velocity from a collision (even with the ground)
    _actor->setAngularVelocity(NxVec3(0, 0, w));
}

float Robot::getAngle() const
{
	NxVec3 c1 = _actor->getGlobalOrientation().getColumn(0);
	return atan2(c1[1] , c1[0]) * 180.0f / M_PI;
}

void Robot::radioTx(const Packet::RadioTx::Robot *data)
{
	//motors
	#if 0
	for (int i = 0; i < 4; ++i)
	{
		NxMotorDesc motorDesc;
		_motors[i]->getMotor(motorDesc);
		
		//create a velocity target based on the requested travel velocity
		motorDesc.velTarget = 100.0f * data.motors[i] / 255.0f;

		_motors[i]->setMotor(motorDesc);
	}
	#endif
	
	Geometry2d::Point axles[4] =
	{
		Point( .08,  .08),
		Point( .08, -.08),
		Point(-.08, -.08),
		Point(-.08,  .08)
	};
	
	if (_rollerJoint)
	{
		NxMotorDesc motorDesc;
		_rollerJoint->getMotor(motorDesc);
		motorDesc.velTarget = data->roller() * 10.0f / 255.0f;
		_rollerJoint->setMotor(motorDesc);
	}
	
	for (unsigned int i = 0 ; i < 4 ; ++i)
	{
		Point wheel(-axles[i].y, axles[i].x);
		wheel = wheel.normalized();
		wheel.rotate(Point(), getAngle());
		
		NxVec3 p(axles[i].x, axles[i].y, 0);
		
		NxVec3 vp = _actor->getLocalPointVelocity(p);
		Point vel(vp.x, vp.y);
		
		float current = vel.dot(wheel);
		

		float target = data->motors(i) / 127.0f * 1.2;
		
		// reverse for 2010 robots
		if (_rev == rev2010)
		{
			target = -target;
		}
		
		const float diff = target - current;
		
		const float force = 20.0f * diff;
		
		const float fx = force * wheel.x;
		const float fy = force * wheel.y;
		
		NxVec3 f(fx,fy,0);
		_actor->addForceAtLocalPos(f, p);
	}
    
    // Kicker
#if 0
    if (_kickerJoint)
    {
        float kick = data.kick;
        float v;
        if (kick)
        {
            v = kick * 10.0f / 255.0f;
        } else {
            v = -1;
        }
        _kickerJoint->setDriveLinearVelocity(NxVec3(v, 0, 0));
    }
#else

    /** How we kick:
     * Kick speed will be zeroed if we are not kicking
     * Otherwise we determine which direction we are kicking and kick that way, using
     * max speeds guessed with science
     */

    if (data->kick() && Utils::timestamp() - _lastKicked > RechargeTime)
    {
    	// FIXME: make these parameters some place else
    	float maxKickSpeed = 5.0f, // m/s direct kicking speed
    		  maxChipSpeed = 3.0f, // m/s chip kicking at the upwards angle
    		  chipAngle = 20.0f;   // angle (degrees) of upwards chip

    	// determine the kick speed
    	float kickSpeed;
    	bool chip = data->use_chipper() && _rev == rev2010;
    	if (chip)
		{
    		kickSpeed = data->kick() / 255.0f * maxChipSpeed;
		} else {
    		kickSpeed = data->kick() / 255.0f * maxKickSpeed;
		}

        // construct a velocity to apply
        NxVec3 kickVel = _actor->getGlobalOrientation().getColumn(0);
        if (chip) {
        	kickVel.setz(cos(chipAngle));
        	kickVel.normalize();
        }

        kickVel *= kickSpeed;
        BOOST_FOREACH(Ball *ball, _env->balls())
        {
            if (ballSense(ball))
            {
            	if (chip)
            		printf("Robot %d chip %p by %f: %f, %f, %f\n", shell, ball, kickSpeed, kickVel.x, kickVel.y, kickVel.z);
            	else
            		printf("Robot %d kick %p by %f: %f, %f, %f\n", shell, ball, kickSpeed, kickVel.x, kickVel.y, kickVel.z);

                ball->actor()->addForce(kickVel, NX_VELOCITY_CHANGE);
                _lastKicked = Utils::timestamp();
            }
        }
    }
#endif
}

Packet::RadioRx Robot::radioRx() const
{
	Packet::RadioRx packet;
	
	packet.set_timestamp(Utils::timestamp());
	packet.set_battery(1.0f);
	packet.set_rssi(1.0f);
	packet.set_charged(Utils::timestamp() - _lastKicked > RechargeTime);
	
	BOOST_FOREACH(const Ball* ball, _env->balls())
	{
		if (ballSense(ball))
		{
			packet.set_ball(true);
		}
	}
	
	return packet;
}

bool Robot::ballSense(const Ball *ball) const
{
	const float halfKickerFOV = 30 * M_PI / 180.0f;
	NxVec3 kickerMax(cos(halfKickerFOV), sin(halfKickerFOV), 0);
	NxVec3 kickerMin(kickerMax.x, -kickerMax.y, 0);

	// convert orientation to actual global space
	NxMat33 orientation = _actor->getGlobalOrientation();
	kickerMin = orientation * kickerMin;
	kickerMax = orientation * kickerMax;
	
	Geometry2d::Point pos = getPosition();
	Geometry2d::Point ballPos = ball->getPosition();
	bool near = ballPos.nearPoint(pos, Robot_Radius + Ball_Radius);
	
	if (!near)
	{
		return false;
	}
	
	// FIXME - This works as long as the robot is flat on the ground.
	//         A sensor object would be better.
	NxVec3 ballRel = ball->actor()->getGlobalPosition() - _actor->getGlobalPosition();
	NxVec3 cmin, cmax;
	cmin.cross(kickerMin, ballRel);
	cmax.cross(ballRel, kickerMax);
	
	return cmin.z > 0 && cmax.z > 0;
}
