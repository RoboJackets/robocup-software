#include "Robot.hpp"

#include <Geometry/Point2d.hpp>

using namespace Physics;
using namespace Geometry;

const float Robot::RollerLength = .07f;
const float Robot::RollerDiameter = .02f;
const float Robot::RollerHeight = .032f; //TODO fixme
const float Robot::RollerDist = .065f;

const float Robot::WheelDiameter = .05f;
const float Robot::WheelThickness = .01f;

const float Robot::KickerLength = 0.05f;
const float Robot::KickerWidth = 0.05f;
const float Robot::KickerThickness = .02f;
const float Robot::KickerTravel = RollerDist + .015f;
const float Robot::KickerDist = 0;
const float Robot::KickerHeight = .023f; //from ground

#define KICKER  1
#define ROLLER  1
#define BODY    0

Robot::Robot(dWorldID world, dSpaceID space, unsigned int id) :
	Entity(world, space, 3), _id(id), _ballPresent(false)
{
	_geomID = dCreateCylinder(space, ROBOT_RADIUS, .15f);
	dGeomSetBody(_geomID, _bodyID);
	dGeomSetData(_geomID, (void*)Entity::RobotShell);
	dBodySetData(_bodyID, this);

	dMassSetCylinderTotal(&_mass, 5, 3, ROBOT_RADIUS, .15); //mass, direction, radius, height
	dBodySetMass(_bodyID, &_mass);

	//dBodySetPosition(_bodyID, 0, 0, .5);

	dGeomSetOffsetPosition(_geomID, 0, 0, .15/2.0f);

	//dMatrix3 rot;
	//dRFromAxisAndAngle(rot, 0, 0, 1, M_PI*.5);
	//dBodySetRotation(_bodyID, rot);

#if ROLLER
	_rollerGeomID = dCreateCylinder(space, RollerDiameter/2.0f, RollerLength);
	dGeomSetData(_rollerGeomID, (void*)Entity::Roller);

	_rollerBodyID = dBodyCreate(world);
	dGeomSetBody(_rollerGeomID, _rollerBodyID);
	dBodySetData(_rollerBodyID, this);

	dMassSetCylinderTotal(&_rollerMass, .02, 2, .09, .15);
	dBodySetMass(_rollerBodyID, &_rollerMass);
	dGeomSetOffsetPosition(_rollerGeomID, 0, 0, RollerLength/2.0f);

	dMatrix3 rot;
	dRFromAxisAndAngle(rot, 0, 1, 0, M_PI/2.0);
	dRFromAxisAndAngle(rot, 1, 0, 0, M_PI/2.0);

	dGeomSetRotation(_rollerGeomID, rot);
	dGeomSetPosition(_rollerGeomID, RollerDist, 0, RollerHeight);

	/// set joint settings for roller
	joint = dJointCreateHinge(world, 0);

	dJointAttach(joint, _bodyID, _rollerBodyID);
	dJointSetHingeAnchor(joint, RollerDist, 0, RollerHeight);
	dJointSetHingeAxis(joint, 0, 1, 0);

	jointM = dJointCreateAMotor(world, 0);
	dJointAttach(jointM, _bodyID, _rollerBodyID);

	dJointSetAMotorNumAxes(jointM, 1);
	dJointSetAMotorAxis(jointM, 0, 1, 0, 1, 0);

	dJointSetAMotorParam(jointM, dParamFMax, 20);
	dJointSetAMotorParam(jointM, dParamVel, 0);
#endif

#if KICKER
	_kickerGeomID = dCreateBox(space, KickerLength, KickerWidth, KickerThickness);
	dGeomSetData(_kickerGeomID, (void*)Entity::Kicker);

	_kickerBodyID = dBodyCreate(world);
	dGeomSetBody(_kickerGeomID, _kickerBodyID);

	dMass kickerMass;
	dMassSetBoxTotal(&kickerMass, 0.01, KickerLength, KickerWidth, KickerThickness);
	dBodySetMass(_kickerBodyID, &kickerMass);

	dGeomSetPosition(_kickerGeomID, KickerDist, 0, KickerHeight);

	jointK = dJointCreateSlider(world, 0);
	dJointAttach(jointK, _bodyID, _kickerBodyID);
	dJointSetSliderAxis(jointK, -1, 0, 0);

	dJointSetSliderParam(jointK, dParamLoStop, 0);
	dJointSetSliderParam(jointK, dParamHiStop, KickerTravel);
	dJointSetSliderParam(jointK, dParamFMax, 5);
	
	_kickCount = 0;
	_kickState = Ready;
	_charged = true;
#endif

	//stop the motors
	memset(_motors, 0, sizeof(_motors));

	_kick = false;
    _quadric = gluNewQuadric();
}

Robot::~Robot()
{
    gluDeleteQuadric(_quadric);
}

void Robot::setPosition(float x, float y, float z)
{
#if KICKER || ROLLER
	Point2d oldPos = pos();
	float dx = x - oldPos.x, dy = y - oldPos.y;
#endif

	//TODO maybe??
	//disable the bodies
	//set positions

	dBodySetPosition(_bodyID, x, y, z);

#if KICKER
	const dReal* kp = dBodyGetPosition(_kickerBodyID);
	dBodySetPosition(_kickerBodyID, kp[0]+dx, kp[1]+dy, kp[2]);
#endif

#if ROLLER
	//const dReal* rp = dBodyGetPosition(_rollerBodyID);
	//dBodySetPosition(_rollerBodyID, rp[0]+dx, rp[1]+dy, rp[2]);
	const dReal* rp = dGeomGetPosition(_rollerGeomID);
	dGeomSetPosition(_rollerGeomID, rp[0]+dx, rp[1]+dy, rp[2]);
#endif
}

float Robot::theta() const
{
	//http://www.gamedev.net/community/forums/topic.asp?topic_id=403277
	const dReal* matrix = dBodyGetRotation(_bodyID);

	float kz, ky;
	if (matrix[2] <= 1 && matrix[2] >= -1)
	{
		ky = -asin(matrix[2]);
		dReal c = cos(ky);
		kz = atan2(matrix[1]/c, matrix[0]/c);
	}
	else
	{
		kz = 0;
		//ky = -atan2(matrix[2], 0);
		//kx = atan2(-matrix[9], matrix[5]);
	}

	return -kz*180.0f/M_PI;
}

void Robot::internal()
{
	//reset ball sense, collision will turn it back on
	ballPresent(false);
	
#if KICKER
	static unsigned int KickCount = 10;
	
	switch (_kickState)
	{
		case Ready:
			if (_kick)
			{
				_kickState = Kicking;
			}
			else
			{
				break;
			}
		case Kicking:
			if (_kickCount < KickCount)
			{
				//kick
				dJointSetSliderParam(jointK, dParamFMax, 20 * _kick/255.0);
				dJointSetSliderParam(jointK, dParamVel, 2);
				_kickCount++;
			}	
			else
			{
				//finished kicking
				_kickState = Recharging;
				_charged = false;
			}
			break;
		case Recharging:
			if (_kickCount > 0)
			{
				//retract the kicker
				dJointSetSliderParam(jointK, dParamFMax, 10);
				dJointSetSliderParam(jointK, dParamVel, -1);
				_kickCount--;
			}
			else
			{
				//indicate charged
				_kickState = Ready;
				_charged = true;
			}
			break;
	}
#endif
	
#if ROLLER
	dJointSetAMotorParam(jointM, dParamVel, -30 * _roller/255.0);
#endif

	/*wheel axels, wheel is assumed perp to axel
	 * /3  0\
	 *
	 * \2  1/
	 */
	Point2d axels[4] =
	{
		Point2d( 1,  1),
		Point2d( 1, -1),
		Point2d(-1, -1),
		Point2d(-1,  1)
	};

	for (unsigned int i=0 ; i<4 ; ++i)
	{
		axels[i].rotate(Point2d(0,0), -90);

		Point2d wheel(axels[i].y, -axels[i].x);
		wheel = wheel.norm();

		const float force = .1 * _motors[i];
		const float fx = force * wheel.x;
		const float fy = force * wheel.y;
		dBodyAddRelForceAtRelPos(_bodyID, fx, fy, 0, axels[i].x, axels[i].y, 0);
	}
}

void Robot::motors(int8_t m1, int8_t m2, int8_t m3, int8_t m4)
{
	_motors[0] = m1;
	_motors[1] = m2;
	_motors[2] = m3;
	_motors[3] = m4;
}

void Robot::paint()
{
	const dReal* pos = dBodyGetPosition(_bodyID);

	glColor3ub(0, 0, 0);

	glColor3ub(0xff, 0xff, 0xff);

	const dReal* r = dBodyGetRotation(_bodyID);
	float m[16];
	m[ 0] = r[ 0]; m[ 1] = r[ 4]; m[ 2] = r[ 8]; m[ 3] = 0;
	m[ 4] = r[ 1]; m[ 5] = r[ 5]; m[ 6] = r[ 9]; m[ 7] = 0;
	m[ 8] = r[ 2]; m[ 9] = r[ 6]; m[10] = r[10]; m[11] = 0;
	m[12] = pos[ 0]; m[13] = pos[ 1]; m[14] = pos[ 2] ;m[15] = 1;

	glPushMatrix();
		glMultMatrixf(m);

		//glDrawElements(GL_TRIANGLES, sizeof(cap)/sizeof(int), GL_UNSIGNED_INT, cap);
		//glDrawElements(GL_QUADS, sizeof(indices)/sizeof(int), GL_UNSIGNED_INT, indices);
		//glDrawElements(GL_QUADS, sizeof(indices2)/sizeof(int), GL_UNSIGNED_INT, indices2);

		//glColor3ub(0xff, 0xff, 0xff);
		glBegin(GL_LINES);
			glVertex3f(0, 0, .2);
			glVertex3f(.09, 0,.2);
		glEnd();

	glPopMatrix();

	glColor3ub(0xff, 0xff, 0xff);

#if BODY
	float radius, length;
	const dReal* p = dBodyGetPosition(_bodyID);
	dGeomCylinderGetParams(_geomID, &radius, &length);

	//dGeomGetRotation returns a 4x3 matrix in row major order (3 rows, 4 cols)
	//MultMatrix needs column major order
	//const dReal*
	r = dGeomGetRotation(_geomID);
	//float m[16];
	m[ 0] = r[ 0]; m[ 1] = r[ 4]; m[ 2] = r[ 8]; m[ 3] = 0;
	m[ 4] = r[ 1]; m[ 5] = r[ 5]; m[ 6] = r[ 9]; m[ 7] = 0;
	m[ 8] = r[ 2]; m[ 9] = r[ 6]; m[10] = r[10]; m[11] = 0;
	m[12] = p[ 0]; m[13] = p[ 1]; m[14] = p[ 2] ;m[15] = 1;

	glPushMatrix();
			glMultMatrixf(m);
			gluCylinder(_quadric, radius, radius, length, 10, 1); //radius, slices, stacks
	glPopMatrix();
#endif

#if KICKER
	glPushMatrix();
	{
			const dReal* p = dBodyGetPosition(_kickerBodyID);
			const dReal* r = dGeomGetRotation(_kickerGeomID);
			float m[16];
			m[ 0] = r[ 0]; m[ 1] = r[ 4]; m[ 2] = r[ 8]; m[ 3] = 0;
			m[ 4] = r[ 1]; m[ 5] = r[ 5]; m[ 6] = r[ 9]; m[ 7] = 0;
			m[ 8] = r[ 2]; m[ 9] = r[ 6]; m[10] = r[10]; m[11] = 0;
			m[12] = p[ 0]; m[13] = p[ 1]; m[14] = p[ 2] ;m[15] = 1;

			glMultMatrixf(m);

			glBegin(GL_QUADS);
				glVertex3f( KickerLength/2.0f,  KickerWidth/2.0f, 0);
				glVertex3f(-KickerLength/2.0f,  KickerWidth/2.0f, 0);
				glVertex3f(-KickerLength/2.0f, -KickerWidth/2.0f, 0);
				glVertex3f( KickerLength/2.0f, -KickerWidth/2.0f, 0);
			glEnd();
	}
	glPopMatrix();
#endif

#if ROLLER
	float radius, length;
	const dReal* p = dGeomGetPosition(_rollerGeomID);
	dGeomCylinderGetParams(_rollerGeomID, &radius, &length);

	//dGeomGetRotation returns a 4x3 matrix in row major order (3 rows, 4 cols)
	//MultMatrix needs column major order
	//const dReal*
	r = dGeomGetRotation(_rollerGeomID);
	//float m[16];
	m[ 0] = r[ 0]; m[ 1] = r[ 4]; m[ 2] = r[ 8]; m[ 3] = 0;
	m[ 4] = r[ 1]; m[ 5] = r[ 5]; m[ 6] = r[ 9]; m[ 7] = 0;
	m[ 8] = r[ 2]; m[ 9] = r[ 6]; m[10] = r[10]; m[11] = 0;
	m[12] = p[ 0]; m[13] = p[ 1]; m[14] = p[ 2] ;m[15] = 1;

	glPushMatrix();
			glMultMatrixf(m);
			gluCylinder(_quadric, radius, radius, length/2.0, 10, 1); //radius, slices, stacks
	glPopMatrix();
#endif
}
