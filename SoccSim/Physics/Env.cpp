#include "Env.hpp"
#include "Env.moc"
#include "Entity.hpp"
#include "Ball.hpp"
#include "Field.hpp"
#include "Robot.hpp"

#include <Constants.hpp>
#include <Team.h>
#include <Network/Network.hpp>
#include <QMutexLocker>
#include <boost/foreach.hpp>

NxPhysicsSDK* Env::_physicsSDK = 0;
unsigned int Env::_refCount = 0;

using namespace Geometry2d;

Env::Env()
{
    if (!_physicsSDK)
    {
		//initialize the PhysX SDK
		_physicsSDK = NxCreatePhysicsSDK(NX_PHYSICS_SDK_VERSION);
		if(!_physicsSDK)
		{
			printf("Wrong SDK DLL version\n");
			//TODO throw exception
		}

		NxReal myScale = 0.5f;
		_physicsSDK->setParameter(NX_VISUALIZATION_SCALE, myScale);
		//gPhysicsSDK->setParameter(NX_VISUALIZE_WORLD_AXES, 2.0f);
		_physicsSDK->setParameter(NX_VISUALIZE_COLLISION_SHAPES, 1.0f);
		//_physicsSDK->setParameter(NX_VISUALIZE_COLLISION_SKELETONS, 1.0f);
		//_physicsSDK->setParameter(NX_VISUALIZE_COLLISION_AABBS, 1.0f);
		//_physicsSDK->setParameter(NX_VISUALIZE_BODY_LIN_VELOCITY, 1.0f);

		_physicsSDK->setParameter(NX_VISUALIZE_JOINT_WORLD_AXES, 1.0f);
		//_physicsSDK->setParameter(NX_VISUALIZE_JOINT_LOCAL_AXES, 1.0f);
		//_physicsSDK->setParameter(NX_VISUALIZE_JOINT_LIMITS, 1.0f);

		_physicsSDK->setParameter(NX_SKIN_WIDTH, 0.001f); //depth of the two skins
		_physicsSDK->setParameter(NX_CONTINUOUS_CD, true);
		_physicsSDK->setParameter(NX_CCD_EPSILON, .001f);
    }

    //setup a new scene
    NxSceneDesc sceneDesc;
    sceneDesc.setToDefault();

    //setup gravity to world gravity
    sceneDesc.gravity.set(0, 0, -9.81);

    //set z-axis as up axis
    sceneDesc.upAxis = 2;

    //setup the bounds for the scene, use field bounds
    //TODO field bounds
    NxBounds3 bounds;
    bounds.set(-10, -10, 0, 10, 10, 10); //minx,y,z, maxx,y,z
    sceneDesc.maxBounds = &bounds;

    _scene = _physicsSDK->createScene(sceneDesc);
    if (!_scene)
    {
	    printf("Can't create scene!\n");
	    //TODO throw exception
    }

    //always add the field
    _field = new Field(this);

    //NxActor** actors = _scene->getActors();
    //NxU32 actorCount = _scene->getNbActors();

    //new environment created
    ++_refCount;

    //packetRxd = 0;
    //_receiver = new Network::PacketReceiver();
    //_receiver->addType(Network::Address, Network::addTeamOffset(Blue,Network::RadioTx), this, &Env::radioHandler);
    //txPacket = new Packet::RadioTx();

    connect(&_step, SIGNAL(timeout()), this, SLOT(step()));
}

Env::~Env()
{
    if (_scene)
    {
    	_physicsSDK->releaseScene(*_scene);
    	_scene = 0;
    }

    //teardown the SDK ... only do when refcount = 0;
    if (--_refCount == 0)
    {
    	_physicsSDK->release();
    	_physicsSDK = 0;
    }
}

const NxDebugRenderable& Env::dbgRenderable() const
{
	QMutexLocker ml(&_sceneMutex);
    return *_scene->getDebugRenderable();
}

void Env::step()
{
	//no new radio data while this is running
	//QMutexLocker ml(&_sceneMutex);
	_sceneMutex.lock();

    _scene->simulate(.005);
    _scene->flushStream();

    //can use old data here
	_sceneMutex.unlock();

    _scene->fetchResults(NX_RIGID_BODY_FINISHED, true);

    //generate some vision information with latest positions
    genVision();
    genRadio();
}

void Env::start()
{
	_step.start(5);
}

void Env::addBall(Geometry2d::Point pos)
{
	QMutexLocker ml1(&_sceneMutex);
	QMutexLocker ml2(&_entitiesMutex);

    Ball* b = new Ball(this);
    b->position(pos.x, pos.y);

    _balls.append(b);

    printf("New Ball: %f %f\n", pos.x, pos.y);
}

void Env::addRobot(Team t, int id, Geometry2d::Point pos)
{
	if (t == UnknownTeam)
	{
		printf("Cannot add robots to unknown team.\n");
		return;
	}

	QMutexLocker ml1(&_sceneMutex);
	QMutexLocker ml2(&_entitiesMutex);

    Robot* r = new Robot(this);
    r->position(pos.x, pos.y);

    if (t == Blue)
    {
    	_blue.append(r);
    }
    else
    {
    	_yellow.append(r);
    }

    printf("New Robot: %d : %f %f\n", id, pos.x, pos.y);
}

Packet::Vision Env::vision()
{
	//lock vision data mutex
	QMutexLocker ml(&_visionMutex);

	//return copy
	return _visionInfo;
}

Geometry2d::Point gaussianPoint(int n, float scale)
{
	Geometry2d::Point pt;
	for (int i = 0; i < n; ++i)
	{
		pt.x += drand48() - 0.5;
		pt.y += drand48() - 0.5;
	}
	pt *= scale / n;

	return pt;
}

bool Env::occluded(Geometry2d::Point ball, Geometry2d::Point camera)
{
	float camZ = 4;
	float ballZ = Constants::Ball::Radius;
	float intZ = Constants::Robot::Height;

	// Find where the line from the camera to the ball intersects the
	// plane at the top of the robots.
	//
	// intZ = (ballZ - camZ) * t + camZ
	float t = (intZ - camZ) / (ballZ - camZ);
	Geometry2d::Point intersection;
	intersection.x = (ball.x - camera.x) * t + camera.x;
	intersection.y = (ball.y - camera.y) * t + camera.y;

	// Return true if the intersection point is inside any robot
	BOOST_FOREACH(const Robot* r, _blue)
	{
		if (intersection.nearPoint(r->getPosition(), Constants::Robot::Radius))
		{
			return true;
		}
	}
	BOOST_FOREACH(const Robot* r, _yellow)
	{
		if (intersection.nearPoint(r->getPosition(), Constants::Robot::Radius))
		{
			return true;
		}
	}
	return false;
}

void Env::genVision()
{
	QMutexLocker ml(&_visionMutex);

	_visionInfo = Packet::Vision();

	//vision information is created by the environment because of the strict
	//rules on when scene information can be accessed. This prevents needless
	//locks on scene objects to generate vision info since a copy is returned

	int i=0;
	BOOST_FOREACH(const Robot* r, _blue)
	{
		Packet::Vision::Robot vr;
		vr.angle = r->getAngle();
		vr.pos = r->getPosition();// + gaussianPoint(25, 0.08);
		vr.shell = i++;

		_visionInfo.blue.push_back(vr);
	}

	i=0;
	BOOST_FOREACH(const Robot* r, _yellow)
	{
		Packet::Vision::Robot vr;
		vr.angle = r->getAngle();
		vr.pos = r->getPosition();// + gaussianPoint(25, 0.08);
		vr.shell = i++;

		_visionInfo.yellow.push_back(vr);
	}

	Geometry2d::Point cam0(-Constants::Field::Length / 4, 0);
	Geometry2d::Point cam1(Constants::Field::Length / 4, 0);

	BOOST_FOREACH(const Ball* b, _balls)
	{
		Geometry2d::Point ballPos = b->getPosition();

		bool occ;
		if (ballPos.x < 0)
		{
			occ = occluded(ballPos, cam0);
		} else {
			occ = occluded(ballPos, cam1);
		}

		if (!occ)
		{
			Packet::Vision::Ball vb;
			vb.pos = ballPos;// + gaussianPoint(25, 0.08);
			_visionInfo.balls.push_back(vb);
		}
	}
}

Packet::RadioRx Env::radio(Team t)
{
	QMutexLocker ml(&_radioRxMutex);

	if (t == Blue)
	{
		return _radioRxBlue;
	}
	else if (t == Yellow)
	{
		return _radioRxYellow;
	}

	return Packet::RadioRx();
}

void Env::radio(Team t, const Packet::RadioTx& data)
{
	//control robots when not working with the scene
	//this will prep new data for the next scene
	QMutexLocker ml(&_sceneMutex);

	for (int i=0 ; i< 5 ; ++i)
	{
		const Packet::RadioTx::Robot& r = data.robots[i];
		if (r.valid)
		{
			Robot* robot = 0;

			QVector<Robot*>* robots = &_blue;
			if (t == Yellow)
			{
				robots = &_yellow;
			}

			if (r.board_id < robots->size() && r.board_id >= 0)
			{
				robot = robots->at(r.board_id);
			}

			//if we found a robot to control
			if (robot)
			{
				robot->radio(r);
			}
		}
	}
}

void Env::command(const Packet::SimCommand &cmd)
{
    QMutexLocker ml(&_sceneMutex);

    if (cmd.ball.valid && !_balls.empty())
    {
        _balls[0]->velocity(cmd.ball.vel.x, cmd.ball.vel.y);
        _balls[0]->position(cmd.ball.pos.x, cmd.ball.pos.y);
    }
}

void Env::genRadio()
{
	//generate radio data every loop cycle
	//it will be pulled by radio when needed
	QMutexLocker ml(&_radioRxMutex);

	_radioRxBlue = Packet::RadioRx();
	_radioRxYellow = Packet::RadioRx();

	//first 5 robots generate radio data...?
	unsigned int count = 0;
	BOOST_FOREACH(const Robot* r, _blue)
	{
		_radioRxBlue.robots[count] = r->radio();

		if (++count >= 5)
		{
			break;
		}
	}

	count = 0;
	BOOST_FOREACH(const Robot* r, _yellow)
	{
		_radioRxYellow.robots[count] = r->radio();

		if (++count >= 5)
		{
			break;
		}
	}
}
