#include "Env.hpp"
#include "Env.moc"
#include "Entity.hpp"
#include "Ball.hpp"
#include "Field.hpp"
#include "Robot.hpp"

#include <Team.h>
#include <Network/Network.hpp>
#include <QMutexLocker>

NxPhysicsSDK* Env::_physicsSDK = 0;
unsigned int Env::_refCount = 0;

using namespace Geometry;

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
	//_physicsSDK->setParameter(NX_VISUALIZE_COLLISION_AABBS, 1.0f);
	//_physicsSDK->setParameter(NX_VISUALIZE_BODY_LIN_VELOCITY, 1.0f);

	_physicsSDK->setParameter(NX_VISUALIZE_JOINT_WORLD_AXES, 1.0f);
	//_physicsSDK->setParameter(NX_VISUALIZE_JOINT_LOCAL_AXES, 1.0f);
	//_physicsSDK->setParameter(NX_VISUALIZE_JOINT_LIMITS, 1.0f);

	_physicsSDK->setParameter(NX_SKIN_WIDTH, 0.0005f); //depth of the two skins
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
    _field = new Field(*_scene);

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
	QMutexLocker ml(&_sceneMutex);
	
    _scene->simulate(1.0/60.0);
    _scene->flushStream();

    //can use old data here

    _scene->fetchResults(NX_RIGID_BODY_FINISHED, true);

    //generate some vision information with latest positions
    genVision();
    genRadio();
}

void Env::start()
{
	_step.start(30);
}

void Env::addBall(Geometry::Point2d pos)
{
	QMutexLocker ml1(&_sceneMutex);
	QMutexLocker ml2(&_entitiesMutex);
	
    Ball* b = new Ball(*_scene);
    b->position(pos.x, pos.y);

    _balls.append(b);

    printf("New Ball: %f %f\n", pos.x, pos.y);
}

void Env::addRobot(Team t, int id, Geometry::Point2d pos)
{
	if (t == UnknownTeam)
	{
		printf("Cannot add robots to unknown team.\n");
		return;
	}
	
	QMutexLocker ml1(&_sceneMutex);
	QMutexLocker ml2(&_entitiesMutex);
	
    Robot* r = new Robot(*_scene);
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

void Env::genVision()
{
	QMutexLocker ml(&_visionMutex);
	
	_visionInfo = Packet::Vision();
	
	//vision information is created by the environment because of the strict
	//rules on when scene information can be accessed. This prevents needless
	//locks on scene objects to generate vision info since a copy is returned
	
	int i=0;
	Q_FOREACH(const Robot* r, _blue)
	{
		Packet::Vision::Robot vr;
		vr.angle = r->getAngle();
		vr.pos = r->getPosition();
		vr.shell = i++; 
		
		_visionInfo.blue.push_back(vr);
	}
	
	i=0;
	Q_FOREACH(const Robot* r, _yellow)
	{
		Packet::Vision::Robot vr;
		vr.angle = r->getAngle();
		vr.pos = r->getPosition();
		vr.shell = i++; 
		
		_visionInfo.yellow.push_back(vr);
	}
	
	Q_FOREACH(const Ball* b, _balls)
	{
		Packet::Vision::Ball vb;
		vb.pos = b->getPosition();
		
		_visionInfo.balls.push_back(vb);
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

void Env::radio(Team t, Packet::RadioTx& data)
{
	//control robots when not working with the scene
	//this will prep new data for the next scene
	QMutexLocker ml(&_sceneMutex);
	
	for (int i=0 ; i< 5 ; ++i)
	{
		Packet::RadioTx::Robot& r = data.robots[i];
		if (r.valid)
		{
			Robot* robot = 0;
			//need to get robot with given id (i)
			if (t == Blue)
			{
				
			}
			else if (t == Yellow)
			{
				
			}
			
			//if we found a robot to control
			if (robot)
			{
				
			}
		}
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
	/*
	Q_FOREACH(const Robot* r, _blue)
	{
		//_radioRxBlue.robots
	}
	*/
	//TODO gen outgoing data
}

/*
const QVector<const Robot*>& Env::getRobots(Team t)
{	
	if (t == Blue)
	{
		return *((const QVector<const Robot*> *) &_blue);
	}
	
	return *((const QVector<const Robot*> *) &_yellow);
}

const QVector<const Ball*>& Env::getBalls()
{
	return *((const QVector<const Ball*> *) &_balls);
}
*/
