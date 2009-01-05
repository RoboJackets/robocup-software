#include "Env.hpp"
#include "Entity.hpp"
#include "Ball.hpp"
#include "Field.hpp"
#include "Robot.hpp"
#include <Team.h>
#include <Network/Network.hpp>

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
    _entities.append(new Field(*_scene));

    //NxActor** actors = _scene->getActors();
    //NxU32 actorCount = _scene->getNbActors();

    //new environment created
    ++_refCount;

    try
    {
	inputHandler = new InputHandler();

	printf("Using /dev/input/js0 for input.\n");
	//start handling controller input
	inputHandler->start();
    }
    catch (std::runtime_error err)
    {
	printf("No input controller.\n");
	inputHandler = 0;
    }

    txPacket = new Packet::RadioTx();

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
    return *_scene->getDebugRenderable();
}

void Env::step()
{

    int rid;
#if 0
    if (inputHandler)
    {
	Packet::CommData::Robot data = inputHandler->genRobotData();

	for (int i=0 ; i<4 ; ++i)
	{
	    _robots[0]->vels[i] = data.motor[i];
	}

	_robots[0]->step();
    }
#endif

    if(inputHandler)
    {
	rid = inputHandler->currentRobot();

	if(rid > _robots.size())
	{
	    rid = 0;
	}

	Packet::RadioTx::Robot data = inputHandler->genRobotData();

        for (int i=0 ; i<4 ; ++i)
	{
	    _robots[rid]->vels[i] = data.motors[i];
	}
	_robots[rid]->step();
    }
    else
    {
        int i = 0;

        Q_FOREACH(Robot* r, _robots)
        {
	    for (int k=0 ; k<4 ; ++k)
	    {
		r->vels[k] = txPacket->robots[i].motors[k];
//                 printf("Robot %d Wheel %d\n",i, txPacket->robots[i].motors[k]);
	    }
	    r->step();


            i++;
        }

    }
    _scene->simulate(1.0/60.0);
    _scene->flushStream();

    //can use old data here

    _scene->fetchResults(NX_RIGID_BODY_FINISHED, true);

    //send data out
}

void Env::start()
{
	_step.start(30);
}

void Env::addBall(float x, float y)
{
    //TODO lock mutex
    Ball* b = new Ball(*_scene);
    b->position(x, y);

    _entities.append(b);

    printf("New Ball: %f %f\n", x, y);
}

void Env::addRobot(int id, float x, float y)
{
    Robot* r = new Robot(*_scene);
    r->position(x, y);

    _entities.append(r);
    _robots.append(r);

    printf("New Robot: %d : %f %f\n", id, x, y);
}

QVector<Point2d*> Env::getRobotsPositions()
{
    QVector<Point2d*> botPositions;
    Q_FOREACH(Robot* r, _robots)
    {
        botPositions.append(r->getPosition());
    }

    return botPositions;
}

QVector<Point2d*> Env::getBallPositions()
{
    QVector<Point2d*> ballPositions;
    return ballPositions;
}
