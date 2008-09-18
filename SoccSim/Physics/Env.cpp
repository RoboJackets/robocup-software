#include "Env.hpp"

#include "Entity.hpp"
#include "Ball.hpp"
#include "Field.hpp"
#include "Robot.hpp"

NxPhysicsSDK* Env::_physicsSDK = 0;
unsigned int Env::_refCount = 0;

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
		
		NxReal myScale = 1.0f;
		_physicsSDK->setParameter(NX_VISUALIZATION_SCALE, myScale);
		//gPhysicsSDK->setParameter(NX_VISUALIZE_WORLD_AXES, 2.0f);
		_physicsSDK->setParameter(NX_VISUALIZE_COLLISION_SHAPES, 1.0f);
		//_physicsSDK->setParameter(NX_VISUALIZE_COLLISION_AABBS, 1.0f);
		//_physicsSDK->setParameter(NX_VISUALIZE_BODY_LIN_VELOCITY, 1.0f);
		
		_physicsSDK->setParameter(NX_VISUALIZE_JOINT_WORLD_AXES, 1.0f);
		_physicsSDK->setParameter(NX_VISUALIZE_JOINT_LOCAL_AXES, 1.0f);
		_physicsSDK->setParameter(NX_VISUALIZE_JOINT_LIMITS, 1.0f);
		
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
#if 1
	NxBounds3 bounds;
	bounds.set(-10, -10, 0, 10, 10, 10); //minx,y,z, maxx,y,z
	sceneDesc.maxBounds = &bounds;
#endif

	_scene = _physicsSDK->createScene(sceneDesc);
	if (!_scene)
	{
		printf("Can't create scene!\n");
		//TODO throw exception
	}
	
	//creates a new ball in the scene
	_entities.append(new Ball(*_scene));
	_entities.append(new Field(*_scene));
	_entities.append(new Robot(*_scene));
	
	//NxActor** actors = _scene->getActors();
	//NxU32 actorCount = _scene->getNbActors();
	
	//new environment created
	++_refCount;
}

Env::~Env()
{
	//_scene->releaseActor(*actor);
	//actor = 0;
	
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

void Env::redraw() const
{
    Q_FOREACH(Entity* e, _entities)
    {
        e->paint();
    }
}

void Env::step()
{
	_scene->simulate(1.0/60.0);
	_scene->flushStream();

	//render the scene
	//const NxDebugRenderable *dbgRenderable=gScene->getDebugRenderable();
	//renderData(dbgRenderable);

	//can use old data here

	_scene->fetchResults(NX_RIGID_BODY_FINISHED, true);
}

void Env::run()
{
    
}
