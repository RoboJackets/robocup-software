#include <iostream>

//#include <BulletDynamics/btBulletDynamicsCommon.h>
#include <btBulletDynamicsCommon.h>

int main(void) {

	// world setup
	btBroadphaseInterface* broadphase = new btDbvtBroadphase();

	btDefaultCollisionConfiguration* collisionConfiguration =
			new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(
			collisionConfiguration);

	btSequentialImpulseConstraintSolver* solver =
			new btSequentialImpulseConstraintSolver;

	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld( // FAIL: runtime malloc error
			dispatcher, broadphase, solver, collisionConfiguration);

//	dynamicsWorld->setGravity(btVector3(0, -10, 0));

//	// add shapes
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1); // FAIL: linking error
//	btCollisionShape* fallShape = new btSphereShape(1);

//	btDefaultMotionState* groundMotionState = new btDefaultMotionState(
//			btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
//	btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,
//			groundMotionState, groundShape, btVector3(0, 0, 0));
//	btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
//	dynamicsWorld->addRigidBody(groundRigidBody);

//	btDefaultMotionState* fallMotionState = new btDefaultMotionState(
//			btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 50, 0)));
//	btScalar mass = 1;
//	btVector3 fallInertia(0, 0, 0);
//	fallShape->calculateLocalInertia(mass, fallInertia);
//	btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,
//			fallMotionState, fallShape, fallInertia);
//	btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
//	dynamicsWorld->addRigidBody(fallRigidBody);
//
//	for (int i = 0; i < 300; i++) {
//		dynamicsWorld->stepSimulation(1 / 60.f, 10);
//
//		btTransform trans;
//		fallRigidBody->getMotionState()->getWorldTransform(trans);
//
//		std::cout << "sphere height: " << trans.getOrigin().getY() << std::endl;
//	}
//
//	dynamicsWorld->removeRigidBody(fallRigidBody);
//	delete fallRigidBody->getMotionState();
//	delete fallRigidBody;

//	dynamicsWorld->removeRigidBody(groundRigidBody);
//	delete groundRigidBody->getMotionState();
//	delete groundRigidBody;

//	delete fallShape;

//	delete groundShape;

	// cleanup world
	delete dynamicsWorld;
	delete solver;
	delete collisionConfiguration;
	delete dispatcher;
	delete broadphase;

	return 7;
}

