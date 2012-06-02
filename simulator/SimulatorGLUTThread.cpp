#include <stdio.h>

#include <physics/GlutCamera.hpp>
#include "physics/Robot.hpp"
#include <physics/Environment.hpp>
#include <SimulatorGLUTThread.hpp>
#include "physics/RobotBallController.hpp"

using namespace std;

// Hooks for GLUT

static SimulatorGLUTThread* gSimpleApplication = 0;

static void glutKeyboardCallback(unsigned char key, int x, int y) {
	gSimpleApplication->keyboardCallback(key, x, y);
}

static void glutKeyboardUpCallback(unsigned char key, int x, int y) {
	gSimpleApplication->keyboardUpCallback(key, x, y);
}

static void glutSpecialKeyboardCallback(int key, int x, int y) {
	gSimpleApplication->specialKeyboard(key, x, y);
}

static void glutSpecialKeyboardUpCallback(int key, int x, int y) {
	gSimpleApplication->specialKeyboardUp(key, x, y);
}

static void glutReshapeCallback(int w, int h) {
	gSimpleApplication->camera()->reshape(w, h);
}

static void glutMoveAndDisplayCallback() {
	gSimpleApplication->clientMoveAndDisplay();
}

static void glutDisplayCallback(void) {
	gSimpleApplication->camera()->displayCallback();
}

// SimulatorGLUTThread implementation

SimulatorGLUTThread::SimulatorGLUTThread(int argc, char* argv[], const QString& configFile, bool sendShared)
: _argv(argv), _argc(argc), _env(0), _vehicle(0),
  _camera(0), _cameraHeight(4.f),	_minCameraDistance(3.f), _maxCameraDistance(10.f)
{
	initialize(configFile, sendShared);
}

SimulatorGLUTThread::~SimulatorGLUTThread() {
	if (_env)
		delete _env;

	if (_simEngine)
		delete _simEngine;

	if (_vehicle)
		delete _vehicle;

	if (_camera)
		delete _camera;
}

void SimulatorGLUTThread::run() {

	// set up glut
	gSimpleApplication = this;
	int width = 800, height = 640;
	const char* title = "Robocup Simulator";
	glutInit(&_argc, _argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STENCIL);
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(width, height);
	glutCreateWindow(title);

	_camera->myinit();

	glutKeyboardFunc(glutKeyboardCallback);
	glutKeyboardUpFunc(glutKeyboardUpCallback);
	glutSpecialFunc(glutSpecialKeyboardCallback);
	glutSpecialUpFunc(glutSpecialKeyboardUpCallback);

	glutReshapeFunc(glutReshapeCallback);
	glutIdleFunc(glutMoveAndDisplayCallback);
	glutDisplayFunc(glutDisplayCallback);

	glutMoveAndDisplayCallback();

	// Actually start loop
	glutMainLoop();
}

void SimulatorGLUTThread::keyboardCallback(unsigned char key, int x, int y) {
	(void) x;
	(void) y;

	switch (key) {
	case 'q':
		exit(0);
		break;
	case 'h':	_simEngine->setDebug(btIDebugDraw::DBG_NoHelpText);	    break;
	case 'w':	_simEngine->setDebug(btIDebugDraw::DBG_DrawWireframe);  break;
	case 'p':	_simEngine->setDebug(btIDebugDraw::DBG_ProfileTimings);	break;
	case '=': {
		int maxSerializeBufferSize = 1024 * 1024 * 5;
		btDefaultSerializer* serializer = new btDefaultSerializer(
				maxSerializeBufferSize);
		//serializer->setSerializationFlags(BT_SERIALIZE_NO_DUPLICATE_ASSERT);
		_simEngine->dynamicsWorld()->serialize(serializer);
		FILE* f2 = fopen("testFile.bullet", "wb");
		fwrite(serializer->getBufferPointer(), serializer->getCurrentBufferSize(),
				1, f2);
		fclose(f2);
		delete serializer;
		break;
	}
	case 'm': _simEngine->setDebug(btIDebugDraw::DBG_EnableSatComparison);  break;
	case 'n':	_simEngine->setDebug(btIDebugDraw::DBG_DisableBulletLCP);		  break;
	case 't':	_simEngine->setDebug(btIDebugDraw::DBG_DrawText);		          break;
	case 'y':	_simEngine->setDebug(btIDebugDraw::DBG_DrawFeaturesText);		  break;
	case 'a':	_simEngine->setDebug(btIDebugDraw::DBG_DrawAabb);             break;
	case 'c':	_simEngine->setDebug(btIDebugDraw::DBG_DrawContactPoints);    break;
	case 'C':	_simEngine->setDebug(btIDebugDraw::DBG_DrawConstraints);      break;
	case 'L':	_simEngine->setDebug(btIDebugDraw::DBG_DrawConstraintLimits); break;

	//rotate/zoom camera
	case 'j': _camera->setAzi(_camera->getAzi() + 10); break;
	case 'l': _camera->setAzi(_camera->getAzi() - 10); break;
	case 'k': _camera->setEle(_camera->getEle() - 10); break;
	case 'i': _camera->setEle(_camera->getEle() + 10); break;
	case 'u': _camera->setCameraDistance(_camera->getCameraDistance()+.2); break;
	case 'o': _camera->setCameraDistance(_camera->getCameraDistance()-.2); break;

	//kick/chip
	case 'z': _vehicle->getRobotController()->prepareKick(255,false); break;
	case 'x': _vehicle->getRobotController()->prepareKick(255,true); break;

	case 'd':
		_simEngine->setDebug(btIDebugDraw::DBG_NoDeactivation);
		if (_simEngine->debugMode() & btIDebugDraw::DBG_NoDeactivation) {
			gDisableDeactivation = true;
		} else {
			gDisableDeactivation = false;
		}
		break;
	case 's':
		clientMoveAndDisplay();
		break;
		//    case ' ' : newRandom(); break;
	case ' ':
		clientResetScene();
		break;
	case '1':	_simEngine->setDebug(btIDebugDraw::DBG_EnableCCD); break;

	default:
		//        std::cout << "unused key : " << key << std::endl;
		break;
	}

	if (getDynamicsWorld() && getDynamicsWorld()->getDebugDrawer())
		getDynamicsWorld()->getDebugDrawer()->setDebugMode(_simEngine->debugMode());

}

btDynamicsWorld* SimulatorGLUTThread::getDynamicsWorld() {
	return _simEngine->dynamicsWorld();
}

int SimulatorGLUTThread::getDebugMode() const {
	return _simEngine->debugMode();
}

void SimulatorGLUTThread::setDebugMode(int mode) {
	_simEngine->debugMode(mode);
	if (getDynamicsWorld() && getDynamicsWorld()->getDebugDrawer())
		getDynamicsWorld()->getDebugDrawer()->setDebugMode(mode);
}

void SimulatorGLUTThread::initialize(const QString& configFile, bool sendShared) {

	// set up the simulation
	_simEngine = new SimEngine();
	_simEngine->initPhysics();

	// Set up environment
	_env = new Environment(configFile, sendShared, _simEngine);

	// Set up the vehicle
	_vehicle = new Robot(_env,0,Robot::rev2008,Geometry2d::Point(-1,0));
	_vehicle->initPhysics(false);

	// Set up the camera
	_camera = new GlutCamera(_simEngine);
	_camera->setCameraPosition(scaling*btVector3(1, 1, 1));//


	// Connect the debug drawer
	_simEngine->dynamicsWorld()->setDebugDrawer(_camera->debugDrawer());
}

void SimulatorGLUTThread::render() {
	updateCamera();

	btVector3 worldBoundsMin, worldBoundsMax;
	getDynamicsWorld()->getBroadphase()->getBroadphaseAabb(worldBoundsMin, worldBoundsMax);

	_vehicle->renderWheels(camera()->shapeDrawer(), worldBoundsMin, worldBoundsMax);

	_env->renderScene(camera()->shapeDrawer(), worldBoundsMin, worldBoundsMax);

	_camera->renderme(_simEngine->debugMode());
}

void SimulatorGLUTThread::clientMoveAndDisplay() {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//get delta
	float delta = _simEngine->getClock()->getTimeMicroseconds() * 0.000001f;

	// apply controls
	_vehicle->applyEngineForces(delta);

	// simulation steps
	_env->preStep(delta);
	_simEngine->stepSimulation();
	//_env->step();

//#define motion_debug 1
#ifdef motion_debug
	// print out current vel, ang vel, pos of vehicle
	btRigidBody* m_chassisBody = _vehicle->getRigidBody();

	btVector3 vel = m_chassisBody->getLinearVelocity();
	btVector3 ang = m_chassisBody->getAngularVelocity();
	btTransform pos;
	m_chassisBody->getMotionState()->getWorldTransform(pos);
	btVector3 trans = pos.getOrigin();

	printf("lin vel x: %8.4f y: %8.4f z: %8.4f \n", vel[0], vel[1], vel[2]);
	printf("ang vel x: %8.4f y: %8.4f z: %8.4f \n", ang[0], ang[1], ang[2]);
	printf("pos     x: %8.4f y: %8.4f z: %8.4f \n", trans[0], trans[1], trans[2]);

	trans = _vehicle->getRaycastVehicle()->m_wheelInfo[0].m_worldTransform.getOrigin();
	printf("wheel     x: %8.4f y: %8.4f z: %8.4f \n", trans[0], trans[1], trans[2]);

	printf("angle = %5.3f\n",_vehicle->getAngle());
	printf("position = (%5.3f,%5.3f)\n",_vehicle->getPosition().x,_vehicle->getPosition().y);
#endif

	render();

	//optional but useful: debug drawing
	_simEngine->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}

void SimulatorGLUTThread::displayCallback(void) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	render();

	//optional but useful: debug drawing
	_simEngine->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}

void SimulatorGLUTThread::clientResetScene() {
	if (_vehicle)
		_vehicle->resetScene();
}

void SimulatorGLUTThread::specialKeyboardUp(int key, int x, int y) {
	switch (key) {
	case GLUT_KEY_UP: {
		//_vehicle->setEngineForce(0.f);
		_vehicle->velocity(0,0,0);
		break;
	}
	case GLUT_KEY_DOWN: {
		//_vehicle->setBrakingForce(0.f);
		_vehicle->velocity(0,0,0);
		break;
	}
	case GLUT_KEY_LEFT: {
		//_vehicle->setEngineForce(0.f);
		_vehicle->velocity(0,0,0);
	}
	case GLUT_KEY_RIGHT: {
		//_vehicle->setEngineForce(0.f);
		_vehicle->velocity(0,0,0);
	}
	default:
		break;
	}
}

void SimulatorGLUTThread::specialKeyboard(int key, int x, int y) {
	//	printf("key = %i x=%i y=%i\n",key,x,y);
	switch (key) {
	case GLUT_KEY_LEFT: {
		//_vehicle->steerLeft();
		_vehicle->velocity(0,0,3);
		break;
	}
	case GLUT_KEY_RIGHT: {
		//_vehicle->steerRight();
		_vehicle->velocity(0,0,-3);
		break;
	}
	case GLUT_KEY_UP: {
		//_vehicle->driveForward();
		_vehicle->velocity(3,0,0);
		break;
	}
	case GLUT_KEY_DOWN: {
		//_vehicle->driveBackward();
		_vehicle->velocity(-3,0,0);
		break;
	}
	default:
		break;
	}
}

void SimulatorGLUTThread::updateCamera() {
	if (!_vehicle || !_camera) return;

	// calculate where the camera should be
	btTransform chassisWorldTrans;
	_vehicle->getWorldTransform(chassisWorldTrans);

	btVector3 targetPosition = chassisWorldTrans.getOrigin();
	btVector3 cameraPosition = camera()->getCameraPosition();

//#define camera_debug 1
#ifdef camera_debug
	printf("target  x: %8.4f y: %8.4f z: %8.4f \n",targetPosition[0],targetPosition[1],targetPosition[2]);
	printf("camera  x: %8.4f y: %8.4f z: %8.4f \n",cameraPosition[0],cameraPosition[1],cameraPosition[2]);
#endif

	//interpolate the camera height
	cameraPosition[1] = (15.0 * cameraPosition[1] + targetPosition[1] + _cameraHeight) / 16.0;

	btVector3 camToObject = targetPosition - cameraPosition;

	//keep distance between min and max distance
	float cameraDistance = camToObject.length();
	float correctionFactor = 0.f;
	if (cameraDistance < _minCameraDistance) {
		correctionFactor = 0.15 * (_minCameraDistance - cameraDistance)
						/ cameraDistance;
	}
	if (cameraDistance > _maxCameraDistance) {
		correctionFactor = 0.15 * (_maxCameraDistance - cameraDistance)
						/ cameraDistance;
	}
	cameraPosition -= correctionFactor * camToObject;

#ifdef camera_debug
	printf("new cam x: %8.4f y: %8.4f z: %8.4f \n",cameraPosition[0],cameraPosition[1],cameraPosition[2]);
#endif

	_camera->setCameraPosition(cameraPosition);
	_camera->setCameraTargetPosition(targetPosition);

	//_camera->updateFreeCamera();

	// rendering details
	_camera->updateCamera(); // default camera
//	_camera->chaseCamera(); // based on the original version

}




