#include <iostream>

#include <physics/GlutCamera.hpp>
#include <physics/GroundSurface.hpp>
#include <physics/SimpleVehicle.hpp>
#include <physics/SimEngine.hpp>
#include <physics/Environment.hpp>

#include <SimulatorThread.hpp>

using namespace std;

// Hooks for GLUT

static SimulatorThread* gSimpleApplication = 0;

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

// SimulatorThread implementation

SimulatorThread::SimulatorThread(int argc, char* argv[], const QString& configFile, bool sendShared)
: _argv(argv), _argc(argc), _env(new Environment(configFile, sendShared)), _vehicle(0), _ground(0),
  _camera(0), _cameraHeight(4.f),	_minCameraDistance(3.f), _maxCameraDistance(10.f)
{
}

SimulatorThread::~SimulatorThread() {
	if (_env)
		delete _env;

	if (_simEngine)
		delete _simEngine;

	if (_ground)
		delete _ground;

	if (_vehicle)
		delete _vehicle;

	if (_camera)
		delete _camera;
}

void SimulatorThread::run() {
//	_env->init();

	// Set up demo
//	_vehicleDemo = new SimulatorThread;
	initPhysics();

	// set up glut
	gSimpleApplication = this;
	int width = 640, height = 480;
	const char* title = "Bullet Vehicle Demo";
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

void SimulatorThread::keyboardCallback(unsigned char key, int x, int y) {
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

btDynamicsWorld* SimulatorThread::getDynamicsWorld() {
	return _simEngine->dynamicsWorld();
}

int SimulatorThread::getDebugMode() const {
	return _simEngine->debugMode();
}

void SimulatorThread::setDebugMode(int mode) {
	_simEngine->debugMode(mode);
	if (getDynamicsWorld() && getDynamicsWorld()->getDebugDrawer())
		getDynamicsWorld()->getDebugDrawer()->setDebugMode(mode);
}

void SimulatorThread::initPhysics() {

	// set up the simulation
	_simEngine = new SimEngine();
	_simEngine->initPhysics();

	// Set up the ground
	_ground = new GroundSurface(_simEngine);
	_ground->initPhysics();

	// Set up the vehicle
	_vehicle = new SimpleVehicle(_simEngine);
	_vehicle->initPhysics();

	// Set up the camera
	_camera = new GlutCamera(_simEngine);
	_camera->setCameraPosition(btVector3(30, 30, 30));
	_camera->setCameraDistance(26.f);

	// Connect the debug drawer
	_simEngine->dynamicsWorld()->setDebugDrawer(_camera->debugDrawer());
}

void SimulatorThread::render() {
	updateCamera();

	btVector3 worldBoundsMin, worldBoundsMax;
	getDynamicsWorld()->getBroadphase()->getBroadphaseAabb(worldBoundsMin, worldBoundsMax);

	_vehicle->drawWheels(camera()->shapeDrawer(), worldBoundsMin, worldBoundsMax);

	_camera->renderme(_simEngine->debugMode());
}

void SimulatorThread::clientMoveAndDisplay() {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// apply controls
	_vehicle->move();

	// simulation steps
	_simEngine->stepSimulation();
	_env->step();

	// print out the current position of the vehicle
	btTransform pose;
	_vehicle->getWorldTransform(pose);
	btVector3 trans = pose.getOrigin();
	cout << "Pose: x = " << trans.x() << " y = " << trans.y() << " z = " << trans.z() << endl;

	render();

	//optional but useful: debug drawing
	_simEngine->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}

void SimulatorThread::displayCallback(void) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	render();

	//optional but useful: debug drawing
	_simEngine->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}

void SimulatorThread::clientResetScene() {
	if (_vehicle)
		_vehicle->resetScene();
}

void SimulatorThread::specialKeyboardUp(int key, int x, int y) {
	switch (key) {
	case GLUT_KEY_UP: {
		_vehicle->engineForce(0.f);
		break;
	}
	case GLUT_KEY_DOWN: {
		_vehicle->breakingForce(0.f);
		break;
	}
	default:
		break;
	}
}

void SimulatorThread::specialKeyboard(int key, int x, int y) {
	//	printf("key = %i x=%i y=%i\n",key,x,y);
	switch (key) {
	case GLUT_KEY_LEFT: {
		_vehicle->steerLeft();
		break;
	}
	case GLUT_KEY_RIGHT: {
		_vehicle->steerRight();
		break;
	}
	case GLUT_KEY_UP: {
		_vehicle->driveForward();
		break;
	}
	case GLUT_KEY_DOWN: {
		_vehicle->driveBackward();
		break;
	}
	default:
		break;
	}
}

void SimulatorThread::updateCamera() {
	if (!_vehicle || !_camera) return;

	// calculate where the camera should be
	btTransform chassisWorldTrans;
	_vehicle->getWorldTransform(chassisWorldTrans);

	btVector3 targetPosition = chassisWorldTrans.getOrigin();
	btVector3 cameraPosition = camera()->getCameraPosition();

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

	_camera->setCameraPosition(cameraPosition);
	_camera->setCameraTargetPosition(targetPosition);

	// rendering details
	_camera->updateCamera(); // default camera
//	_camera->chaseCamera(); // based on the original version
}


