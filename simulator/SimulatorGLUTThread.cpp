#include <stdio.h>

#include <physics/GlutCamera.hpp>
#include "physics/Robot.hpp"
#include <physics/Environment.hpp>
#include <SimulatorGLUTThread.hpp>
#include "physics/RobotBallController.hpp"
#include <GLDebugFont.h>
#include <GL/freeglut.h>

using namespace std;

// Hooks for GLUT

static SimulatorGLUTThread* gSimpleApplication = nullptr;

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

SimulatorGLUTThread::SimulatorGLUTThread(int argc, char* argv[],
                                         const QString& configFile,
                                         bool sendShared, bool showWindow)
    : _argv(argv),
      _argc(argc),
      _env(nullptr),
      _vehicle(nullptr),
      _blue(false),
      _camera(nullptr),
      _cameraHeight(4.f),
      _minCameraDistance(3.f),
      _maxCameraDistance(10.f),
      _showWindow(showWindow),
      _stopped(false) {
    initialize(configFile, sendShared);
}

SimulatorGLUTThread::~SimulatorGLUTThread() {
    if (_env) delete _env;

    if (_simEngine) delete _simEngine;

    if (_vehicle) delete _vehicle;

    if (_camera) delete _camera;
}

void SimulatorGLUTThread::run() {
    if (_showWindow) {
        // set up glut
        gSimpleApplication = this;
        int width = 800, height = 640;
        const char* title = "Robocup Simulator";
        glutInit(&_argc, _argv);
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH |
                            GLUT_STENCIL);
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
    } else {
        while (true) {
            if (_stopped) return;
            stepSimulation();
            usleep(16667);  //	wait 1/60 seconds
        }
    }
}

void SimulatorGLUTThread::stop() {
    QMutexLocker locker(&_mutex);
    //	in headless mode, we check this variable to stop nicely
    _stopped = true;
    if (_showWindow) {
        glutLeaveMainLoop();
    }
}

static bool DisplayMotion = false;
static bool DisplayState = true;

void SimulatorGLUTThread::keyboardCallback(unsigned char key, int x, int y) {
    (void)x;
    (void)y;

    switch (key) {
        case 'q':
            exit(0);
            break;
        case 'h':
            _simEngine->setDebug(btIDebugDraw::DBG_NoHelpText);
            break;
        case 'w':
            _simEngine->setDebug(btIDebugDraw::DBG_DrawWireframe);
            break;
        case 'p':
            _simEngine->setDebug(btIDebugDraw::DBG_ProfileTimings);
            break;
        case '=': {
            int maxSerializeBufferSize = 1024 * 1024 * 5;
            btDefaultSerializer* serializer =
                new btDefaultSerializer(maxSerializeBufferSize);
            // serializer->setSerializationFlags(BT_SERIALIZE_NO_DUPLICATE_ASSERT);
            _simEngine->dynamicsWorld()->serialize(serializer);
            FILE* f2 = fopen("testFile.bullet", "wb");
            fwrite(serializer->getBufferPointer(),
                   serializer->getCurrentBufferSize(), 1, f2);
            fclose(f2);
            delete serializer;
            break;
        }
        case 'm':
            _simEngine->setDebug(btIDebugDraw::DBG_EnableSatComparison);
            break;
        case 'n':
            _simEngine->setDebug(btIDebugDraw::DBG_DisableBulletLCP);
            break;
        case 't':
            _simEngine->setDebug(btIDebugDraw::DBG_DrawText);
            break;
        case 'y':
            _simEngine->setDebug(btIDebugDraw::DBG_DrawFeaturesText);
            break;
        case 'a':
            _simEngine->setDebug(btIDebugDraw::DBG_DrawAabb);
            break;
        //	case 'c':
        //_simEngine->setDebug(btIDebugDraw::DBG_DrawContactPoints);    break;
        case 'C':
            _simEngine->setDebug(btIDebugDraw::DBG_DrawConstraints);
            break;
        case 'L':
            _simEngine->setDebug(btIDebugDraw::DBG_DrawConstraintLimits);
            break;

        // camera mode
        case '1':
            _camera->setCameraMode(GlutCamera::SideLine);
            break;
        case '2':
            _camera->setCameraMode(GlutCamera::Overhead);
            break;
        case '3':
            _camera->setCameraMode(GlutCamera::BehindYellowGoal);
            break;
        case '4':
            _camera->setCameraMode(GlutCamera::BehindBlueGoal);
            break;
        case '5':
            _camera->setCameraMode(GlutCamera::TrackVehicle);
            _camera->setRobot(_vehicle);
            break;
        case '6':
            _camera->setCameraMode(GlutCamera::TrackVehicle |
                                   GlutCamera::BehindVehicle);
            break;
        case '7':
            _camera->setCameraMode(GlutCamera::TrackVehicle |
                                   GlutCamera::FrontOfVehicle);
            break;
        case '\t':
            nextVehicle();
            _camera->setRobot(_vehicle);
            break;

        // rotate/zoom camera
        case 'j':
            _camera->setAzi(_camera->getAzi() + 10);
            break;
        case 'l':
            _camera->setAzi(_camera->getAzi() - 10);
            break;
        case 'k':
            _camera->setEle(_camera->getEle() - 10);
            break;
        case 'i':
            _camera->setEle(_camera->getEle() + 10);
            break;
        case 'u':
            _camera->setCameraDistance(_camera->getCameraDistance() + .2);
            break;
        case 'o':
            _camera->setCameraDistance(_camera->getCameraDistance() - .2);
            break;

        // kick/chip
        case 'z':
            if (_vehicle)
                _vehicle->getRobotController()->prepareKick(255, false);
            break;
        case 'x':
            if (_vehicle)
                _vehicle->getRobotController()->prepareKick(255, true);
            break;
        case 'c':
            if (_vehicle)
                _vehicle->getRobotController()->prepareDribbler(
                    _vehicle->getRobotController()->getDribblePower() > 0
                        ? 0
                        : 127);
            break;

        // debug
        case 'b':
            DisplayState = DisplayState ? false : true;
            break;
        case 'v':
            DisplayMotion = DisplayMotion ? false : true;
            break;

        case '.':
            (*(_env->balls().begin()))->velocity(0, 0.1);
            break;

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
        //	case '1':	_simEngine->setDebug(btIDebugDraw::DBG_EnableCCD);
        //break;

        default:
            //        std::cout << "unused key : " << key << std::endl;
            break;
    }

    if (getDynamicsWorld() && getDynamicsWorld()->getDebugDrawer())
        getDynamicsWorld()->getDebugDrawer()->setDebugMode(
            _simEngine->debugMode());
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

void SimulatorGLUTThread::initialize(const QString& configFile,
                                     bool sendShared) {
    // set up the simulation
    _simEngine = new SimEngine();
    _simEngine->initPhysics();

    // Set up environment
    _env = new Environment(configFile, sendShared, _simEngine);

    _vehicle = _env->yellow().begin().value();

    // Set up the camera
    if (_showWindow) {
        _camera = new GlutCamera(_simEngine);
        _camera->setCameraPosition(scaling * btVector3(1, 1, 1));

        // Connect the debug drawer
        _simEngine->dynamicsWorld()->setDebugDrawer(_camera->debugDrawer());
    }
}

void SimulatorGLUTThread::stepSimulation() {
    // get delta time
    float delta = _simEngine->getClock()->getTimeMicroseconds() * 0.000001f;

    // simulation steps
    _env->preStep(delta);
    _simEngine->stepSimulation();
}

void SimulatorGLUTThread::render() {
    updateCamera();

    btVector3 worldBoundsMin, worldBoundsMax;
    getDynamicsWorld()->getBroadphase()->getBroadphaseAabb(worldBoundsMin,
                                                           worldBoundsMax);

    _env->renderScene(camera()->shapeDrawer(), worldBoundsMin, worldBoundsMax);

    _camera->renderme(_simEngine->debugMode());
}

void SimulatorGLUTThread::clientMoveAndDisplay() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    stepSimulation();

    render();

//#define motion_debug 1
#ifdef motion_debug
    printf("Robot %d\n", _vehicle->shell);

    // print out current vel, ang vel, pos of vehicle
    btRigidBody* m_chassisBody = _vehicle->getRigidBody();

    btVector3 vel = m_chassisBody->getLinearVelocity();
    btVector3 ang = m_chassisBody->getAngularVelocity();
    btTransform pos;
    m_chassisBody->getMotionState()->getWorldTransform(pos);
    btVector3 trans = pos.getOrigin();

    printf("lin vel x: %8.4f y: %8.4f z: %8.4f \n", vel[0], vel[1], vel[2]);
    printf("ang vel x: %8.4f y: %8.4f z: %8.4f \n", ang[0], ang[1], ang[2]);
    printf("pos     x: %8.4f y: %8.4f z: %8.4f \n", trans[0], trans[1],
           trans[2]);

    trans = _vehicle->getRaycastVehicle()
                ->m_wheelInfo[0]
                .m_worldTransform.getOrigin();
    printf("wheel     x: %8.4f y: %8.4f z: %8.4f \n", trans[0], trans[1],
           trans[2]);

    printf("angle = %5.3f\n", _vehicle->getAngle());
    printf("position = (%5.3f,%5.3f)\n", _vehicle->getPosition().x,
           _vehicle->getPosition().y);
#else
    if (_vehicle) {
        int xOffset = 10;
        int yStart = 20;
        int yIncr = 20;
        showVehicleInfo(xOffset, yStart, yIncr);
    }
#endif

    // optional but useful: debug drawing
    _simEngine->debugDrawWorld();

    glFlush();
    glutSwapBuffers();
}

void SimulatorGLUTThread::displayCallback(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    render();

    // optional but useful: debug drawing
    _simEngine->debugDrawWorld();

    glFlush();
    glutSwapBuffers();
}

void SimulatorGLUTThread::clientResetScene() {
    if (_vehicle) _vehicle->resetScene();
    if (_env) _env->resetScene();
}

void SimulatorGLUTThread::specialKeyboardUp(int key, int x, int y) {
    switch (key) {
        case GLUT_KEY_UP: {
            if (_vehicle) _vehicle->velocity(0, 0, 0);
            break;
        }
        case GLUT_KEY_DOWN: {
            if (_vehicle) _vehicle->velocity(0, 0, 0);
            break;
        }
        case GLUT_KEY_LEFT: {
            if (_vehicle) _vehicle->velocity(0, 0, 0);
            break;
        }
        case GLUT_KEY_RIGHT: {
            if (_vehicle) _vehicle->velocity(0, 0, 0);
            break;
        }
        default:
            break;
    }
}

void SimulatorGLUTThread::specialKeyboard(int key, int x, int y) {
    //	printf("key = %i x=%i y=%i\n",key,x,y);
    switch (key) {
        case GLUT_KEY_LEFT: {
            if (_vehicle) _vehicle->velocity(0, 0, 3);
            break;
        }
        case GLUT_KEY_RIGHT: {
            if (_vehicle) _vehicle->velocity(0, 0, -3);
            break;
        }
        case GLUT_KEY_UP: {
            if (_vehicle) _vehicle->velocity(3, 0, 0);
            break;
        }
        case GLUT_KEY_DOWN: {
            if (_vehicle) _vehicle->velocity(-3, 0, 0);
            break;
        }
        default:
            break;
    }
}

void SimulatorGLUTThread::updateCamera() {
    if (!_camera) return;
    _camera->updateCamera();  // default camera
}

void SimulatorGLUTThread::nextVehicle() {
    if (_vehicle) {
        Environment::RobotMap map;
        if (_blue)
            map = _env->blue();
        else
            map = _env->yellow();

        Environment::RobotMap::const_iterator i = map.find(_vehicle->shell);
        ++i;
        if (i != map.end())
            _vehicle = i.value();
        else
            _vehicle = map.begin().value();
    }
}

void SimulatorGLUTThread::displayProfileString(int xOffset, int yStart,
                                               char* message) {
    glRasterPos3f(btScalar(xOffset), btScalar(yStart), btScalar(0));
    //	GLDebugDrawString(xOffset,yStart,message);
    GLDebugDrawStringInternal(xOffset, yStart, message, btVector3(1, 1, 1),
                              true, 10);
}

void SimulatorGLUTThread::showVehicleInfo(int& xOffset, int& yStart,
                                          int yIncr) {
    if (_simEngine->debugMode() & btIDebugDraw::DBG_NoHelpText) return;

    char robotText[128];

    {
        sprintf(robotText, "---                 Robot %2d                  ---",
                _vehicle->shell);
        displayProfileString(xOffset, yStart, robotText);
        yStart += yIncr;

        sprintf(robotText,
                "press v-motion, b-state, h-hide, zxc-kick/chip/dribble, "
                "arrows-move");
        displayProfileString(xOffset, yStart, robotText);
        yStart += yIncr;

        /*sprintf(robotText,"coordinate system is in field space");
        displayProfileString(xOffset,yStart,robotText);
        yStart += yIncr;*/
    }

    if (DisplayMotion) {
        sprintf(robotText, "linear velocity = (%+7.5f,%+7.5f)",
                _vehicle->getVelFS().x, _vehicle->getVelFS().y);
        displayProfileString(xOffset, yStart, robotText);
        yStart += yIncr;

        sprintf(robotText, "angular velocity = %+7.5f",
                _vehicle->getAngVelFS());
        displayProfileString(xOffset, yStart, robotText);
        yStart += yIncr;
    }

    if (DisplayState) {
        /*sprintf(robotText,"---                 State %2d
        ---", _vehicle->shell);
        displayProfileString(xOffset,yStart,robotText);
        yStart += yIncr;*/

        sprintf(robotText, "target linear = (% 7.5f,% 7.5f)",
                _vehicle->getTargetVelFS().x, _vehicle->getTargetVelFS().y);
        displayProfileString(xOffset, yStart, robotText);
        yStart += yIncr;

        sprintf(robotText, "target angular = % 7.5f",
                _vehicle->getTargetAngVelFS());
        displayProfileString(xOffset, yStart, robotText);
        yStart += yIncr;

        sprintf(robotText, "%s %s, power = %lu",
                _vehicle->getRobotController()->chipEnabled() ? "chip" : "kick",
                _vehicle->getRobotController()->getKickerStatus() ? "ready"
                                                                  : "wait",
                _vehicle->getRobotController()->getKickPower());
        displayProfileString(xOffset, yStart, robotText);
        yStart += yIncr;

        sprintf(robotText, "ball sense = %s, dribbler = %lu",
                _vehicle->getRobotController()->hasBall() ? "true" : "false",
                _vehicle->getRobotController()->getDribblePower());
        displayProfileString(xOffset, yStart, robotText);
        yStart += yIncr;
    }

    sprintf(robotText, "-------------------------------------------------");
    displayProfileString(xOffset, yStart, robotText);
    yStart += yIncr;
}
