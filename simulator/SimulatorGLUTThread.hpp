#pragma once

#include <map>
#include <memory>

#include <QThread>

#include <physics/SimEngine.hpp>

class btCollisionShape;
class btDynamicsWorld;

class Robot;
class GlutCamera;

class Environment;

/**
 * Create a new thread to act as a wrapper for the simulation
 */
class SimulatorGLUTThread : public QThread {
    Q_OBJECT;

protected:
    char** _argv;
    int _argc;

    Environment* _env;

    // Drivable vehicle
    Robot* _vehicle;
    bool _blue;

    // Dynamics/Collision Environment parts
    SimEngine* _simEngine;

    // Camera components
    GlutCamera* _camera;

    // Additional camera components
    float _cameraHeight;
    float _minCameraDistance;
    float _maxCameraDistance;

public:
    typedef std::shared_ptr<SimulatorGLUTThread> shared_ptr;

    /** need to pass arguments through to glut */
    SimulatorGLUTThread(int argc, char* argv[], const QString& configFile,
                        bool sendShared, bool showWindow = true);

    ~SimulatorGLUTThread();

    /** access environment */
    Environment* env() { return _env; }

    void stop();

private:
    // Re-implement the run function to start the process
    void run() override;

    QMutex _mutex;
    bool _stopped;

public:
    btDynamicsWorld* getDynamicsWorld();

    void stepSimulation();

    void setDrawClusters(bool drawClusters) {}

    int getDebugMode() const;

    GlutCamera* camera() { return _camera; }

    void setDebugMode(int mode);

    void clientMoveAndDisplay();

    void clientResetScene();

    void displayCallback();

    void updateCamera();

    void nextVehicle();

    void displayProfileString(int xOffset, int yStart, char* message);

    void showVehicleInfo(int& xOffset, int& yStart, int yIncr);

    void specialKeyboard(int key, int x, int y);

    void specialKeyboardUp(int key, int x, int y);

    void render();

    void initialize(const QString& configFile, bool sendShared);

    /// glut callbacks

    void keyboardCallback(unsigned char key, int x, int y);

    void keyboardUpCallback(unsigned char key, int x, int y) {}

    // physics initializations for objects
    std::pair<btCollisionShape*, btTransform> addGround();

    void addVehicle(btDynamicsWorld* m_dynamicsWorld,
                    btAlignedObjectArray<btCollisionShape*>& m_collisionShapes);

    bool _showWindow;

};  // \class SimulatorGLUTThread
