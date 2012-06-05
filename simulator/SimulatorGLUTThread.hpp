#pragma once

#include <map>

#include <QThread>

#include <boost/shared_ptr.hpp>

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
	char ** _argv;
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
	typedef boost::shared_ptr<SimulatorGLUTThread> shared_ptr;

	/** need to pass arguments through to glut */
	SimulatorGLUTThread(int argc, char* argv[], const QString& configFile, bool sendShared);

	~SimulatorGLUTThread();

	/** access environment */
	Environment* env() { return _env; }

private:
	// Re-implement the run function to start the process
	void run();

public:

	btDynamicsWorld* getDynamicsWorld();

	void setDrawClusters(bool drawClusters) {}

	int getDebugMode() const;

	GlutCamera* camera() { return _camera; }

	void setDebugMode(int mode);

	void clientMoveAndDisplay();

	void clientResetScene();

	void displayCallback();

	void updateCamera();

	void nextVehicle();

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

}; // \class SimulatorGLUTThread




