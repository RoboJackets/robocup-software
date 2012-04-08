#pragma once

#include <QThread>

#include <boost/shared_ptr.hpp>

#include <physics/Environment.hpp>

/**
 * Create a new thread to act as a wrapper for the simulation
 */
class SimulatorThread : public QThread {
	Q_OBJECT;

protected:
	char ** _argv;
	int _argc;

	Environment* _env;

public:
	typedef boost::shared_ptr<SimulatorThread> shared_ptr;

	/** need to pass arguments through to glut */
	SimulatorThread(int argc, char* argv[], const QString& configFile, bool sendShared);

	~SimulatorThread();

	/** access environment */
	Environment* env() { return _env; }

private:
	// Re-implement the run function to start the process
	void run();

}; // \class SimulatorThread




