#pragma once

#include <QThread>

#include <boost/shared_ptr.hpp>

/**
 * Create a new thread to act as a wrapper for the simulation
 */
class SimulatorThread : public QThread {
protected:
	char ** argv_;
	int argc_;

public:
	typedef boost::shared_ptr<SimulatorThread> shared_ptr;

	/** need to pass arguments through to glut */
	SimulatorThread(int argc, char* argv[]);

private:
	// Re-implement the run function to start the process
	void run();

}; // \class SimulatorThread




