
#include <gameplay/GameplayModule.hpp>
#include <Constants.hpp>
#include <protobuf/LogFrame.pb.h>
#include <Robot.hpp>
#include <SystemState.hpp>

#include <stdio.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

//	for python stuff
#include "robocup-py.hpp"

using namespace std;
using namespace boost;
using namespace boost::python;

using namespace Geometry2d;



Gameplay::GameplayModule::GameplayModule(SystemState *state):
	_mutex(QMutex::Recursive)
{
	_state = state;

	_centerMatrix = Geometry2d::TransformMatrix::translate(Geometry2d::Point(0, Field_Length / 2));
	_oppMatrix = Geometry2d::TransformMatrix::translate(Geometry2d::Point(0, Field_Length)) *
				Geometry2d::TransformMatrix::rotate(180);

	//// Make an obstacle to cover the opponent's half of the field except for one robot diameter across the center line.
	Polygon *sidePolygon = new Polygon;
	_sideObstacle = std::shared_ptr<Shape>(sidePolygon);
	float x = Field_Width / 2 + Field_Border;
	const float y1 = Field_Length / 2;
	const float y2 = Field_Length + Field_Border;
	const float r = Field_CenterRadius;
	sidePolygon->vertices.push_back(Geometry2d::Point(-x, y1));
	sidePolygon->vertices.push_back(Geometry2d::Point(-r, y1));
	sidePolygon->vertices.push_back(Geometry2d::Point(0, y1 + r));
	sidePolygon->vertices.push_back(Geometry2d::Point(r, y1));
	sidePolygon->vertices.push_back(Geometry2d::Point(x, y1));
	sidePolygon->vertices.push_back(Geometry2d::Point(x, y2));
	sidePolygon->vertices.push_back(Geometry2d::Point(-x, y2));

	float y = -Field_Border;
	float deadspace = Field_Border;
	x = Field_Width/2.0f;
	Polygon* floorObstacle = new Polygon;
	floorObstacle->vertices.push_back(Geometry2d::Point(-x, y));
	floorObstacle->vertices.push_back(Geometry2d::Point(-x, y-1));
	floorObstacle->vertices.push_back(Geometry2d::Point(x, y-1));
	floorObstacle->vertices.push_back(Geometry2d::Point(x, y));
	_nonFloor[0] = std::shared_ptr<Shape>(floorObstacle);

	y = Field_Length + Field_Border;
	floorObstacle = new Polygon;
	floorObstacle->vertices.push_back(Geometry2d::Point(-x, y));
	floorObstacle->vertices.push_back(Geometry2d::Point(-x, y+1));
	floorObstacle->vertices.push_back(Geometry2d::Point(x, y+1));
	floorObstacle->vertices.push_back(Geometry2d::Point(x, y));
	_nonFloor[1] = std::shared_ptr<Shape>(floorObstacle);

	y = Field_Length;
	floorObstacle = new Polygon;
	floorObstacle->vertices.push_back(Geometry2d::Point(-x, -deadspace));
	floorObstacle->vertices.push_back(Geometry2d::Point(-x-1, -deadspace));
	floorObstacle->vertices.push_back(Geometry2d::Point(-x-1, y));
	floorObstacle->vertices.push_back(Geometry2d::Point(-x, y));
	_nonFloor[2] = std::shared_ptr<Shape>(floorObstacle);

	floorObstacle = new Polygon;
	floorObstacle->vertices.push_back(Geometry2d::Point(x, -deadspace));
	floorObstacle->vertices.push_back(Geometry2d::Point(x+1, -deadspace));
	floorObstacle->vertices.push_back(Geometry2d::Point(x+1, y));
	floorObstacle->vertices.push_back(Geometry2d::Point(x, y));
	_nonFloor[3] = std::shared_ptr<Shape>(floorObstacle);

	Polygon* goalArea = new Polygon;
	const float halfFlat = Field_GoalFlat/2.0;
	const float radius = Field_ArcRadius;
	goalArea->vertices.push_back(Geometry2d::Point(-halfFlat, 0));
	goalArea->vertices.push_back(Geometry2d::Point(-halfFlat, radius));
	goalArea->vertices.push_back(Geometry2d::Point( halfFlat, radius));
	goalArea->vertices.push_back(Geometry2d::Point( halfFlat, 0));
	_goalArea.add(std::shared_ptr<Shape>(goalArea));
	_goalArea.add(std::shared_ptr<Shape>(new Circle(Geometry2d::Point(-halfFlat, 0), radius)));
	_goalArea.add(std::shared_ptr<Shape>(new Circle(Geometry2d::Point(halfFlat, 0), radius)));

	_ourHalf = std::make_shared<Polygon>();
	_ourHalf->vertices.push_back(Geometry2d::Point(-x, -Field_Border));
	_ourHalf->vertices.push_back(Geometry2d::Point(-x, y1));
	_ourHalf->vertices.push_back(Geometry2d::Point(x, y1));
	_ourHalf->vertices.push_back(Geometry2d::Point(x, -Field_Border));

	_opponentHalf = std::make_shared<Polygon>();
	_opponentHalf->vertices.push_back(Geometry2d::Point(-x, y1));
	_opponentHalf->vertices.push_back(Geometry2d::Point(-x, y2));
	_opponentHalf->vertices.push_back(Geometry2d::Point(x, y2));
	_opponentHalf->vertices.push_back(Geometry2d::Point(x, y1));

	_goalieID = -1;



	//
	//	setup python interpreter
	//
	try {
        cout << "Initializing embedded python interpreter..." << endl;
        
        //  this tells python how to load the robocup module
        //  it has to be done before Py_Initialize()
        PyImport_AppendInittab("robocup", &PyInit_robocup);


        //	we use Py_InitializeEx(0) instead of regular Py_Initialize() so that Ctrl-C kills soccer as expected
        Py_InitializeEx(0);
        PyEval_InitThreads(); {
	        object main_module((handle<>(borrowed(PyImport_AddModule("__main__")))));
	        _mainPyNamespace = main_module.attr("__dict__");

	        object robocup_module((handle<>(PyImport_ImportModule("robocup"))));
	        _mainPyNamespace["robocup"] = robocup_module;

	        //	add gameplay directory to python import path (so import XXX) will look in the right directory
	        handle<>ignored2((PyRun_String("import sys; sys.path.append('../soccer/gameplay')",
	            Py_file_input,
	            _mainPyNamespace.ptr(),
	            _mainPyNamespace.ptr())));


	        //	instantiate the root play
	        handle<>ignored3((PyRun_String("import main; main.init()",
	            Py_file_input,
	            _mainPyNamespace.ptr(),
	            _mainPyNamespace.ptr())));
        } PyEval_SaveThread();
    } catch (error_already_set) {
        PyErr_Print();
        throw new runtime_error("Unable to initialize embedded python interpreter");
    } 
}

Gameplay::GameplayModule::~GameplayModule() {
	Py_Finalize();
}

void Gameplay::GameplayModule::setupUI() {
	PyGILState_STATE state = PyGILState_Ensure(); {
		try {
		    handle<>ignored3((PyRun_String("import ui.main; ui.main.setup()",
		        Py_file_input,
		        _mainPyNamespace.ptr(),
		        _mainPyNamespace.ptr())));
		} catch (error_already_set) {
			PyErr_Print();
			throw new runtime_error("Error trying to setup python-based UI");
		}
	} PyGILState_Release(state);
}


void Gameplay::GameplayModule::goalieID(int value)
{
	_goalieID = value;

	//	pass this value to python
	PyGILState_STATE state = PyGILState_Ensure(); {
		try {
			getRootPlay().attr("goalie_id") = _goalieID;
		} catch (error_already_set) {
			cout << "PYTHON ERROR!!!" << endl;
			PyErr_Print();
			cout << "END PYTHON ERROR" << endl;
			throw new runtime_error("Error trying to set python goalie_id on root_play");
		}
	} PyGILState_Release(state);
}

/**
 * returns the group of obstacles for the field
 */
Geometry2d::CompositeShape Gameplay::GameplayModule::globalObstacles() const {
	Geometry2d::CompositeShape obstacles;
	if (_state->gameState.stayOnSide())
	{
		obstacles.add(_sideObstacle);
	}

	if (!_state->logFrame->use_our_half())
	{
		obstacles.add(_ourHalf);
	}

	if (!_state->logFrame->use_opponent_half())
	{
		obstacles.add(_opponentHalf);
	}






	/// Add non floor obstacles
	BOOST_FOREACH(const std::shared_ptr<Shape>& ptr, _nonFloor)
	{
		obstacles.add(ptr);
	}
	return obstacles;
}

/**
 * runs the current play
 */
void Gameplay::GameplayModule::run()
{
	QMutexLocker lock(&_mutex);
	
	bool verbose = false;
	if (verbose) cout << "Starting GameplayModule::run()" << endl;

	_ballMatrix = Geometry2d::TransformMatrix::translate(_state->ball.pos);


	///	prepare each bot for the next iteration by resetting temporary things
	BOOST_FOREACH(OurRobot* robot, _state->self)
	{
		if (robot) {
			robot->resetAvoidBall();
			robot->resetAvoidRobotRadii();
			robot->resetForNextIteration();
		}
	}

	/// Build a list of visible robots
	_playRobots.clear();
	BOOST_FOREACH(OurRobot *r, _state->self)
	{
		if (r->visible && r->rxIsFresh())
		{
			_playRobots.insert(r);
		}
	}

	PyGILState_STATE state = PyGILState_Ensure(); {
		try {
			//	vector of shared pointers to pass to python
			std::vector<OurRobot *> *botVector = new std::vector<OurRobot *>();
			for (auto itr = _playRobots.begin(); itr != _playRobots.end(); itr++) {
				OurRobot *ourBot = *itr;
				//	don't attempt to drive the robot that's joystick-controlled
				//	FIXME: exclude manual id robot
				// if (ourBot->shell() != MANUAL_ID) {
					botVector->push_back(ourBot);
				// }
			}
			getMainModule().attr("set_our_robots")(botVector);

			std::vector<OpponentRobot *> *theirBotVector = new std::vector<OpponentRobot *>();
			for (auto itr = _state->opp.begin(); itr != _state->opp.end(); itr++) {
				OpponentRobot *bot = *itr;
				if (bot && bot->visible) {
					theirBotVector->push_back(bot);
				}
			}
			getMainModule().attr("set_their_robots")(theirBotVector);

			getMainModule().attr("set_game_state")(_state->gameState);

			getMainModule().attr("set_system_state")(&_state);

			getMainModule().attr("set_ball")(_state->ball);
		} catch (error_already_set) {
			PyErr_Print();
			throw new runtime_error("Error trying to pass robots and/or ball and/or game state to python");
		}

		/// Run the current play
		if (verbose) cout << "  Running play" << endl;
		try {
			/*
			 We wrap this in a try catch block because main.run() should NEVER throw an exception.
			 There are exception handlers setup on the python side of the setup - anything not handled
			 there should crash the program.

			 The part where we get the behavior tree description is wrapped in its own try/catch
			 because if it fails, we don't want to crash the program.
			 */

			handle<>ignored3((PyRun_String("main.run()",
		        Py_file_input,
		        _mainPyNamespace.ptr(),
		        _mainPyNamespace.ptr())));

			try {
				//	record the state of our behavior tree
				std::string bhvrTreeDesc = extract<std::string>(getRootPlay().attr("__str__")());
				_state->logFrame->set_behavior_tree(bhvrTreeDesc);
			}
			catch (error_already_set) {
	        	PyErr_Print();
			}
		} catch (error_already_set) {
	        PyErr_Print();
	        throw new runtime_error("Error trying to run root play");
	    }
	} PyGILState_Release(state);

	/// determine global obstacles - field requirements
	/// Two versions - one set with goal area, another without for goalie
	Geometry2d::CompositeShape global_obstacles = globalObstacles();
	Geometry2d::CompositeShape obstacles_with_goal = global_obstacles;
	obstacles_with_goal.add(_goalArea);

	/// execute motion planning for each robot
	BOOST_FOREACH(OurRobot* r, _state->self) {
		if (r && r->visible) {
			/// set obstacles for the robots
			if (r->shell() == _goalieID)
				r->replanIfNeeded(global_obstacles); /// just for goalie
			else
				r->replanIfNeeded(obstacles_with_goal); /// all other robots
		}
	}

	/// visualize
	if (_state->gameState.stayAwayFromBall() && _state->ball.valid)
	{
		_state->drawCircle(_state->ball.pos, Field_CenterRadius, Qt::black, "Rules");
	}

	if (verbose) cout << "Finishing GameplayModule::run()" << endl;

	if(_state->gameState.ourScore > _our_score_last_frame)
	{
		BOOST_FOREACH(OurRobot* r, _state->self)
		{
			r->sing();
		}
	}
	_our_score_last_frame = _state->gameState.ourScore;
}


#pragma mark python

boost::python::object Gameplay::GameplayModule::getRootPlay() {
	return getMainModule().attr("root_play")();
}

boost::python::object Gameplay::GameplayModule::getMainModule() {
	return _mainPyNamespace["main"];
}
