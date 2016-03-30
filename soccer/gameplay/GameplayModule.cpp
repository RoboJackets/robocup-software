
#include <gameplay/GameplayModule.hpp>
#include <Constants.hpp>
#include <planning/MotionInstant.hpp>
#include <protobuf/LogFrame.pb.h>
#include <Robot.hpp>
#include <SystemState.hpp>

#include <stdio.h>
#include <iostream>
#include <cmath>

// for python stuff
#include "robocup-py.hpp"

using namespace Gameplay;

REGISTER_CONFIGURABLE(GameplayModule);

using namespace std;
using namespace boost;
using namespace boost::python;

using namespace Geometry2d;

ConfigDouble* GameplayModule::_fieldEdgeInset;

void GameplayModule::createConfiguration(Configuration* cfg) {
    // this sets the disance from the field boundries to the edge of the global
    // obstacles, which the
    // robots will not move through or into
    _fieldEdgeInset = new ConfigDouble(cfg, "Field Edge Obstacle", .3);
}

bool GameplayModule::hasFieldEdgeInsetChanged() const {
    if (abs(_fieldEdgeInset->value() - _oldFieldEdgeInset) >
        numeric_limits<double>::epsilon()) {
        return true;
    }
    return false;
}

Gameplay::GameplayModule::GameplayModule(SystemState* state)
    : _mutex(QMutex::Recursive) {
    _state = state;

    calculateFieldObstacles();

    _oldFieldEdgeInset = _fieldEdgeInset->value();

    _goalieID = -1;

    //
    // setup python interpreter
    //
    try {
        cout << "Initializing embedded python interpreter..." << endl;

        //  this tells python how to load the robocup module
        //  it has to be done before Py_Initialize()
        PyImport_AppendInittab("robocup", &PyInit_robocup);

        // we use Py_InitializeEx(0) instead of regular Py_Initialize() so that
        // Ctrl-C kills soccer as expected
        Py_InitializeEx(0);
        PyEval_InitThreads();
        {
            object main_module(
                (handle<>(borrowed(PyImport_AddModule("__main__")))));
            _mainPyNamespace = main_module.attr("__dict__");

            object robocup_module((handle<>(PyImport_ImportModule("robocup"))));
            _mainPyNamespace["robocup"] = robocup_module;

            QDir gameplayDir = ApplicationRunDirectory();
            gameplayDir.cd("../soccer/gameplay");

            // add gameplay directory to python import path (so import XXX)
            // will look in the right directory
            string importStmt = QString("import sys; sys.path.append('%1')")
                                    .arg(gameplayDir.absolutePath())
                                    .toStdString();
            handle<> ignored2(
                (PyRun_String(importStmt.data(), Py_file_input,
                              _mainPyNamespace.ptr(), _mainPyNamespace.ptr())));

            _mainPyNamespace["constants"] =
                handle<>(PyImport_ImportModule("constants"));

            _mainPyNamespace["constants"].attr("Field") =
                &Field_Dimensions::Current_Dimensions;

            // instantiate the root play
            handle<> ignored3(
                (PyRun_String("import main; main.init()", Py_file_input,
                              _mainPyNamespace.ptr(), _mainPyNamespace.ptr())));
        }
        PyEval_SaveThread();
    } catch (error_already_set) {
        PyErr_Print();
        throw new runtime_error(
            "Unable to initialize embedded python interpreter");
    }
}

void Gameplay::GameplayModule::calculateFieldObstacles() {
    auto dimensions = Field_Dimensions::Current_Dimensions;

    _centerMatrix =
        TransformMatrix::translate(Point(0, dimensions.Length() / 2));
    _oppMatrix = TransformMatrix::translate(Point(0, dimensions.Length())) *
                 TransformMatrix::rotate(M_PI);

    //// Make an obstacle to cover the opponent's half of the field except for
    /// one robot diameter across the center line.
    // TODO(barulicm): double check this - shouldn't the y be inset, not the x?
    float x = dimensions.Width() / 2 + (float)_fieldEdgeInset->value();
    const float y1 = dimensions.Length() / 2;
    const float y2 = dimensions.Length() + (float)_fieldEdgeInset->value();
    const float r = dimensions.CenterRadius();
    _sideObstacle = make_shared<Polygon>(
        vector<Point>{Point(-x, y1), Point(-r, y1), Point(0, y1 + r),
                      Point(r, y1), Point(x, y1), Point(x, y2), Point(-x, y2)});

    float y = -(float)_fieldEdgeInset->value();
    float deadspace = (float)_fieldEdgeInset->value();
    x = dimensions.Width() / 2.0f + (float)_fieldEdgeInset->value();
    _nonFloor[0] = make_shared<Polygon>(vector<Point>{
        Point(-x, y), Point(-x, y - 1), Point(x, y - 1), Point(x, y)});

    y = dimensions.Length() + (float)_fieldEdgeInset->value();
    _nonFloor[1] = make_shared<Polygon>(vector<Point>{
        Point(-x, y), Point(-x, y + 1), Point(x, y + 1), Point(x, y)});

    y = dimensions.FloorLength();
    _nonFloor[2] = make_shared<Polygon>(
        vector<Point>{Point(-x, -deadspace), Point(-x - 1, -deadspace),
                      Point(-x - 1, y), Point(-x, y)});

    _nonFloor[3] = make_shared<Polygon>(
        vector<Point>{Point(x, -deadspace), Point(x + 1, -deadspace),
                      Point(x + 1, y), Point(x, y)});

    const float halfFlat = dimensions.GoalFlat() / 2.0;
    const float radius = dimensions.ArcRadius();
    auto ourGoalArea = make_shared<Polygon>(
        vector<Point>{Point(-halfFlat, 0), Point(-halfFlat, radius),
                      Point(halfFlat, radius), Point(halfFlat, 0)});
    _ourGoalArea = make_shared<CompositeShape>();
    _ourGoalArea->add(ourGoalArea);
    _ourGoalArea->add(std::dynamic_pointer_cast<Shape>(
        make_shared<Circle>(Point(-halfFlat, 0), radius)));
    _ourGoalArea->add(std::dynamic_pointer_cast<Shape>(
        make_shared<Circle>(Point(halfFlat, 0), radius)));

    auto theirGoalArea = make_shared<Polygon>(
        vector<Point>{Point(-halfFlat, dimensions.Length()),
                      Point(-halfFlat, dimensions.Length() - radius),
                      Point(halfFlat, dimensions.Length() - radius),
                      Point(halfFlat, dimensions.Length())});
    _theirGoalArea = make_shared<CompositeShape>();
    _theirGoalArea->add(theirGoalArea);
    _theirGoalArea->add(std::dynamic_pointer_cast<Shape>(
        make_shared<Circle>(Point(-halfFlat, dimensions.Length()), radius)));
    _theirGoalArea->add(std::dynamic_pointer_cast<Shape>(
        make_shared<Circle>(Point(halfFlat, dimensions.Length()), radius)));

    _ourHalf = make_shared<Polygon>(
        vector<Point>{Point(-x, -dimensions.Border()), Point(-x, y1),
                      Point(x, y1), Point(x, -dimensions.Border())});

    _opponentHalf = make_shared<Polygon>(vector<Point>{
        Point(-x, y1), Point(-x, y2), Point(x, y2), Point(x, y1)});

    _ourGoal = make_shared<Polygon>(vector<Point>{
        Point(-dimensions.GoalWidth() / 2, 0),
        Point(-dimensions.GoalWidth() / 2 - dimensions.LineWidth(), 0),
        Point(-dimensions.GoalWidth() / 2 - dimensions.LineWidth(),
              -dimensions.GoalDepth() - dimensions.LineWidth()),
        Point(dimensions.GoalWidth() / 2 + dimensions.LineWidth(),
              -dimensions.GoalDepth() - dimensions.LineWidth()),
        Point(dimensions.GoalWidth() / 2 + dimensions.LineWidth(), 0),
        Point(dimensions.GoalWidth() / 2, 0),
        Point(dimensions.GoalWidth() / 2, -dimensions.GoalDepth()),
        Point(-dimensions.GoalWidth() / 2, -dimensions.GoalDepth())});

    _theirGoal = make_shared<Polygon>(vector<Point>{
        Point(-dimensions.GoalWidth() / 2, dimensions.Length()),
        Point(-dimensions.GoalWidth() / 2 - dimensions.LineWidth(),
              dimensions.Length()),
        Point(-dimensions.GoalWidth() / 2 - dimensions.LineWidth(),
              dimensions.Length() + dimensions.GoalDepth() +
                  dimensions.LineWidth()),
        Point(dimensions.GoalWidth() / 2 + dimensions.LineWidth(),
              dimensions.Length() + dimensions.GoalDepth() +
                  dimensions.LineWidth()),
        Point(dimensions.GoalWidth() / 2 + dimensions.LineWidth(),
              dimensions.Length()),
        Point(dimensions.GoalWidth() / 2, dimensions.Length()),
        Point(dimensions.GoalWidth() / 2,
              dimensions.Length() + dimensions.GoalDepth()),
        Point(-dimensions.GoalWidth() / 2,
              dimensions.Length() + dimensions.GoalDepth())});

    _oldFieldEdgeInset = _fieldEdgeInset->value();
}

Gameplay::GameplayModule::~GameplayModule() {
    // Apparently this is broken in Boost 1.57 as per:
    // http://www.boost.org/doc/libs/1_57_0/libs/python/doc/tutorial/doc/html/python/embedding.html
    // Py_Finalize();
}

void Gameplay::GameplayModule::setupUI() {
    PyGILState_STATE state = PyGILState_Ensure();
    {
        try {
            handle<> ignored3(
                (PyRun_String("import ui.main; ui.main.setup()", Py_file_input,
                              _mainPyNamespace.ptr(), _mainPyNamespace.ptr())));
        } catch (error_already_set) {
            PyErr_Print();
            throw new runtime_error("Error trying to setup python-based UI");
        }
    }
    PyGILState_Release(state);
}

void Gameplay::GameplayModule::loadPlaybook(const string& playbookFile,
                                            bool isAbsolute) {
    PyGILState_STATE state = PyGILState_Ensure();
    try {
        getMainModule().attr("load_playbook")(playbookFile, isAbsolute);
    } catch (error_already_set) {
        PyErr_Print();
        PyGILState_Release(state);
        throw new runtime_error("Error trying to load playbook.");
    }
    PyGILState_Release(state);
}

void Gameplay::GameplayModule::savePlaybook(const string& playbookFile,
                                            bool isAbsolute) {
    PyGILState_STATE state = PyGILState_Ensure();
    try {
        getMainModule().attr("save_playbook")(playbookFile, isAbsolute);
    } catch (error_already_set) {
        PyErr_Print();
        PyGILState_Release(state);
        throw new runtime_error("Error trying to save playbook.");
    }
    PyGILState_Release(state);
}

void Gameplay::GameplayModule::goalieID(int value) {
    _goalieID = value;

    // pass this value to python
    PyGILState_STATE state = PyGILState_Ensure();
    {
        try {
            getRootPlay().attr("goalie_id") = _goalieID;
        } catch (error_already_set) {
            cout << "PYTHON ERROR!!!" << endl;
            PyErr_Print();
            cout << "END PYTHON ERROR" << endl;
            throw new runtime_error(
                "Error trying to set python goalie_id on root_play");
        }
    }
    PyGILState_Release(state);
}

/**
 * returns the group of obstacles for the field
 */
Geometry2d::ShapeSet Gameplay::GameplayModule::globalObstacles() const {
    Geometry2d::ShapeSet obstacles;
    if (_state->gameState.stayOnSide()) {
        obstacles.add(_sideObstacle);
    }

    if (!_state->logFrame->use_our_half()) {
        obstacles.add(_ourHalf);
    }

    if (!_state->logFrame->use_opponent_half()) {
        obstacles.add(_opponentHalf);
    }

    /// Add non floor obstacles
    for (const std::shared_ptr<Shape>& ptr : _nonFloor) {
        obstacles.add(ptr);
    }

    obstacles.add(_ourGoal);
    obstacles.add(_theirGoal);

    return obstacles;
}

Geometry2d::ShapeSet Gameplay::GameplayModule::goalZoneObstacles() const {
    Geometry2d::ShapeSet zones;
    zones.add(_theirGoalArea);
    zones.add(_ourGoalArea);
    return zones;
}

/**
 * runs the current play
 */
void Gameplay::GameplayModule::run() {
    QMutexLocker lock(&_mutex);

    bool verbose = false;
    if (verbose) cout << "Starting GameplayModule::run()" << endl;

    _ballMatrix = Geometry2d::TransformMatrix::translate(_state->ball.pos);

    /// prepare each bot for the next iteration by resetting temporary things
    for (OurRobot* robot : _state->self) {
        if (robot) {
            robot->resetAvoidBall();
            robot->resetAvoidRobotRadii();
            robot->resetForNextIteration();
        }
    }

    /// Build a list of visible robots
    _playRobots.clear();
    for (OurRobot* r : _state->self) {
        if (r->visible && r->rxIsFresh()) {
            _playRobots.insert(r);
        }
    }

    PyGILState_STATE state = PyGILState_Ensure();
    {
        try {
            // vector of shared pointers to pass to python
            vector<OurRobot*>* botVector = new vector<OurRobot*>();
            for (auto itr = _playRobots.begin(); itr != _playRobots.end();
                 itr++) {
                OurRobot* ourBot = *itr;
                // don't attempt to drive the robot that's joystick-controlled
                // FIXME: exclude manual id robot
                // if (ourBot->shell() != MANUAL_ID) {
                botVector->push_back(ourBot);
                // }
            }
            getMainModule().attr("set_our_robots")(botVector);

            vector<OpponentRobot*>* theirBotVector =
                new vector<OpponentRobot*>();
            for (auto itr = _state->opp.begin(); itr != _state->opp.end();
                 itr++) {
                OpponentRobot* bot = *itr;
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
            throw new runtime_error(
                "Error trying to pass robots and/or ball and/or game state to "
                "python");
        }

        /// Run the current play
        if (verbose) cout << "  Running play" << endl;
        try {
            /*
             We wrap this in a try catch block because main.run() should NEVER
             throw an exception.
             There are exception handlers setup on the python side of the setup
             - anything not handled
             there should crash the program.

             The part where we get the behavior tree description is wrapped in
             its own try/catch
             because if it fails, we don't want to crash the program.
             */

            handle<> ignored3(
                (PyRun_String("main.run()", Py_file_input,
                              _mainPyNamespace.ptr(), _mainPyNamespace.ptr())));

            try {
                // record the state of our behavior tree
                std::string bhvrTreeDesc =
                    extract<std::string>(getRootPlay().attr("__str__")());
                _state->logFrame->set_behavior_tree(bhvrTreeDesc);
            } catch (error_already_set) {
                PyErr_Print();
            }
        } catch (error_already_set) {
            PyErr_Print();
            throw new runtime_error("Error trying to run root play");
        }
    }
    PyGILState_Release(state);

    /// visualize
    if (_state->gameState.stayAwayFromBall() && _state->ball.valid) {
        _state->drawCircle(_state->ball.pos,
                           Field_Dimensions::Current_Dimensions.CenterRadius(),
                           Qt::black, "Rules");
    }

    if (verbose) cout << "Finishing GameplayModule::run()" << endl;

    if (_state->gameState.ourScore > _our_score_last_frame) {
        for (OurRobot* r : _state->self) {
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
