#include <protobuf/LogFrame.pb.h>

#include <Constants.hpp>
#include <Network.hpp>
#include <Referee.hpp>
#include <Robot.hpp>
#include <SystemState.hpp>
#include <gameplay/GameplayModule.hpp>
#include <planning/MotionInstant.hpp>

// for python stuff
#include "DebugDrawer.hpp"
#include "robocup-py.hpp"

using namespace Gameplay;

REGISTER_CONFIGURABLE(GameplayModule);

using namespace std;
using namespace boost;
using namespace boost::python;

using namespace Geometry2d;
using namespace RefereeModuleEnums;

ConfigDouble* GameplayModule::_fieldEdgeInset;

void GameplayModule::createConfiguration(Configuration* cfg) {
    /*  This sets the disance from the field boundries to the edge of the global
+     * obstacles, which the robots will not move through or into.
+     * The value is given in meters. As of April 20, 2016 the inner 300mm
+     * is free space for the robots.
+     */
    _fieldEdgeInset =
        new ConfigDouble(cfg, "PathPlanner/Field Edge Obstacle", .33);
}

bool GameplayModule::hasFieldEdgeInsetChanged() const {
    return abs(_fieldEdgeInset->value() - _oldFieldEdgeInset) >
           numeric_limits<double>::epsilon();
}

// TODO: Replace this whole file when we move to ROS2
Gameplay::GameplayModule::GameplayModule(Context* const context,
                                         Referee* const refereeModule)
    : _context(context), _refereeModule(refereeModule) {
    calculateFieldObstacles();

    _oldFieldEdgeInset = _fieldEdgeInset->value();

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
    } catch (const error_already_set&) {
        PyErr_Print();
        throw runtime_error("Unable to initialize embedded python interpreter");
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
    auto deadspace = (float)_fieldEdgeInset->value();
    x = dimensions.Width() / 2.0f + (float)_fieldEdgeInset->value();
    _nonFloor[0] = make_shared<Polygon>(vector<Point>{
        Point(-x, y), Point(-x, y - 1000), Point(x, y - 1000), Point(x, y)});

    y = dimensions.Length() + (float)_fieldEdgeInset->value();
    _nonFloor[1] = make_shared<Polygon>(vector<Point>{
        Point(-x, y), Point(-x, y + 1000), Point(x, y + 1000), Point(x, y)});

    y = dimensions.FloorLength();
    _nonFloor[2] = make_shared<Polygon>(vector<Point>{
        Point(-x, -3 * deadspace), Point(-x - 1000, -3 * deadspace),
        Point(-x - 1000, y), Point(-x, y)});

    _nonFloor[3] = make_shared<Polygon>(
        vector<Point>{Point(x, -3 * deadspace), Point(x + 1000, -3 * deadspace),
                      Point(x + 1000, y), Point(x, y)});

    const float halfFlat = static_cast<float>(dimensions.GoalFlat() / 2.0);
    const float shortDist = dimensions.PenaltyShortDist();
    const float longDist = dimensions.PenaltyLongDist();

    auto ourGoalArea = make_shared<Polygon>(vector<Point>{
        Point(-longDist / 2, 0), Point(longDist / 2, 0),
        Point(longDist / 2, shortDist), Point(-longDist / 2, shortDist)});
    _ourGoalArea = make_shared<CompositeShape>();

    _ourGoalArea->add(ourGoalArea);

    auto theirGoalArea = make_shared<Polygon>(
        vector<Point>{Point(-longDist / 2, dimensions.Length()),
                      Point(longDist / 2, dimensions.Length()),
                      Point(longDist / 2, dimensions.Length() - shortDist),
                      Point(-longDist / 2, dimensions.Length() - shortDist)});
    _theirGoalArea = make_shared<CompositeShape>();

    _theirGoalArea->add(theirGoalArea);

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
        } catch (const error_already_set&) {
            PyErr_Print();
            throw runtime_error("Error trying to setup python-based UI");
        }
    }
    PyGILState_Release(state);
}

void Gameplay::GameplayModule::loadPlaybook(const string& playbookFile,
                                            bool isAbsolute) {
    PyGILState_STATE state = PyGILState_Ensure();
    try {
        getMainModule().attr("load_playbook")(playbookFile, isAbsolute);
    } catch (const error_already_set&) {
        PyErr_Print();
        PyGILState_Release(state);
        throw runtime_error("Error trying to load playbook.");
    }
    PyGILState_Release(state);
}

void Gameplay::GameplayModule::savePlaybook(const string& playbookFile,
                                            bool isAbsolute) {
    PyGILState_STATE state = PyGILState_Ensure();
    try {
        getMainModule().attr("save_playbook")(playbookFile, isAbsolute);
    } catch (const error_already_set&) {
        PyErr_Print();
        PyGILState_Release(state);
        throw runtime_error("Error trying to save playbook.");
    }
    PyGILState_Release(state);
}

void Gameplay::GameplayModule::clearPlays() {
    PyGILState_STATE state = PyGILState_Ensure();
    getMainModule().attr("clear")();
    PyGILState_Release(state);
}

bool Gameplay::GameplayModule::checkPlaybookStatus() {
    PyGILState_STATE state = PyGILState_Ensure();
    static int prevStatus =
        extract<int>(getMainModule().attr("numEnablePlays")());
    bool static change = false;
    int status = extract<int>(getMainModule().attr("numEnablePlays")());
    if (status == 0) {
        change = false;
    } else if (status != prevStatus) {
        change = (abs(prevStatus - status) == 1);
    }
    prevStatus = status;
    PyGILState_Release(state);
    return change;
}

/**
 * returns the group of obstacles for the field
 */
Geometry2d::ShapeSet Gameplay::GameplayModule::globalObstacles() const {
    Geometry2d::ShapeSet obstacles;
    if (_context->game_state.stayOnSide()) {
        obstacles.add(_sideObstacle);
    }

    if (!_context->game_settings.use_our_half) {
        obstacles.add(_ourHalf);
    }

    if (!_context->game_settings.use_their_half) {
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
    bool verbose = false;
    if (verbose) {
        cout << "Starting GameplayModule::run()" << endl;
    }

    _ballMatrix =
        Geometry2d::TransformMatrix::translate(_context->state.ball.pos);

    /// prepare each bot for the next iteration by resetting temporary things
    for (OurRobot* robot : _context->state.self) {
        if (robot != nullptr) {
            robot->resetAvoidBall();
            robot->resetAvoidRobotRadii();
            robot->resetForNextIteration();
        }
    }

    /// Build a list of visible robots
    _playRobots.clear();
    for (OurRobot* r : _context->state.self) {
        if (r->visible() && r->statusIsFresh()) {
            _playRobots.insert(r);
        }
    }

    PyGILState_STATE state = PyGILState_Ensure();
    {
        try {
            // vector of shared pointers to pass to python
            vector<OurRobot*> botVector;
            for (auto* ourBot : _playRobots) {
                // don't attempt to drive the robot that's joystick-controlled
                // FIXME: exclude manual id robot
                // if (ourBot->shell() != MANUAL_ID) {
                botVector.push_back(ourBot);
                // }
            }
            getMainModule().attr("set_our_robots")(botVector);

            vector<OpponentRobot*> theirBotVector;
            for (auto* bot : _context->state.opp) {
                if (bot != nullptr && bot->visible()) {
                    theirBotVector.push_back(bot);
                }
            }
            getMainModule().attr("set_their_robots")(theirBotVector);

            getMainModule().attr("set_context")(&_context);

            // Handle Tests
            if (runningTests) {
                // I could add a bool to check if this needs to run or not if
                // this is too inefficient
                object rtrn(handle<>(PyRun_String(
                    "ui.main._tests.getNextCommand()", Py_eval_input,
                    _mainPyNamespace.ptr(), _mainPyNamespace.ptr())));

                // TODO(Kyle): Part two of the
                //  multiple-places-publishing-to-the-same-struct garbage-fest.
                if (rtrn.ptr() != Py_None) {
                    Command cmd = extract<Command>(rtrn);
                    _context->game_settings.requestRefCommand = cmd;
                }
            }

        } catch (const error_already_set&) {
            PyErr_Print();
            throw runtime_error(
                "Error trying to pass robots and/or ball and/or game state to "
                "python");
        }

        /// Run the current play
        if (verbose) {
            cout << "  Running play" << endl;
        }
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
                _context->behavior_tree = bhvrTreeDesc;
            } catch (const error_already_set&) {
                PyErr_Print();
            }
        } catch (const error_already_set&) {
            PyErr_Print();
            throw runtime_error("Error trying to run root play");
        }
    }
    PyGILState_Release(state);

    /// visualize
    if (_context->game_state.stayAwayFromBall() && _context->state.ball.valid) {
        _context->debug_drawer.drawCircle(
            _context->state.ball.pos,
            Field_Dimensions::Current_Dimensions.CenterRadius(), Qt::black,
            "Rules");
    }

    if (verbose) {
        cout << "Finishing GameplayModule::run()" << endl;
    }

    if (_context->game_state.ourScore > _our_score_last_frame) {
        for (OurRobot* r : _context->state.self) {
            r->sing();
        }
    }
    _our_score_last_frame = _context->game_state.ourScore;
}

#pragma mark python

boost::python::object Gameplay::GameplayModule::getRootPlay() {
    return getMainModule().attr("root_play")();
}

boost::python::object Gameplay::GameplayModule::getMainModule() {
    return _mainPyNamespace["main"];
}

void Gameplay::GameplayModule::updateFieldDimensions() {
    PyGILState_STATE state = PyGILState_Ensure();
    {
        _mainPyNamespace["constants"].attr("Field") =
            &Field_Dimensions::Current_Dimensions;
    }
    PyGILState_Release(state);
}

void Gameplay::GameplayModule::addTests() {
    PyGILState_STATE state = PyGILState_Ensure();
    {
        try {
            handle<> ignored3((PyRun_String(
                "import ui.main; ui.main._tests.addTests()", Py_file_input,
                _mainPyNamespace.ptr(), _mainPyNamespace.ptr())));
        } catch (const error_already_set&) {
            PyErr_Print();
            throw runtime_error("Error trying to add tests");
        }
    }
    PyGILState_Release(state);

    // TODO: Implement custom MIME type for tests
    //      Enable dragdrop in allTestsTable
    //      Link selectedTestsTable to a python list in main.py
}

void Gameplay::GameplayModule::removeTest() {
    PyGILState_STATE state = PyGILState_Ensure();
    {
        try {
            handle<> ignored3((PyRun_String(
                "import ui.main; ui.main._tests.removeTest()", Py_file_input,
                _mainPyNamespace.ptr(), _mainPyNamespace.ptr())));
        } catch (const error_already_set&) {
            PyErr_Print();
            throw runtime_error("Error trying to add tests");
        }
    }
    PyGILState_Release(state);
}

void Gameplay::GameplayModule::nextTest() {
    PyGILState_STATE state = PyGILState_Ensure();
    {
        try {
            object rtrn(handle<>(
                PyRun_String("ui.main._tests.nextTest()", Py_eval_input,
                             _mainPyNamespace.ptr(), _mainPyNamespace.ptr())));

            runningTests = extract<bool>(rtrn);
        } catch (const error_already_set&) {
            PyErr_Print();
            throw runtime_error("Error trying to go to next test");
        }
    }
    PyGILState_Release(state);

    // load the test
    loadTest();
}

void Gameplay::GameplayModule::loadTest() {
    PyGILState_STATE state = PyGILState_Ensure();
    {
        try {
            object rtrn(handle<>(
                PyRun_String("ui.main._tests.loadTest()", Py_eval_input,
                             _mainPyNamespace.ptr(), _mainPyNamespace.ptr())));

            runningTests = extract<bool>(rtrn);

            // TODO(Kyle): Okay, so really all of this testing logic should be
            // removed from Gameplay and put behind some sort of GameController
            // abstraction that it shares with the main UI code. However, for
            // now we can hack around it by publishing to the same struct twice
            // from two different places. This is the big sad.
            //
            // See the other to-do in this file for the other instance of the
            // same issue.
            if (runningTests) {
                _context->game_settings.requestRefCommand = Command::HALT;

                // Place robots and ball
                grSim_Packet simPacket;

                grSim_Replacement* replacement =
                    simPacket.mutable_replacement();

                // Load OurRobots information
                object our_robot_rtrn(handle<>(PyRun_String(
                    "ui.main._tests.getTestOurRobots()", Py_eval_input,
                    _mainPyNamespace.ptr(), _mainPyNamespace.ptr())));

                boost::python::list our_robots =
                    extract<boost::python::list>(our_robot_rtrn);

                const int NUM_COLS = 2;
                const int ROBOTS_PER_COL = Robots_Per_Team / NUM_COLS;
                const int teamDirection =
                    _context->game_state.blueTeam ? -1 : 1;
                for (int i = 0; i < Robots_Per_Team; i++) {
                    auto* rob = replacement->add_robots();

                    if (i < len(our_robots)) {
                        boost::python::list robot =
                            extract<boost::python::list>(our_robots[i]);

                        float x = extract<float>(robot[0]);
                        float y = extract<float>(robot[1]);

                        rob->set_x(
                            -teamDirection *
                            (y -
                             (Field_Dimensions::Current_Dimensions.Length() /
                              2)));
                        rob->set_y(teamDirection * x);
                        rob->set_dir(extract<float>(robot[2]));
                    } else {
                        double x_pos =
                            teamDirection * (2.5 - i / ROBOTS_PER_COL);
                        double y_pos =
                            i % ROBOTS_PER_COL - ROBOTS_PER_COL / NUM_COLS;
                        rob->set_x(x_pos);
                        rob->set_y(y_pos);
                        rob->set_dir(0);
                    }
                    rob->set_id(i);
                    rob->set_yellowteam(not _context->game_state.blueTeam);
                }

                // Load TheirRobots information
                object their_robot_rtrn(handle<>(PyRun_String(
                    "ui.main._tests.getTestTheirRobots()", Py_eval_input,
                    _mainPyNamespace.ptr(), _mainPyNamespace.ptr())));

                boost::python::list their_robots =
                    extract<boost::python::list>(their_robot_rtrn);

                for (int i = 0; i < Robots_Per_Team; i++) {
                    auto* rob = replacement->add_robots();

                    if (i < len(their_robots)) {
                        boost::python::list robot =
                            extract<boost::python::list>(their_robots[i]);

                        float x = extract<float>(robot[0]);
                        float y = extract<float>(robot[1]);

                        rob->set_x(
                            -teamDirection *
                            (y -
                             (Field_Dimensions::Current_Dimensions.Length() /
                              2)));
                        rob->set_y(teamDirection * x);
                        rob->set_dir(extract<float>(robot[2]));
                    } else {
                        double x_pos =
                            -teamDirection * (2.5 - i / ROBOTS_PER_COL);
                        double y_pos =
                            i % ROBOTS_PER_COL - ROBOTS_PER_COL / NUM_COLS;
                        rob->set_x(x_pos);
                        rob->set_y(y_pos);
                        rob->set_dir(0);
                    }
                    rob->set_id(i);
                    rob->set_yellowteam(_context->game_state.blueTeam);
                }

                // Get ball Information
                object ball_rtrn(handle<>(PyRun_String(
                    "ui.main._tests.getTestBall()", Py_eval_input,
                    _mainPyNamespace.ptr(), _mainPyNamespace.ptr())));

                boost::python::list ball =
                    extract<boost::python::list>(ball_rtrn);
                auto* ball_replace = replacement->mutable_ball();
                double posx = extract<double>(ball[0]);
                double posy = extract<double>(ball[1]);
                double velx = extract<double>(ball[2]);
                double vely = extract<double>(ball[3]);

                ball_replace->set_x(
                    -teamDirection *
                    (posy -
                     (Field_Dimensions::Current_Dimensions.Length() / 2)));
                ball_replace->set_y(teamDirection * posx);
                ball_replace->set_vx(-teamDirection * vely);
                ball_replace->set_vy(teamDirection * velx);

                _context->grsim_command = simPacket;
            }

        } catch (const error_already_set&) {
            PyErr_Print();
            throw runtime_error("Error trying to load test");
        }
    }
    PyGILState_Release(state);
}
