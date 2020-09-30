#include <robot.hpp>
#include <system_state.hpp>
#include <gameplay/gameplay_module.hpp>
#include <planning/instant.hpp>
#include <referee/external_referee.hpp>
#include <rj_common/field_dimensions.hpp>
#include <rj_common/network.hpp>
#include <rj_common/qt_utils.hpp>
#include <rj_constants/constants.hpp>
#include <rj_protos/LogFrame.pb.h>

// for python stuff
#include "debug_drawer.hpp"

#include "robocup-py.hpp"

// For getting data
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace Gameplay;

REGISTER_CONFIGURABLE(GameplayModule);

using namespace std;
using namespace boost;
using namespace boost::python;

using namespace rj_geometry;
using namespace RefereeModuleEnums;

ConfigDouble* GameplayModule::field_edge_inset;

void GameplayModule::create_configuration(Configuration* cfg) {
    /*  This sets the disance from the field boundries to the edge of the global
+     * obstacles, which the robots will not move through or into.
+     * The value is given in meters. As of April 20, 2016 the inner 300mm
+     * is free space for the robots.
+     */
    field_edge_inset = new ConfigDouble(cfg, "PathPlanner/Field Edge Obstacle", .33);
}

bool GameplayModule::has_field_edge_inset_changed() const {
    return abs(field_edge_inset->value() - old_field_edge_inset_) >
           numeric_limits<double>::epsilon();
}

// TODO: Replace this whole file when we move to ROS2
Gameplay::GameplayModule::GameplayModule(Context* context) : context_(context) {
    calculate_field_obstacles();

    old_field_edge_inset_ = field_edge_inset->value();

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
            object main_module((handle<>(borrowed(PyImport_AddModule("__main__")))));
            main_py_namespace_ = main_module.attr("__dict__");

            object robocup_module((handle<>(PyImport_ImportModule("robocup"))));
            main_py_namespace_["robocup"] = robocup_module;

            QDir gameplay_dir = application_run_directory();

            // Get the location of the share directory (where the python files
            // are installed)
            const auto share_dir = ament_index_cpp::get_package_share_directory("rj_robocup");
            std::stringstream command;
            command << "import sys; sys.path.append('" << share_dir << "/gameplay/')";
            handle<> ignored2{(PyRun_String(command.str().c_str(), Py_file_input,
                                            main_py_namespace_.ptr(), main_py_namespace_.ptr()))};

            main_py_namespace_["constants"] = handle<>(PyImport_ImportModule("constants"));

            main_py_namespace_["constants"].attr("Field") = &FieldDimensions::current_dimensions;

            // instantiate the root play
            handle<> ignored3((PyRun_String("import main; main.init()", Py_file_input,
                                            main_py_namespace_.ptr(), main_py_namespace_.ptr())));
        }
        PyEval_SaveThread();
    } catch (const error_already_set&) {
        PyErr_Print();
        throw runtime_error("Unable to initialize embedded python interpreter");
    }
}

void Gameplay::GameplayModule::calculate_field_obstacles() {
    auto dimensions = FieldDimensions::current_dimensions;

    center_matrix_ = TransformMatrix::translate(Point(0, dimensions.length() / 2));
    opp_matrix_ =
        TransformMatrix::translate(Point(0, dimensions.length())) * TransformMatrix::rotate(M_PI);

    //// Make an obstacle to cover the opponent's half of the field except for
    /// one robot diameter across the center line.
    // TODO(barulicm): double check this - shouldn't the y be inset, not the x?
    float x = dimensions.width() / 2 + (float)field_edge_inset->value();
    const float y1 = dimensions.length() / 2;
    const float y2 = dimensions.length() + (float)field_edge_inset->value();
    const float r = dimensions.center_radius();
    side_obstacle_ = make_shared<Rect>(Point(-x, y1), Point(x, y2));

    float y = -(float)field_edge_inset->value();
    auto deadspace = (float)field_edge_inset->value();
    x = dimensions.width() / 2.0f + (float)field_edge_inset->value();
    non_floor_[0] = make_shared<Rect>(Point(-x, y), Point(x, y - 1000));

    y = dimensions.length() + (float)field_edge_inset->value();
    non_floor_[1] = make_shared<Rect>(Point(-x, y), Point(x, y + 1000));

    y = dimensions.floor_length();
    non_floor_[2] = make_shared<Rect>(Point(-x, -3 * deadspace), Point(-x - 1000, y));

    non_floor_[3] = make_shared<Rect>(Point(x, -3 * deadspace), Point(x + 1000, y));

    const float half_flat = static_cast<float>(dimensions.goal_flat() / 2.0);
    const float short_dist = dimensions.penalty_short_dist();
    const float long_dist = dimensions.penalty_long_dist();

    our_goal_area_ = make_shared<Rect>(Point(-long_dist / 2, 0), Point(long_dist / 2, short_dist));

    their_goal_area_ = make_shared<Rect>(Point(-long_dist / 2, dimensions.length()),
                                         Point(long_dist / 2, dimensions.length() - short_dist));

    our_half_ = make_shared<Rect>(Point(-x, -dimensions.border()), Point(x, y1));

    opponent_half_ = make_shared<Rect>(Point(-x, y1), Point(x, y2));

    our_goal_ = make_shared<Rect>(Point(-dimensions.goal_width() / 2 - dimensions.line_width(), 0),
                                  Point(dimensions.goal_width() / 2 + dimensions.line_width(),
                                        -dimensions.goal_depth() - dimensions.line_width()));

    their_goal_ = make_shared<Rect>(
        Point(-dimensions.goal_width() / 2, dimensions.length()),
        Point(dimensions.goal_width() / 2 + dimensions.line_width(),
              dimensions.length() + dimensions.goal_depth() + dimensions.line_width()));

    old_field_edge_inset_ = field_edge_inset->value();
}

Gameplay::GameplayModule::~GameplayModule() {
    // Apparently this is broken in Boost 1.57 as per:
    // http://www.boost.org/doc/libs/1_57_0/libs/python/doc/tutorial/doc/html/python/embedding.html
    // Py_Finalize();
}

void Gameplay::GameplayModule::setup_ui() {
    PyGILState_STATE state = PyGILState_Ensure();
    {
        try {
            handle<> ignored3((PyRun_String("import ui.main; ui.main.setup()", Py_file_input,
                                            main_py_namespace_.ptr(), main_py_namespace_.ptr())));
        } catch (const error_already_set&) {
            PyErr_Print();
            throw runtime_error("Error trying to setup python-based UI");
        }
    }
    PyGILState_Release(state);
}

void Gameplay::GameplayModule::load_playbook(const string& playbook_file, bool is_absolute) {
    PyGILState_STATE state = PyGILState_Ensure();
    try {
        get_main_module().attr("load_playbook")(playbook_file, is_absolute);
    } catch (const error_already_set&) {
        PyErr_Print();
        PyGILState_Release(state);
        throw runtime_error("Error trying to load playbook.");
    }
    PyGILState_Release(state);
}

void Gameplay::GameplayModule::save_playbook(const string& playbook_file, bool is_absolute) {
    PyGILState_STATE state = PyGILState_Ensure();
    try {
        get_main_module().attr("save_playbook")(playbook_file, is_absolute);
    } catch (const error_already_set&) {
        PyErr_Print();
        PyGILState_Release(state);
        throw runtime_error("Error trying to save playbook.");
    }
    PyGILState_Release(state);
}

void Gameplay::GameplayModule::clear_plays() {
    PyGILState_STATE state = PyGILState_Ensure();
    get_main_module().attr("clear")();
    PyGILState_Release(state);
}

bool Gameplay::GameplayModule::check_playbook_status() {
    PyGILState_STATE state = PyGILState_Ensure();
    static int prev_status = extract<int>(get_main_module().attr("numEnablePlays")());
    bool static change = false;
    int status = extract<int>(get_main_module().attr("numEnablePlays")());
    if (status == 0) {
        change = false;
    } else if (status != prev_status) {
        change = (abs(prev_status - status) == 1);
    }
    prev_status = status;
    PyGILState_Release(state);
    return change;
}

/**
 * returns the group of obstacles for the field
 */
rj_geometry::ShapeSet Gameplay::GameplayModule::global_obstacles() const {
    rj_geometry::ShapeSet obstacles;
    if (context_->game_state.stay_on_side()) {
        obstacles.add(side_obstacle_);
    }

    if (!context_->game_settings.use_our_half) {
        obstacles.add(our_half_);
    }

    if (!context_->game_settings.use_their_half) {
        obstacles.add(opponent_half_);
    }

    /// Add non floor obstacles
    for (const std::shared_ptr<Shape>& ptr : non_floor_) {
        obstacles.add(ptr);
    }

    obstacles.add(our_goal_);
    obstacles.add(their_goal_);

    return obstacles;
}

rj_geometry::ShapeSet Gameplay::GameplayModule::goal_zone_obstacles() const {
    rj_geometry::ShapeSet zones;
    zones.add(their_goal_area_);
    zones.add(our_goal_area_);
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

    ball_matrix_ = rj_geometry::TransformMatrix::translate(context_->world_state.ball.position);

    context_->global_obstacles = global_obstacles();
    context_->goal_zone_obstacles = goal_zone_obstacles();

    /// prepare each bot for the next iteration by resetting temporary things
    for (OurRobot* robot : context_->state.self) {
        if (robot != nullptr) {
            robot->reset_for_next_iteration();
        }
    }

    /// Build a list of visible robots
    play_robots_.clear();
    for (OurRobot* r : context_->state.self) {
        if (r->visible() && r->status_is_fresh()) {
            play_robots_.insert(r);
        }
    }

    PyGILState_STATE state = PyGILState_Ensure();
    {
        try {
            // vector of shared pointers to pass to python
            vector<OurRobot*> bot_vector;
            for (auto* our_bot : play_robots_) {
                // don't attempt to drive the robot that's joystick-controlled
                // FIXME: exclude manual id robot
                // if (our_bot->shell() != MANUAL_ID) {
                bot_vector.push_back(our_bot);
                // }
            }
            get_main_module().attr("set_our_robots")(bot_vector);

            vector<OpponentRobot*> their_bot_vector;
            for (auto* bot : context_->state.opp) {
                if (bot != nullptr && bot->visible()) {
                    their_bot_vector.push_back(bot);
                }
            }
            get_main_module().attr("set_their_robots")(their_bot_vector);

            get_main_module().attr("set_context")(&context_);

            // Handle Tests
            if (running_tests_) {
                // I could add a bool to check if this needs to run or not if
                // this is too inefficient
                object rtrn(
                    handle<>(PyRun_String("ui.main._tests.getNextCommand()", Py_eval_input,
                                          main_py_namespace_.ptr(), main_py_namespace_.ptr())));

#if 0
                // TODO(Kyle): Part two of the
                //  multiple-places-publishing-to-the-same-struct garbage-fest.
                if (rtrn.ptr() != Py_None) {
                    Command cmd = extract<Command>(rtrn);
                    context_->game_settings.request_ref_command = cmd;
                }
#endif
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

            handle<> ignored3((PyRun_String("main.run()", Py_file_input, main_py_namespace_.ptr(),
                                            main_py_namespace_.ptr())));

            try {
                // record the state of our behavior tree
                std::string bhvr_tree_desc =
                    extract<std::string>(get_root_play().attr("__str__")());
                context_->behavior_tree = bhvr_tree_desc;
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
    if (context_->game_state.stay_away_from_ball() && context_->world_state.ball.visible) {
        context_->debug_drawer.draw_circle(context_->world_state.ball.position,
                                           FieldDimensions::current_dimensions.center_radius(),
                                           Qt::black, "Rules");
    }

    if (verbose) {
        cout << "Finishing GameplayModule::run()" << endl;
    }

    our_score_last_frame_ = context_->our_info.score;
}

#pragma mark python

boost::python::object Gameplay::GameplayModule::get_root_play() {
    return get_main_module().attr("root_play")();
}

boost::python::object Gameplay::GameplayModule::get_main_module() {
    return main_py_namespace_["main"];
}

void Gameplay::GameplayModule::update_field_dimensions() {
    PyGILState_STATE state = PyGILState_Ensure();
    { main_py_namespace_["constants"].attr("Field") = &FieldDimensions::current_dimensions; }
    PyGILState_Release(state);
}

void Gameplay::GameplayModule::add_tests() {
    PyGILState_STATE state = PyGILState_Ensure();
    {
        try {
            handle<> ignored3(
                (PyRun_String("import ui.main; ui.main._tests.addTests()", Py_file_input,
                              main_py_namespace_.ptr(), main_py_namespace_.ptr())));
        } catch (const error_already_set&) {
            PyErr_Print();
            throw runtime_error("Error trying to add tests");
        }
    }
    PyGILState_Release(state);

    // TODO: Implement custom MIME type for tests
    //      Enable dragdrop in all_tests_table
    //      Link selected_tests_table to a python list in main.py
}

void Gameplay::GameplayModule::remove_test() {
    PyGILState_STATE state = PyGILState_Ensure();
    {
        try {
            handle<> ignored3(
                (PyRun_String("import ui.main; ui.main._tests.removeTest()", Py_file_input,
                              main_py_namespace_.ptr(), main_py_namespace_.ptr())));
        } catch (const error_already_set&) {
            PyErr_Print();
            throw runtime_error("Error trying to add tests");
        }
    }
    PyGILState_Release(state);
}

void Gameplay::GameplayModule::next_test() {
    PyGILState_STATE state = PyGILState_Ensure();
    {
        try {
            object rtrn(handle<>(PyRun_String("ui.main._tests.nextTest()", Py_eval_input,
                                              main_py_namespace_.ptr(), main_py_namespace_.ptr())));

            running_tests_ = extract<bool>(rtrn);
        } catch (const error_already_set&) {
            PyErr_Print();
            throw runtime_error("Error trying to go to next test");
        }
    }
    PyGILState_Release(state);

    // load the test
    load_test();
}

void Gameplay::GameplayModule::load_test() {
    PyGILState_STATE state = PyGILState_Ensure();
    {
        try {
            object rtrn(handle<>(PyRun_String("ui.main._tests.loadTest()", Py_eval_input,
                                              main_py_namespace_.ptr(), main_py_namespace_.ptr())));

            running_tests_ = extract<bool>(rtrn);

#if 0
            // TODO(Kyle): Okay, so really all of this testing logic should be
            // removed from Gameplay and put behind some sort of GameController
            // abstraction that it shares with the main UI code. However, for
            // now we can hack around it by publishing to the same struct twice
            // from two different places. This is the big sad.
            //
            // See the other to-do in this file for the other instance of the
            // same issue.
            if (running_tests_) {
                context_->game_settings.request_ref_command =
                    SSL_Referee_Command_HALT;

                // Place robots and ball
                grSim_Packet sim_packet;

                grSim_Replacement* replacement =
                    simPacket.mutable_replacement();

                // Load OurRobots information
                object our_robot_rtrn(handle<>(PyRun_String(
                    "ui.main._tests.getTestOurRobots()", Py_eval_input,
                    main_py_namespace_.ptr(), main_py_namespace_.ptr())));

                boost::python::list our_robots =
                    extract<boost::python::list>(our_robot_rtrn);

                const int NUM_COLS = 2;
                const int ROBOTS_PER_COL = kRobotsPerTeam / NUM_COLS;
                const int team_direction = context_->blue_team ? -1 : 1;
                for (int i = 0; i < kRobotsPerTeam; i++) {
                    auto* rob = replacement->add_robots();

                    if (i < len(our_robots)) {
                        boost::python::list robot =
                            extract<boost::python::list>(our_robots[i]);

                        float x = extract<float>(robot[0]);
                        float y = extract<float>(robot[1]);

                        rob->set_x(
                            -team_direction *
                            (y -
                             (FieldDimensions::current_dimensions.length() /
                              2)));
                        rob->set_y(team_direction * x);
                        rob->set_dir(extract<float>(robot[2]));
                    } else {
                        double x_pos =
                            team_direction * (2.5 - i / ROBOTS_PER_COL);
                        double y_pos =
                            i % ROBOTS_PER_COL - ROBOTS_PER_COL / NUM_COLS;
                        rob->set_x(x_pos);
                        rob->set_y(y_pos);
                        rob->set_dir(0);
                    }
                    rob->set_id(i);
                    rob->set_yellowteam(not context_->blue_team);
                }

                // Load TheirRobots information
                object their_robot_rtrn(handle<>(PyRun_String(
                    "ui.main._tests.getTestTheirRobots()", Py_eval_input,
                    main_py_namespace_.ptr(), main_py_namespace_.ptr())));

                boost::python::list their_robots =
                    extract<boost::python::list>(their_robot_rtrn);

                for (int i = 0; i < kRobotsPerTeam; i++) {
                    auto* rob = replacement->add_robots();

                    if (i < len(their_robots)) {
                        boost::python::list robot =
                            extract<boost::python::list>(their_robots[i]);

                        float x = extract<float>(robot[0]);
                        float y = extract<float>(robot[1]);

                        rob->set_x(
                            -team_direction *
                            (y -
                             (FieldDimensions::current_dimensions.length() /
                              2)));
                        rob->set_y(team_direction * x);
                        rob->set_dir(extract<float>(robot[2]));
                    } else {
                        double x_pos =
                            -team_direction * (2.5 - i / ROBOTS_PER_COL);
                        double y_pos =
                            i % ROBOTS_PER_COL - ROBOTS_PER_COL / NUM_COLS;
                        rob->set_x(x_pos);
                        rob->set_y(y_pos);
                        rob->set_dir(0);
                    }
                    rob->set_id(i);
                    rob->set_yellowteam(context_->blue_team);
                }

                // Get ball Information
                object ball_rtrn(handle<>(PyRun_String(
                    "ui.main._tests.getTestBall()", Py_eval_input,
                    main_py_namespace_.ptr(), main_py_namespace_.ptr())));

                boost::python::list ball =
                    extract<boost::python::list>(ball_rtrn);
                auto* ball_replace = replacement->mutable_ball();
                double posx = extract<double>(ball[0]);
                double posy = extract<double>(ball[1]);
                double velx = extract<double>(ball[2]);
                double vely = extract<double>(ball[3]);

                ball_replace->set_x(
                    -team_direction *
                    (posy -
                     (FieldDimensions::current_dimensions.length() / 2)));
                ball_replace->set_y(team_direction * posx);
                ball_replace->set_vx(-team_direction * vely);
                ball_replace->set_vy(team_direction * velx);

                context_->grsim_command = sim_packet;
            }
#endif

        } catch (const error_already_set&) {
            PyErr_Print();
            throw runtime_error("Error trying to load test");
        }
    }
    PyGILState_Release(state);
}
