# ======================================================================
# Set Sources
# ======================================================================
set(SOCCER_UIS
        ui/qt/ConfigFileTab.ui
        ui/qt/LogViewer.ui
        ui/qt/MainWindow.ui
        ui/qt/RefereeTab.ui
        ui/qt/RobotStatusWidget.ui)

set(SOCCER_RSRC ui/qt/main_icons.qrc)

set(ROBOCUP_LIB_SRC
    battery_profile.cpp
    configuration.cpp
    gameplay/gameplay_module.cpp
    gameplay/robocup-py.cpp
    joystick/sdl_joystick_node.cpp
    joystick/sdl_gamepad.cpp
    joystick/manual_control_node.cpp
    kick_evaluator.cpp
    logger.cpp
    control/manipulator_control.cpp
    control/motion_control.cpp
    control/motion_control_node.cpp
    control/trapezoidal_motion.cpp
    optimization/gradient_ascent_1d.cpp
    optimization/parallel_gradient_ascent_1d.cpp
    optimization/nelder_mead_2d.cpp
    optimization/python_function_wrapper.cpp
    planning/primitives/angle_planning.cpp
    planning/primitives/create_path.cpp
    planning/primitives/path_smoothing.cpp
    planning/primitives/replanner.cpp
    planning/primitives/rrt_util.cpp
    planning/primitives/trapezoidal_motion.cpp
    planning/primitives/velocity_profiling.cpp
    planning/planner/collect_planner.cpp
    planning/planner/escape_obstacles_path_planner.cpp
    planning/planner/intercept_planner.cpp
    planning/planner/line_kick_planner.cpp
    planning/planner/path_target_planner.cpp
    planning/planner/pivot_path_planner.cpp
    planning/planner/plan_request.cpp
    planning/planner/settle_planner.cpp
    planning/planner_node.cpp
    planning/trajectory.cpp
    planning/trajectory_utils.cpp
    processor.cpp
    radio/network_radio.cpp
    radio/packet_convert.cpp
    radio/sim_radio.cpp
    radio/radio.cpp
    referee/referee_base.cpp
    referee/external_referee.cpp
    referee/internal_referee.cpp
    robot.cpp
    robot_config.cpp
    system_state.cpp
    debug_drawer.cpp
    ui/battery_widget.cpp
    ui/field_view.cpp
    ui/main_window.cpp
    ui/protobuf_tree.cpp
    ui/referee_tab.cpp
    ui/robot_status_widget.cpp
    ui/robot_widget.cpp
    ui/sim_field_view.cpp
    ui/strip_chart.cpp
    ui/style_sheet_manager.cpp
    gr_sim_communicator.cpp
    world_state.cpp
    ros2_temp/soccer_config_client.cpp
    ros2_temp/raw_vision_packet_sub.cpp
    ros2_temp/referee_sub.cpp
    ros2_temp/autonomy_interface.cpp
    ros2_temp/debug_draw_interface.cpp
    window_evaluator.cpp)

set(SOCCER_TEST_SRC
    battery_profile_test.cpp
    ball_test.cpp
    kick_evaluator_test.cpp
    control/trapezoidal_motion_test.cpp
    control/motion_control_test.cpp
    optimization/gradient_ascent_1d_test.cpp
    optimization/parallel_gradient_ascent_1d_test.cpp
    optimization/nelder_mead_2d_test.cpp
    planning/tests/angle_planning_test.cpp
    planning/tests/bezier_path_test.cpp
    planning/tests/planner_test.cpp
    planning/tests/create_path_test.cpp
    planning/tests/testing_utils.cpp
    planning/tests/trajectory_test.cpp
    planning/tests/trapezoidal_motion_test.cpp
    planning/tests/velocity_profiling_test.cpp
    test_main.cpp
    window_evaluator_test.cpp
    logger_test.cpp)

set(LOG_VIEWER_RSRC ui/qt/log_icons.qrc)

set(LOG_VIEWER_UIS ui/qt/LogViewer.ui)

set(LOG_VIEWER_SRC log_viewer.cpp)

set(EXTERNAL_REFEREE_NODE_SRC referee/external_referee_main.cpp)

set(INTERNAL_REFEREE_NODE_SRC referee/internal_referee_main.cpp)

set(PLANNER_NODE_SRC planning/planner_node_main.cpp)

set(CONTROL_NODE_SRC control/control_node_main.cpp)

set(SIM_RADIO_NODE_SRC radio/sim_radio_node_main.cpp)

set(NETWORK_RADIO_NODE_SRC radio/network_radio_node_main.cpp)

# ======================================================================
# Add sources
# ======================================================================
# ---- robocup ----
target_sources(robocup PRIVATE ${ROBOCUP_LIB_SRC} ${SOCCER_UIS} ${SOCCER_RSRC})

# Set the directory so AUTOUIC know where to find .ui files
set(UI_PATH ${CMAKE_CURRENT_SOURCE_DIR}/ui/qt)
set_target_properties(robocup PROPERTIES AUTOUIC_SEARCH_PATHS ${UI_PATH})

# ---- soccer ----
target_sources(soccer PRIVATE main.cpp)

# ---- log_viewer ----
target_sources(log_viewer PRIVATE ${LOG_VIEWER_SRC} ${LOG_VIEWER_UIS} ${LOG_VIEWER_RSRC})

# ---- external_referee_node ----
target_sources(external_referee_node PRIVATE ${EXTERNAL_REFEREE_NODE_SRC})

# ---- internal_referee_node ----
target_sources(internal_referee_node PRIVATE ${INTERNAL_REFEREE_NODE_SRC})

# ---- planner_node ----
target_sources(planner_node PRIVATE ${PLANNER_NODE_SRC})

# ---- control_node ----
target_sources(control_node PRIVATE ${CONTROL_NODE_SRC})

# ---- sim_radio_node ----
target_sources(sim_radio_node PRIVATE ${SIM_RADIO_NODE_SRC})

# ---- sim_radio_node ----
target_sources(network_radio_node PRIVATE ${NETWORK_RADIO_NODE_SRC})

# Set the directory so AUTOUIC know where to find .ui files
set_target_properties(log_viewer PROPERTIES AUTOUIC_SEARCH_PATHS ${UI_PATH})

# ---- testing ----
if(BUILD_TESTING)
  target_sources(test-soccer PRIVATE ${SOCCER_TEST_SRC})
endif()
