"""Shared constants mirroring rj_constants/constants.hpp.

This module provides a single source of truth for physical and geometric
constants used across the RL system. Values are kept in sync with the C++
constants defined in src/rj_constants/include/rj_constants/constants.hpp
and the network configuration in src/rj_common/include/rj_common/network.hpp.

All distances are in meters, times in seconds, weights in kilograms.
"""

# ---------------------------------------------------------------------------
# Robot constants (from constants.hpp)
# ---------------------------------------------------------------------------
NUM_SHELLS = 16
ROBOTS_PER_TEAM = 6

ROBOT_DIAMETER = 0.180
ROBOT_RADIUS = ROBOT_DIAMETER / 2.0
ROBOT_HEIGHT = 0.150
ROBOT_MOUTH_WIDTH = 0.0635
ROBOT_MOUTH_RADIUS = 0.078

# ---------------------------------------------------------------------------
# Ball constants (from constants.hpp)
# ---------------------------------------------------------------------------
BALL_DIAMETER = 0.043
BALL_RADIUS = BALL_DIAMETER / 2.0
BALL_MASS = 0.048
BALL_DECEL = -0.4  # m/s^2

# ---------------------------------------------------------------------------
# Default SSL Division B field dimensions (metres)
# These match the values used in grSim and the C++ FieldDimensions class.
# ---------------------------------------------------------------------------
FIELD_LENGTH = 9.0
FIELD_WIDTH = 6.0
GOAL_WIDTH = 1.0
GOAL_DEPTH = 0.18
GOAL_HEIGHT = 0.16
BORDER_WIDTH = 0.3
CENTER_CIRCLE_RADIUS = 0.5
LINE_WIDTH = 0.01

# Derived field geometry
HALF_FIELD_LENGTH = FIELD_LENGTH / 2.0
HALF_FIELD_WIDTH = FIELD_WIDTH / 2.0
HALF_GOAL_WIDTH = GOAL_WIDTH / 2.0

# Goal line x-coordinates (blue defends left, yellow defends right)
BLUE_GOAL_X = -HALF_FIELD_LENGTH
YELLOW_GOAL_X = HALF_FIELD_LENGTH

# ---------------------------------------------------------------------------
# Simulator network configuration (from network.hpp)
# ---------------------------------------------------------------------------
SIM_VISION_PORT = 10020
SIM_BLUE_STATUS_PORT = 30011
SIM_YELLOW_STATUS_PORT = 30012
SIM_COMMAND_PORT = 10300
SIM_BLUE_COMMAND_PORT = 10301
SIM_YELLOW_COMMAND_PORT = 10302
SIM_DEFAULT_ADDRESS = "127.0.0.1"

# ---------------------------------------------------------------------------
# Simulator physics defaults (from sim_params.yaml)
# ---------------------------------------------------------------------------
SIM_BALL_DECAY_CONSTANT = 0.18
SIM_ROBOT_MAX_SPEED = 2.0  # m/s
SIM_ROBOT_MAX_ACCELERATION = 2.0  # m/s^2
SIM_ROBOT_MAX_KICK_SPEED = 7.0  # m/s
SIM_ROBOT_MAX_CHIP_SPEED = 4.0  # m/s
SIM_ROBOT_MAX_ROTATIONAL_SPEED = 3.0  # rad/s
SIM_DT = 1.0 / 120.0  # 120 Hz vision update rate
