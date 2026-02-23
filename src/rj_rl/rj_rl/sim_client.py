"""gRSim simulator client for the RL environment.

Provides a Python interface to the grSim (or ER-Force) SSL simulator,
reusing the same protobuf protocol and network ports as the existing
C++ sim_radio. This allows the RL system to use the existing simulator
as its physics engine rather than duplicating physics internally.

Communication follows the same pattern as src/rj_radio/src/sim_radio.cpp:
  - Robot commands sent via UDP protobuf (RobotControl)
  - World state received via SSL-Vision UDP multicast (SSL_WrapperPacket)
  - Simulator control (teleport/reset) via SimulatorCommand
"""
import socket
import struct
from dataclasses import dataclass, field as dataclass_field
from typing import Dict, List, Optional, Tuple

import numpy as np

from . import constants
from .proto_gen import ssl_simulation_robot_control_pb2 as robot_control_pb
from .proto_gen import ssl_simulation_control_pb2 as sim_control_pb
from .proto_gen import ssl_gc_common_pb2 as gc_common_pb
from .proto_gen import ssl_vision_wrapper_pb2 as vision_pb
from .proto_gen import ssl_vision_detection_pb2 as detection_pb


@dataclass
class SimRobotState:
    """Robot state as received from the simulator vision."""

    robot_id: int = 0
    pos: np.ndarray = dataclass_field(
        default_factory=lambda: np.zeros(2, dtype=np.float32)
    )
    orientation: float = 0.0
    confidence: float = 0.0


@dataclass
class SimBallState:
    """Ball state as received from the simulator vision."""

    pos: np.ndarray = dataclass_field(
        default_factory=lambda: np.zeros(2, dtype=np.float32)
    )
    confidence: float = 0.0


@dataclass
class SimWorldState:
    """Complete world state snapshot from the simulator."""

    blue_robots: Dict[int, SimRobotState] = dataclass_field(default_factory=dict)
    yellow_robots: Dict[int, SimRobotState] = dataclass_field(default_factory=dict)
    ball: SimBallState = dataclass_field(default_factory=SimBallState)
    timestamp: float = 0.0


class GrSimClient:
    """UDP client for communicating with the grSim SSL simulator.

    Mirrors the communication pattern used by the C++ SimRadio class
    (src/rj_radio/src/sim_radio.cpp) for sending robot commands and
    simulator control messages, and receives vision data via the
    SSL-Vision protocol.

    Args:
        blue_team: Whether we control the blue team.
        sim_address: Simulator IP address.
        vision_port: Port for receiving SSL-Vision data.
        command_port: Port for sending robot commands.
        sim_control_port: Port for simulator control (teleport etc.).
    """

    def __init__(
        self,
        blue_team: bool = True,
        sim_address: str = constants.SIM_DEFAULT_ADDRESS,
        vision_port: int = constants.SIM_VISION_PORT,
        command_port: Optional[int] = None,
        sim_control_port: int = constants.SIM_COMMAND_PORT,
    ):
        self.blue_team = blue_team
        self.sim_address = sim_address
        self.vision_port = vision_port
        self.sim_control_port = sim_control_port

        if command_port is None:
            self.command_port = (
                constants.SIM_BLUE_COMMAND_PORT
                if blue_team
                else constants.SIM_YELLOW_COMMAND_PORT
            )
        else:
            self.command_port = command_port

        # UDP sockets
        self._cmd_socket: Optional[socket.socket] = None
        self._vision_socket: Optional[socket.socket] = None
        self._control_socket: Optional[socket.socket] = None
        self._connected = False

        # Velocity tracking for state estimation
        self._prev_world: Optional[SimWorldState] = None
        self._ball_vel = np.zeros(2, dtype=np.float32)
        self._robot_vels: Dict[int, np.ndarray] = {}

    def connect(self) -> None:
        """Open UDP sockets for communication with the simulator."""
        # Command socket (send robot commands)
        self._cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cmd_socket.setblocking(False)

        # Vision socket (receive world state)
        self._vision_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._vision_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._vision_socket.bind((self.sim_address, self.vision_port))
        self._vision_socket.setblocking(False)

        # Control socket (send teleport/reset commands)
        self._control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._control_socket.setblocking(False)

        self._connected = True

    def close(self) -> None:
        """Close all UDP sockets."""
        for sock in [self._cmd_socket, self._vision_socket, self._control_socket]:
            if sock is not None:
                sock.close()
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    def send_robot_commands(
        self,
        commands: List[Tuple[int, float, float, float, float, float]],
    ) -> None:
        """Send velocity commands to robots.

        Each command is (robot_id, vx, vy, v_angular, kick_speed, dribbler_speed).
        Uses the same RobotControl protobuf as sim_radio.cpp.

        Args:
            commands: List of (id, vx, vy, angular, kick_speed, dribbler_speed).
        """
        if self._cmd_socket is None:
            return

        packet = robot_control_pb.RobotControl()
        for robot_id, vx, vy, v_angular, kick_speed, dribbler_speed in commands:
            cmd = packet.robot_commands.add()
            cmd.id = robot_id
            move = cmd.move_command.global_velocity
            move.x = vx
            move.y = vy
            move.angular = v_angular
            if kick_speed > 0:
                cmd.kick_speed = kick_speed
                cmd.kick_angle = 0.0
            if dribbler_speed > 0:
                cmd.dribbler_speed = dribbler_speed

        data = packet.SerializeToString()
        self._cmd_socket.sendto(
            data, (self.sim_address, self.command_port)
        )

    def teleport_ball(
        self,
        x: float,
        y: float,
        vx: float = 0.0,
        vy: float = 0.0,
    ) -> None:
        """Teleport the ball to a new position.

        Uses the same SimulatorCommand protobuf as sim_radio.cpp
        send_sim_command().

        Args:
            x: Ball x position in meters.
            y: Ball y position in meters.
            vx: Ball x velocity in m/s.
            vy: Ball y velocity in m/s.
        """
        if self._control_socket is None:
            return

        cmd = sim_control_pb.SimulatorCommand()
        ball = cmd.control.teleport_ball
        ball.x = x
        ball.y = y
        ball.z = 0.0
        ball.vx = vx
        ball.vy = vy
        ball.vz = 0.0
        ball.teleport_safely = True

        data = cmd.SerializeToString()
        self._control_socket.sendto(
            data, (self.sim_address, self.sim_control_port)
        )

    def teleport_robot(
        self,
        robot_id: int,
        blue_team: bool,
        x: float,
        y: float,
        orientation: float = 0.0,
        present: bool = True,
    ) -> None:
        """Teleport a robot to a new position.

        Uses the same SimulatorCommand protobuf as sim_radio.cpp.

        Args:
            robot_id: Robot shell ID.
            blue_team: Whether the robot is on the blue team.
            x: Robot x position in meters.
            y: Robot y position in meters.
            orientation: Robot heading in radians.
            present: Whether robot should be present on the field.
        """
        if self._control_socket is None:
            return

        cmd = sim_control_pb.SimulatorCommand()
        robot = cmd.control.teleport_robot.add()
        robot.id.id = robot_id
        robot.id.team = (
            gc_common_pb.BLUE if blue_team else gc_common_pb.YELLOW
        )
        robot.x = x
        robot.y = y
        robot.orientation = orientation
        robot.v_x = 0.0
        robot.v_y = 0.0
        robot.v_angular = 0.0
        robot.present = present

        data = cmd.SerializeToString()
        self._control_socket.sendto(
            data, (self.sim_address, self.sim_control_port)
        )

    def receive_world_state(self) -> Optional[SimWorldState]:
        """Receive the latest world state from the simulator vision.

        Reads all pending vision packets and returns the most recent
        world state. Returns None if no data is available.

        Returns:
            SimWorldState or None.
        """
        if self._vision_socket is None:
            return None

        latest = None
        while True:
            try:
                data, _ = self._vision_socket.recvfrom(65536)
                packet = vision_pb.SSL_WrapperPacket()
                packet.ParseFromString(data)
                if packet.HasField("detection"):
                    latest = self._parse_detection(packet.detection)
            except BlockingIOError:
                break

        if latest is not None:
            self._estimate_velocities(latest)
            self._prev_world = latest

        return latest

    def _parse_detection(
        self, frame: detection_pb.SSL_DetectionFrame
    ) -> SimWorldState:
        """Parse a vision detection frame into SimWorldState."""
        world = SimWorldState(timestamp=frame.t_capture)

        for robot in frame.robots_blue:
            state = SimRobotState(
                robot_id=robot.robot_id,
                pos=np.array(
                    [robot.x / 1000.0, robot.y / 1000.0], dtype=np.float32
                ),
                orientation=robot.orientation if robot.HasField("orientation") else 0.0,
                confidence=robot.confidence,
            )
            world.blue_robots[robot.robot_id] = state

        for robot in frame.robots_yellow:
            state = SimRobotState(
                robot_id=robot.robot_id,
                pos=np.array(
                    [robot.x / 1000.0, robot.y / 1000.0], dtype=np.float32
                ),
                orientation=robot.orientation if robot.HasField("orientation") else 0.0,
                confidence=robot.confidence,
            )
            world.yellow_robots[robot.robot_id] = state

        if frame.balls:
            ball = frame.balls[0]
            world.ball = SimBallState(
                pos=np.array(
                    [ball.x / 1000.0, ball.y / 1000.0], dtype=np.float32
                ),
                confidence=ball.confidence,
            )

        return world

    def _estimate_velocities(self, current: SimWorldState) -> None:
        """Estimate velocities from consecutive vision frames."""
        if self._prev_world is None:
            return

        dt = current.timestamp - self._prev_world.timestamp
        if dt <= 0:
            return

        # Ball velocity
        self._ball_vel = (
            (current.ball.pos - self._prev_world.ball.pos) / dt
        ).astype(np.float32)

        # Robot velocities (blue team)
        for rid, robot in current.blue_robots.items():
            if rid in self._prev_world.blue_robots:
                prev = self._prev_world.blue_robots[rid]
                self._robot_vels[rid] = (
                    (robot.pos - prev.pos) / dt
                ).astype(np.float32)

    def get_ball_velocity(self) -> np.ndarray:
        """Get estimated ball velocity from vision tracking."""
        return self._ball_vel.copy()

    def get_robot_velocity(self, robot_id: int) -> np.ndarray:
        """Get estimated robot velocity from vision tracking."""
        return self._robot_vels.get(
            robot_id, np.zeros(2, dtype=np.float32)
        ).copy()
