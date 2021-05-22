from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import pyqtSlot, QPointF, QRectF, QLineF, Qt
from PyQt5.QtGui import QPainter, QPalette, QColor, QBrush, QPen, QTransform, QFont

import main_icons

import rclpy
import threading
import signal

from rj_msgs.srv import *
from rj_msgs.msg import *
from rj_geometry_msgs.msg import Point, Pose
from builtin_interfaces.msg import Duration

from log_control import TopicGrabber

import time
import enum
import sys

class GameInfo:
    def __init__(self, state: GameState, our_info: TeamInfo, their_info: TeamInfo, team_color: TeamColor):
        self.state = state
        self.our_info = our_info
        self.their_info = their_info
        self.team_color = team_color

class LogFrame:
    def __init__(self, info: GameInfo, dimensions: FieldDimensions, world_state: WorldState):
        self.info = info
        self.dimensions = dimensions
        self.world_state = world_state

class StatusPane(QtWidgets.QWidget):
    def __init__(self, node):
        super(StatusPane, self).__init__()
        uic.loadUi('Status.ui', self)
        self.update_team_info(TeamInfo(name="RoboJackets", score=1, num_red_cards=2), True)
        self.update_team_info(TeamInfo(name="UBC", score=3, num_red_cards=1), False)
        self.update_game_state(GameState(stage_time_left=Duration(sec=100)))

    def update_game_state(self, game_state):
        def stage_text(stage):
            return "Placeholder"

        def command_text(command):
            return "Placeholder"

        self.refStage.setText(stage_text(game_state.period))
        self.refCommand.setText(stage_text(game_state.state))
        self.refTimeLeft.setText(
            "{}:{:02}".format(
                game_state.stage_time_left.sec // 60,
                game_state.stage_time_left.sec % 60))

    def update_team_info(self, team_info, is_blue):
        team = 'Blue' if is_blue else 'Yellow'
        getattr(self, f"refName{team}").setText(str(team_info.name))
        getattr(self, f"refScore{team}").setText(str(team_info.score))
        getattr(self, f"refRedCards{team}").setText(str(team_info.num_red_cards))
        getattr(self, f"refYellowCards{team}").setText(str(team_info.num_yellow_cards))
        getattr(self, f"refTimeouts{team}").setText(str(team_info.timeouts_left))
        getattr(self, f"refGoalie{team}").setText(str(team_info.goalie_id))

    def set_log_frame(self, log_frame):
        self.update_team_info(log_frame.info.our_info, log_frame.info.team_color.is_blue)
        self.update_team_info(log_frame.info.our_info, not log_frame.info.team_color.is_blue)

class QuickRefPane(QtWidgets.QWidget):
    def __init__(self, node):
        super(QuickRefPane, self).__init__()
        uic.loadUi('QuickRef.ui', self)

        # ROS Setup
        self.quick_restart = node.create_client(QuickRestart, 'referee/quick_restart')
        self.quick_commands = node.create_client(QuickCommands, 'referee/quick_commands')

        # Qt5 Setup
        def quick_command(state):
            return lambda: self.quick_commands.call_async(QuickCommands.Request(state=state))

        self.fastHalt.clicked.connect(quick_command(QuickCommands.Request.COMMAND_HALT))
        self.fastStop.clicked.connect(quick_command(QuickCommands.Request.COMMAND_STOP))
        self.fastReady.clicked.connect(quick_command(QuickCommands.Request.COMMAND_READY))
        self.fastForceStart.clicked.connect(quick_command(QuickCommands.Request.COMMAND_PLAY))

        def quick_restart(restart, blue_team):
            return lambda: self.quick_restart.call_async(QuickRestart.Request(restart=restart, blue_team=blue_team))

        self.fastKickoffBlue.clicked.connect(quick_restart(QuickRestart.Request.RESTART_KICKOFF, True))
        self.fastDirectBlue.clicked.connect(quick_restart(QuickRestart.Request.RESTART_INDIRECT, True))
        self.fastIndirectBlue.clicked.connect(quick_restart(QuickRestart.Request.RESTART_INDIRECT, True))
        self.fastKickoffYellow.clicked.connect(quick_restart(QuickRestart.Request.RESTART_KICKOFF, False))
        self.fastDirectYellow.clicked.connect(quick_restart(QuickRestart.Request.RESTART_INDIRECT, False))
        self.fastIndirectYellow.clicked.connect(quick_restart(QuickRestart.Request.RESTART_INDIRECT, False))


class LogControlState(enum.Enum):
    LIVE = 0
    PAUSED = 1
    PLAY = 2

class LogControlPane(QtWidgets.QWidget):

    def __init__(self, node):
        super(LogControlPane, self).__init__()
        uic.loadUi('LogControl.ui', self)
        self.state = LogControlState.LIVE
        self.logHistoryLocation.valueChanged.connect(self.log_history_changed)

        self.start_time = node.get_clock().now()
        self.listeners = []

        self.world_state_grabber = TopicGrabber('/vision_filter/world_state', WorldState, node, self.new_message_callback)
        self.dimensions_grabber = TopicGrabber('/config/field_dimensions', FieldDimensions, node, self.new_message_callback)
        self.game_state_grabber = TopicGrabber('/referee/game_state', GameState, node, self.new_message_callback)
        self.our_info_grabber = TopicGrabber('/referee/our_info', TeamInfo, node, self.new_message_callback)
        self.their_info_grabber = TopicGrabber('/referee/their_info', TeamInfo, node, self.new_message_callback)
        self.team_color_grabber = TopicGrabber('/referee/team_color', TeamColor, node, self.new_message_callback)

        self.log_frame = LogFrame(GameInfo(GameState(), TeamInfo(), TeamInfo(), TeamColor()), FieldDimensions(), WorldState())

        self.clock = node.get_clock()
        self.start_time = None
        self.end_time = None

    def new_message_callback(self, message, stamp):
        if self.state is LogControlState.LIVE:
            if self.start_time is None:
                self.start_time = stamp

            if self.end_time is None:
                self.end_time = stamp

            self.end_time = max(stamp, self.end_time)
            self.logHistoryLocation.setMaximum((self.end_time - self.start_time) // 1e6)

            self.log_frame = LogFrame(
                GameInfo(
                    self.game_state_grabber.latest_or_default(),
                    self.our_info_grabber.latest_or_default(),
                    self.their_info_grabber.latest_or_default(),
                    self.team_color_grabber.latest_or_default(),
                ),
                self.dimensions_grabber.latest_or_default(),
                self.world_state_grabber.latest_or_default(),
            )
            self.on_change()

    def log_history_changed(self):
        self.set_time(self.logHistoryLocation.value())

    def set_time(self, time):
        self.state = LogControlState.PAUSED
        seconds = int(time) // 1000
        self.logTime.setText('{}:{:02}'.format(seconds // 60, seconds % 60))
        self.log_frame = self.get_log_frame(int(time * 1e6 + self.start_time))
        self.on_change()

    def set_listeners(self, listeners):
        self.listeners = listeners

    def on_change(self):
        for l in self.listeners:
            l(self.log_frame)

    def get_log_frame(self, time):
        print(self.our_info_grabber.get_index_for_time(time))
        print(len(self.our_info_grabber.keys), len(self.our_info_grabber.msgs))
        return LogFrame(
            GameInfo(
                self.game_state_grabber.get_msg_for_time(time),
                self.our_info_grabber.get_msg_for_time(time),
                self.their_info_grabber.get_msg_for_time(time),
                self.team_color_grabber.get_msg_for_time(time),
            ),
            self.dimensions_grabber.get_msg_for_time(time),
            self.world_state_grabber.get_msg_for_time(time))


class FieldView(QtWidgets.QWidget):
    def __init__(self, node):
        super(FieldView, self).__init__()

        self.transform = QTransform()
        self.text_rotation = 0

    def calculateTransform(self, size, dimensions):
        rotation = 0

        # Calculate ratios for height and width
        display_width_m, display_height_m = (dimensions.floor_length, dimensions.floor_width) if (rotation % 2 == 0) else (dimensions.floor_width, dimensions.floor_length)

        if display_width_m < 1e-6 or display_height_m < 1e-6:
            return

        self.pixels_per_meter = min(size.width() / display_width_m, size.height() / display_height_m)

        self.transform = QTransform().scale(self.pixels_per_meter, self.pixels_per_meter).translate(display_width_m / 2, display_height_m / 2).rotate(90.0 * rotation)
        self.text_rotation = -90.0 * rotation

    def setLogFrame(self, log_frame):
        self.log_frame = log_frame
        self.calculateTransform(self.size(), self.log_frame.dimensions)
        self.update()

    def resizeEvent(self, event):
        self.calculateTransform(event.size(), self.log_frame.dimensions)

    def drawBall(self, p, x, y):
        p.save()

        ball_radius = 0.0215

        p.translate(-self.log_frame.dimensions.length / 2, 0)
        p.rotate(90)
        p.translate(-x, -y)

        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(255, 128, 0)))
        p.drawEllipse(QRectF(-ball_radius, -ball_radius, 2 * ball_radius, 2 * ball_radius))

        p.restore()

    def drawWorldState(self, p, world_state, blue_team):
        for rid, robot in enumerate(world_state.our_robots):
            if robot.visible:
                self.drawRobot(p, rid, robot.pose.position.x, robot.pose.position.y, robot.pose.heading, blue_team)

        for rid, robot in enumerate(world_state.their_robots):
            if robot.visible:
                self.drawRobot(p, rid, robot.pose.position.x, robot.pose.position.y, robot.pose.heading, not blue_team)

        if world_state.ball.visible:
            self.drawBall(p, world_state.ball.position.x, world_state.ball.position.y)

    def drawField(self, p, dimensions):
        p.save()

        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(0, 100, 0)))
        p.drawRect(QRectF(-dimensions.floor_length / 2,
                          -dimensions.floor_width / 2,
                          dimensions.floor_length,
                          dimensions.floor_width))

        p.setPen(QPen(Qt.white, dimensions.line_width))
        p.setBrush(Qt.NoBrush)

        # Field edge
        p.drawRect(QRectF(-dimensions.length / 2, -dimensions.width / 2, dimensions.length, dimensions.width))

        # Center line
        p.drawLine(QLineF(0, dimensions.width / 2, 0, -dimensions.width / 2))

        # Center circle
        p.drawEllipse(QRectF(-dimensions.center_radius,
                             -dimensions.center_radius,
                             2 * dimensions.center_radius,
                             2 * dimensions.center_radius))

        # -x goal box
        p.drawRect(QRectF(-dimensions.length / 2, -dimensions.penalty_long_dist / 2,
                          dimensions.penalty_short_dist, dimensions.penalty_long_dist))

        # +x goal box
        p.drawRect(QRectF(dimensions.length / 2, -dimensions.penalty_long_dist / 2,
                          -dimensions.penalty_short_dist, dimensions.penalty_long_dist))

        p.restore()

    def drawText(self, p, x, y, text, color):
        p.save()

        p.translate(-self.log_frame.dimensions.length / 2, 0)
        p.rotate(90)
        p.translate(-x, -y)
        p.scale(0.009, 0.009)
        p.rotate(-90)
        p.rotate(self.text_rotation)

        p.setPen(QPen(color))
        p.drawText(QRectF(-10, -10, 20, 20), Qt.AlignCenter, text)
        p.restore()

    def drawRobot(self, p, robot_id, x, y, heading, is_blue):
        angle = -heading * 180 / 3.14159265
        p.save()

        p.translate(-self.log_frame.dimensions.length / 2, 0)
        p.rotate(90)
        p.translate(-x, -y)

        p.rotate(angle)
        p.setPen(QPen(QColor(0, 0, 0, 0)))
        p.setBrush(QBrush(QColor(0, 0, 255) if is_blue else QColor(255, 255, 0)))
        p.drawChord(QRectF(-0.1, -0.1, 0.2, 0.2), 30 * 16, 300 * 16)
        p.restore()

        self.drawText(p, x, y, str(robot_id), QColor(255, 255, 255) if is_blue else QColor(0, 0, 0))

    def paintEvent(self, event):
        p = QPainter(self)
        p.setTransform(self.transform)

        self.drawField(p, self.log_frame.dimensions)
        self.drawWorldState(p,
                            self.log_frame.world_state,
                            True)

        p.end()


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setWindowTitle("Soccer")

        self.shutdown_requested = False

        # ROS setup
        self.node = rclpy.create_node("GUINode")

        # Qt5 setup
        main_widget = QtWidgets.QWidget()

        layout = QtWidgets.QVBoxLayout()

        top = QtWidgets.QWidget()
        layout_top = QtWidgets.QHBoxLayout()
        self.quick_ref = QuickRefPane(self.node)
        self.log_control = LogControlPane(self.node)
        layout_top.addWidget(self.quick_ref)
        layout_top.addWidget(self.log_control)
        top.setLayout(layout_top)

        bottom = QtWidgets.QWidget()
        layout_bottom = QtWidgets.QHBoxLayout()
        self.status = StatusPane(self.node)
        self.field = FieldView(self.node)
        layout_bottom.addWidget(self.status, 0)
        layout_bottom.addWidget(self.field, 1)
        bottom.setLayout(layout_bottom)

        layout.addWidget(top, 0)
        layout.addWidget(bottom, 1)
        layout.setContentsMargins(0, 0, 0, 0)

        self.log_control.set_listeners([self.field.setLogFrame, self.status.set_log_frame])
        self.log_control.on_change()

        main_widget.setLayout(layout)
        self.setCentralWidget(main_widget)

    def closeEvent(self, _):
        self.shutdown_requested = True

    def spin(self):
        while rclpy.ok() and not self.shutdown_requested:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_node()

def main():
    rclpy.init()

    app = QtWidgets.QApplication(sys.argv)

    main = MainWindow()

    # Handle SIGINT
    signal.signal(
        signal.SIGINT,
        lambda unused_signal, unused_frame: main.close()
    )

    # ROS spinner
    ros_thread = threading.Thread(target=main.spin)
    ros_thread.start()

    main.show()

    app.exec_()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
