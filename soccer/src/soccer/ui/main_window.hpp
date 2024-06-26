#pragma once

#include <mutex>
#include <optional>

#include <QComboBox>
#include <QMainWindow>
#include <QPushButton>
#include <QComboBox>
#include <QTime>
#include <QTimer>
#include <QtGui/QStandardItemModel>
#include <rclcpp/rclcpp.hpp>

#include <rj_convert/ros_convert.hpp>
#include <rj_msgs/srv/quick_commands.hpp>
#include <rj_msgs/srv/quick_restart.hpp>
#include <rj_msgs/srv/set_game_settings.hpp>
#include <rj_msgs/msg/override_position.hpp>

#include "field_view.hpp"
#include "game_state.hpp"
#include "processor.hpp"
#include "strategy/agent/position/overriding_positions.hpp"
#include "ui_MainWindow.h"

#include "rc-fshare/rtp.hpp"

class TestResultTab;
class StripChart;

namespace {
// Style sheets used for live/non-live controls
QString LiveStyle("border:2px solid transparent");
QString NonLiveStyle("border:2px solid red");
};  // namespace

enum Side { Yellow, Blue };

/**
 * main gui thread class
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(Processor* processor, bool has_external_ref, QWidget* parent = nullptr);

    void initialize();

    /// Deselects all debug layers
    void allDebugOff();

    /// Selects all debug layers
    void allDebugOn();

    bool live();

    void setLive() {
        if (!live()) {
            _ui.logTree->setStyleSheet(QString("QTreeWidget{%1}").arg(NonLiveStyle));
            _playbackRate = std::nullopt;
        }
    }

    void setPlayBackRate(double playbackRate) {
        if (live()) {
            _ui.logTree->setStyleSheet(QString("QTreeWidget{%1}").arg(LiveStyle));
        }
        _playbackRate = playbackRate;
    }

    int frameNumber() const { return roundf(_doubleFrameNumber); }

    void frameNumber(int value) { _doubleFrameNumber = value; }

    // Call this to update the status bar when the log file has changed
    void logFileChanged();

    QTimer updateTimer;

    void setUseRefChecked(bool use_ref);

private Q_SLOTS:
    void addLayer(int i, const QString& name, bool checked);
    void updateViews();

    void on_fieldView_robotSelected(int shell);
    void on_actionRawBalls_toggled(bool state);
    void on_actionRawRobots_toggled(bool state);
    void on_actionCoords_toggled(bool state);
    void on_actionDotPatterns_toggled(bool state);
    void on_actionTeam_Names_toggled(bool state);
    void on_actionTeamYellow_triggered();
    void on_actionTeamBlue_triggered();
    void on_manualID_currentIndexChanged(int value);
    void on_goalieID_currentIndexChanged(int value);

    void on_actionUse_Field_Oriented_Controls_toggled(bool value);
    void on_actionUse_Multiple_Joysticks_toggled(bool value);

    /// Field side
    void on_actionDefendPlusX_triggered();
    void on_actionDefendMinusX_triggered();
    void on_actionUseOurHalf_toggled(bool value);
    void on_actionUseOpponentHalf_toggled(bool value);

    /// Field rotation
    void on_action0_triggered();
    void on_action90_triggered();
    void on_action180_triggered();
    void on_action270_triggered();

    /// Simulator commands
    void on_actionCenterBall_triggered();
    void on_actionStopBall_triggered();
    void on_actionResetField_triggered();
    void on_actionStopRobots_triggered();
    void on_actionQuicksaveRobotLocations_triggered();
    void on_actionQuickloadRobotLocations_triggered();

    /// Style Sheets
    void on_actionNoneStyle_triggered();
    void on_actionDarkStyle_triggered();
    void on_actionDarculizedStyle_triggered();
    void on_action1337h4x0rStyle_triggered();
    void on_actionNyanStyle_triggered();

    /// Manual control commands
    void on_actionDampedRotation_toggled(bool value);
    void on_actionDampedTranslation_toggled(bool value);

    /// Debug menu commands
    void on_actionRestartUpdateTimer_triggered();
    void on_actionStart_Logging_triggered();

    /// Gameplay menu
    void on_actionSeed_triggered();

    // Joystick settings
    void on_joystickKickOnBreakBeam_stateChanged();

    /// Log controls
    void on_logHistoryLocation_sliderMoved(int value);
    void on_logHistoryLocation_sliderReleased();
    void on_logHistoryLocation_sliderPressed();
    void on_logPlaybackRewind_clicked();
    void on_logPlaybackPrevFrame_clicked();
    void on_logPlaybackPause_clicked();
    void on_logPlaybackNextFrame_clicked();
    void on_logPlaybackPlay_clicked();
    void on_logPlaybackLive_clicked();

    /// Debug layers
    void on_debugLayers_itemChanged(QListWidgetItem* item);
    void on_debugLayers_customContextMenuRequested(const QPoint& pos);

    // Fast Ref Buttons
    void on_fastHalt_clicked();
    void on_fastStop_clicked();
    void on_fastReady_clicked();
    void on_fastForceStart_clicked();
    void on_fastKickoffBlue_clicked();
    void on_fastKickoffYellow_clicked();
    void on_fastBlue_clicked();
    void on_fastYellow_clicked();

    // Robot Position Dropdowns and Reset Buttons
    void on_robotPosition_0_currentIndexChanged(int value);
    void on_robotPosition_1_currentIndexChanged(int value);
    void on_robotPosition_2_currentIndexChanged(int value);
    void on_robotPosition_3_currentIndexChanged(int value);
    void on_robotPosition_4_currentIndexChanged(int value);
    void on_robotPosition_5_currentIndexChanged(int value);
    void on_robotPosition_6_currentIndexChanged(int value);
    void on_robotPosition_7_currentIndexChanged(int value);
    void on_robotPosition_8_currentIndexChanged(int value);
    void on_robotPosition_9_currentIndexChanged(int value);
    void on_robotPosition_10_currentIndexChanged(int value);
    void on_robotPosition_11_currentIndexChanged(int value);
    void on_robotPosition_12_currentIndexChanged(int value);
    void on_robotPosition_13_currentIndexChanged(int value);
    void on_robotPosition_14_currentIndexChanged(int value);
    void on_robotPosition_15_currentIndexChanged(int value);

    void on_positionReset_0_clicked();
    void on_positionReset_1_clicked();
    void on_positionReset_2_clicked();
    void on_positionReset_3_clicked();
    void on_positionReset_4_clicked();
    void on_positionReset_5_clicked();
    void on_positionReset_6_clicked();
    void on_positionReset_7_clicked();
    void on_positionReset_8_clicked();
    void on_positionReset_9_clicked();
    void on_positionReset_10_clicked();
    void on_positionReset_11_clicked();
    void on_positionReset_12_clicked();
    void on_positionReset_13_clicked();
    void on_positionReset_14_clicked();
    void on_positionReset_15_clicked();

Q_SIGNALS:
    // signal used to let widgets that we're viewing a different log frame now
    int historyLocationChanged(int value);

private:
    void updateStatus();
    void updateFromRefPacket(bool haveExternalReferee);
    static std::string formatLabelBold(Side side, const std::string& label);

    enum class StatusType { Status_OK, Status_Warning, Status_Fail };

    void status(const QString& text, StatusType status);
    void updateRadioBaseStatus(bool usbRadio);
    void channel(int n);
    void updateDebugLayers(const Packet::LogFrame& frame);

    Ui_MainWindow _ui{};
    const QStandardItemModel* goalieModel{};

    Processor* const _processor;
    bool _has_external_ref;

    // Log history, copied from Logger.
    // This is used by other controls to get log data without having to copy it
    // again from the Logger.
    std::vector<std::shared_ptr<Packet::LogFrame>> _history{};

    // Longer log history, copied from Logger.
    // This is used specificially via StripChart and ProtobufTree
    // To export a larger amount of data.
    std::vector<std::shared_ptr<Packet::LogFrame>> _longHistory{};

    // Arrays containing dropdown and reset button UI objects
    std::array<QComboBox*, kNumShells> robot_pos_selectors{};
    std::array<QPushButton*, kNumShells> position_reset_buttons{};

    // Methods to update positions and broadcast those changes
    /*
     * Sets which dropdown represents the goalie — disabling it and ensuring the previous goalie
     * dropdown is re-enabled.
     */
    void setGoalieDropdown(int robot);
    /*
     * Callback when a dropdown is changed (whether by user or program)
     */
    void onPositionDropdownChanged(int robot, int position_number);
    /*
     * Callback when a reset button is clicked
     */
    void onResetButtonClicked(int robot);
    // TODO: Add a publisher and a subscriber to implement these

    // Tree items that are not in LogFrame
    QTreeWidgetItem* _frameNumberItem{};
    QTreeWidgetItem* _elapsedTimeItem{};

    /// playback rate of the viewer - a value of 1 means realtime
    std::optional<double> _playbackRate;

    // This is used to update some status items less frequently than the full
    // field view
    int _updateCount;

    // Tracking fractional frames is the easiest way to allow arbitrary playback
    // rates. To keep rounding consistent, only access this with frameNumber().
    double _doubleFrameNumber;

    RJ::Time _lastUpdateTime;

    QLabel* _currentPlay{};
    QLabel* _logFile{};
    QLabel* _viewFPS{};
    QLabel* _procFPS{};
    QLabel* _logMemory{};

    // QActionGroups for Radio Menu Actions
    std::map<std::string, QActionGroup*> qActionGroups{};

    // maps robot shell IDs to items in the list
    std::map<int, std::unique_ptr<QListWidgetItem>> _robotStatusItemMap{};

    /// the play, pause, ffwd, etc buttons
    std::vector<QPushButton*> _logPlaybackButtons{};

    std::vector<QComboBox*> _robotConfigQComboBoxes{};

    std::vector<QComboBox*> _robotDebugResponseQComboBoxes{};

    std::mutex* context__mutex;
    Context* context_;

    // ROS Compatibility stuff
    void send_quick_command(const PlayState& state);

    rclcpp::Node::SharedPtr _node;
    rclcpp::Client<rj_msgs::srv::QuickCommands>::SharedPtr _quick_commands_srv;
    rclcpp::Client<rj_msgs::srv::SetGameSettings>::SharedPtr _set_game_settings;
    rclcpp::Publisher<rj_msgs::msg::OverridePosition>::SharedPtr test_play_pub_;
    rclcpp::executors::SingleThreadedExecutor _executor;
    std::thread _executor_thread;

    /**
     * Queued command from fast restarts. When the last command was halt/stop/force start/ready,
     * this is nullopt. When the last command was a kickoff setup, this is the kickoff command (a
     * ready command will be sent next).
     *
     * Technically if we change our team color between setting queued_command_ and actually sending
     * it, the restart will be for the wrong team, but this really never matters.
     */
    std::optional<PlayState> queued_command_;

    rj_msgs::msg::GameSettings _game_settings;
    bool _game_settings_valid = false;

    std::vector<std::string> overriding_position_labels {
        "Auto",
        "Offense",
        "Defense",
        "Free Kicker",
        "Penalty Player",
        "Penalty Non-Kicker",
        "Solo Offense",
        "Smart Idle",
        "Zoner",
        "Idle"
    };
};
