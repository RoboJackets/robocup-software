#include "main_window.hpp"

#include <ctime>
#include <fstream>

#include <QActionGroup>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include <QString>
#include <boost/algorithm/string.hpp>
#include <google/protobuf/descriptor.h>
#include <spdlog/spdlog.h>

#include <rj_common/qt_utils.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_protos/grSim_Packet.pb.h>
#include <rj_protos/grSim_Replacement.pb.h>
#include <ui/style_sheet_manager.hpp>
#include <ui_MainWindow.h>

#include "battery_profile.hpp"
#include "game_state.hpp"
#include "radio/radio.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_status_widget.hpp"
#include "std_msgs/msg/string.hpp"

#include "rc-fshare/git_version.hpp"

using namespace std;
using namespace boost;
using namespace google::protobuf;
using namespace Packet;
using namespace Eigen;

constexpr int kHistorySize = 60 * 2;
constexpr int kLongHistorySize = 60 * 60 * 30;

static const std::vector<QString> defaultHiddenLayers{
    "MotionControl", "Global Obstacles", "Local Obstacles", "Planning0", "Planning1",
    "Planning2",     "Planning3",        "Planning4",       "Planning5"};

void calcMinimumWidth(QWidget* widget, const QString& text) {
    QRect rect = QFontMetrics(widget->font()).boundingRect(text);
    widget->setMinimumWidth(rect.width());
}

MainWindow::MainWindow(Processor* processor, bool has_external_ref, QWidget* parent)
    : QMainWindow(parent),
      _updateCount(0),
      _doubleFrameNumber(-1),
      _lastUpdateTime(RJ::now()),
      _processor(processor),
      context_(processor->context()),
      _has_external_ref(has_external_ref),
      _game_settings(rj_convert::convert_to_ros(context_->game_settings)) {
    context__mutex = processor->loop_mutex();

    qRegisterMetaType<QVector<int>>("QVector<int>");
    _ui.setupUi(this);
    _ui.fieldView->history(&_history);

    _ui.logTree->history(&_longHistory);
    _ui.logTree->mainWindow = this;
    _ui.logTree->updateTimer = &updateTimer;

    // Initialize live/non-live control styles

    _logFile = new QLabel(this);
    _logFile->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    _logFile->setToolTip("Log File");
    statusBar()->addPermanentWidget(_logFile);

    _viewFPS = new QLabel(this);
    _viewFPS->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    _viewFPS->setToolTip("Display Framerate");
    calcMinimumWidth(_viewFPS, "View: 00.0 fps");
    statusBar()->addPermanentWidget(_viewFPS);

    _procFPS = new QLabel(this);
    _procFPS->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    _procFPS->setToolTip("Processing Framerate");
    calcMinimumWidth(_procFPS, "Proc: 00.0 fps");
    statusBar()->addPermanentWidget(_procFPS);

    _logMemory = new QLabel(this);
    _logMemory->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    _logMemory->setToolTip("Log Memory Usage");
    _logMemory->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    calcMinimumWidth(_logMemory, "Log: 000000/000000 000000 kiB");
    statusBar()->addPermanentWidget(_logMemory);

    _frameNumberItem = new QTreeWidgetItem(_ui.logTree);
    _frameNumberItem->setText(ProtobufTree::Column_Field, "Frame");
    _frameNumberItem->setData(ProtobufTree::Column_Tag, Qt::DisplayRole, -2);

    _elapsedTimeItem = new QTreeWidgetItem(_ui.logTree);
    _elapsedTimeItem->setText(ProtobufTree::Column_Field, "Elapsed Time");
    _elapsedTimeItem->setData(ProtobufTree::Column_Tag, Qt::DisplayRole, -1);

    _ui.debugLayers->setContextMenuPolicy(Qt::CustomContextMenu);

    QActionGroup* teamGroup = new QActionGroup(this);
    teamGroup->addAction(_ui.actionTeamBlue);
    teamGroup->addAction(_ui.actionTeamYellow);
    qActionGroups["teamGroup"] = teamGroup;

    QActionGroup* goalGroup = new QActionGroup(this);
    goalGroup->addAction(_ui.actionDefendMinusX);
    goalGroup->addAction(_ui.actionDefendPlusX);
    qActionGroups["goalGroup"] = goalGroup;

    QActionGroup* rotateGroup = new QActionGroup(this);
    rotateGroup->addAction(_ui.action0);
    rotateGroup->addAction(_ui.action90);
    rotateGroup->addAction(_ui.action180);
    rotateGroup->addAction(_ui.action270);
    qActionGroups["rotateGroup"] = rotateGroup;

    auto visionChannelGroup = new QActionGroup(this);
    visionChannelGroup->addAction(_ui.actionVisionPrimary_Half);
    visionChannelGroup->addAction(_ui.actionVisionSecondary_Half);
    visionChannelGroup->addAction(_ui.actionVisionFull_Field);
    qActionGroups["visionChannelGroup"] = visionChannelGroup;

    auto radioGroup = new QActionGroup(this);
    radioGroup->addAction(_ui.action916MHz);
    radioGroup->addAction(_ui.action918MHz);
    qActionGroups["radioGroup"] = radioGroup;

    auto styleGroup = new QActionGroup(this);
    styleGroup->addAction(_ui.actionNoneStyle);
    styleGroup->addAction(_ui.actionDarkStyle);
    styleGroup->addAction(_ui.actionDarculizedStyle);
    styleGroup->addAction(_ui.action1337h4x0rStyle);
    qActionGroups["styleGroup"] = styleGroup;

    connect(_ui.manualID, SIGNAL(currentIndexChanged(int)), this,
            SLOT(on_manualID_currentIndexChanged(int)));

    // put all log playback buttons into a vector for easy access later
    _logPlaybackButtons.push_back(_ui.logPlaybackRewind);
    _logPlaybackButtons.push_back(_ui.logPlaybackPrevFrame);
    _logPlaybackButtons.push_back(_ui.logPlaybackPause);
    _logPlaybackButtons.push_back(_ui.logPlaybackNextFrame);
    _logPlaybackButtons.push_back(_ui.logPlaybackPlay);
    _logPlaybackButtons.push_back(_ui.logPlaybackLive);

    // Get the item model from the goalieID boxes so we can disable them
    // properly
    goalieModel = qobject_cast<const QStandardItemModel*>(_ui.goalieID->model());

    _node = std::make_shared<rclcpp::Node>("main_window");

    // Append short Git hash to the main window title with an asterisk if the
    // current Git index is dirty
    setWindowTitle(windowTitle() + " @ " + git_version_short_hash + (git_version_dirty ? "*" : ""));

    // Pass context into fieldview
    // (apparently simfieldview is used even outside of simulation)
    _ui.fieldView->setup(context_, _node.get());

    if (!_game_settings.simulation) {
        _ui.menu_Simulator->setEnabled(false);
    } else {
        // reset the field initially, grSim will start out in some weird
        // pattern and we want to keep it consistent
        // TODO(#1524) Fix the race condition here. If grsim isn't up when this
        // is run, nothing happens.
        on_actionResetField_triggered();
    }

    // disabled because of lack of grSim support
    _ui.actionQuickloadRobotLocations->setEnabled(false);
    _ui.actionQuicksaveRobotLocations->setEnabled(false);
    //_ui.actionResetField->setEnabled(false);
    _ui.actionStopRobots->setEnabled(false);

    _quick_commands_srv =
        _node->create_client<rj_msgs::srv::QuickCommands>(referee::topics::kQuickCommandsSrv);
    _set_game_settings = _node->create_client<rj_msgs::srv::SetGameSettings>(
        config_server::topics::kGameSettingsSrv);

    _executor.add_node(_node);
    _executor_thread = std::thread([this]() { _executor.spin(); });
}

void MainWindow::initialize() {
    // Team
    if (_game_settings.request_blue_team) {
        _ui.actionTeamBlue->trigger();
    } else {
        _ui.actionTeamYellow->trigger();
    }

    logFileChanged();

    // Initialize to ui defaults
    on_goalieID_currentIndexChanged(_ui.goalieID->currentIndex());

    qActionGroups["teamGroup"]->checkedAction()->trigger();
    qActionGroups["rotateGroup"]->checkedAction()->trigger();
    qActionGroups["radioGroup"]->checkedAction()->trigger();

    // Default to FullField on Simulator
    if (_game_settings.simulation) {
        _ui.actionVisionFull_Field->trigger();
    }

    updateTimer.setSingleShot(true);
    connect(&updateTimer, SIGNAL(timeout()), SLOT(updateViews()));
    updateTimer.start(30);

    if (_game_settings.defend_plus_x) {
        on_actionDefendPlusX_triggered();
        _ui.actionDefendPlusX->setChecked(true);
    } else {
        on_actionDefendMinusX_triggered();
        _ui.actionDefendMinusX->setChecked(true);
    }

    // If we're reading logs, we should already have some data. Update frames
    // for all of it.
    for (const auto& frame : context_->logs.frames) {
        updateDebugLayers(*frame);
    }

    if (context_->logs.state == Logs::State::kReading) {
        _playbackRate = 0;
    }
}

void MainWindow::logFileChanged() {
    if (context_->logs.state == Logs::State::kWriting) {
        QString filename_q = QString::fromStdString(context_->logs.filename.value());
        _logFile->setText(filename_q);
        _ui.actionStart_Logging->setText(QString("Already Logging to: ") + filename_q);
        _ui.actionStart_Logging->setEnabled(false);
    } else {
        _logFile->setText("Not Recording");
    }
}

void MainWindow::addLayer(int i, const QString& name, bool checked) {
    auto* item = new QListWidgetItem(name);
    Qt::CheckState checkState = checked ? Qt::Checked : Qt::Unchecked;
    item->setCheckState(checkState);
    item->setData(Qt::UserRole, i);
    _ui.debugLayers->addItem(item);
    on_debugLayers_itemChanged(item);
}

string MainWindow::formatLabelBold(Side side, const string& label) {
    string color;
    // Colors match up with those statically defined in MainWindow.ui
    if (side == Side::Yellow) {
        color = "#ac9f2d";
    } else if (side == Side::Blue) {
        color = "#000064";
    }
    return "<html><head/><body><p><span style=\"color:" + color + "; font-weight: bold;\">" +
           label + "</span></p></body></html>";
}

void MainWindow::updateFromRefPacket(bool haveExternalReferee) {
    // update goalie from Packet
    if (haveExternalReferee) {
        // The External Ref is connected
        _ui.goalieID->setEnabled(false);
        // disable Blue/Yellow team
        qActionGroups["teamGroup"]->setEnabled(false);
        // disable Blue/Yellow team
        qActionGroups["teamGroup"]->setEnabled(false);

        // Changes the goalie INDEX which is 1 higher than the goalie ID
        if (_ui.goalieID->currentIndex() != _game_settings.request_goalie_id + 1) {
            _ui.goalieID->setCurrentIndex(_game_settings.request_goalie_id + 1);
        }

        bool blueTeam = context_->blue_team;
        if (_game_settings.request_blue_team != blueTeam) {
            blueTeam ? _ui.actionTeamBlue->trigger() : _ui.actionTeamYellow->trigger();
        }
    } else {
        _ui.goalieID->setEnabled(true);
        qActionGroups["teamGroup"]->setEnabled(true);
    }
}

void MainWindow::updateViews() {
    // TODO(Kyle): Re-enable manual control
#if MANUAL
    int manual = context_->game_settings.joystick_config.manualID;
    if ((manual >= 0 || _ui.manualID->isEnabled()) && !context_->joystick_valid) {
        // Joystick is gone - turn off manual control
        _ui.manualID->setCurrentIndex(0);
        context_->game_settings.joystick_config.manualID = -1;
        _ui.manualID->setEnabled(false);
        _ui.tabWidget->setTabEnabled(_ui.tabWidget->indexOf(_ui.joystickTab), false);
    } else if (!_ui.manualID->isEnabled() && context_->joystick_valid) {
        // Joystick reconnected
        _ui.manualID->setEnabled(true);
        _ui.joystickTab->setVisible(true);
        _ui.tabWidget->setTabEnabled(_ui.tabWidget->indexOf(_ui.joystickTab), true);
    }
#endif

    PlayState play_state = PlayState::halt();
    MatchState match_state;
    TeamInfo our_info;
    TeamInfo their_info;
    bool blue_team = false;
    {
        std::lock_guard<std::mutex> lock(*context__mutex);
        play_state = context_->play_state;
        match_state = context_->match_state;
        blue_team = context_->blue_team;
        our_info = context_->our_info;
        their_info = context_->their_info;
    }

    // Time since last update
    RJ::Time now = RJ::now();
    auto delta_time = now - _lastUpdateTime;
    _lastUpdateTime = now;
    double framerate = RJ::Seconds(1) / delta_time;

    // Update status line displays
    ++_updateCount;
    if (_updateCount == 4) {
        _updateCount = 0;

        _viewFPS->setText(QString("View: %1 fps").arg(framerate, 0, 'f', 1));
        _procFPS->setText(QString("Proc: %1 fps").arg(_processor->framerate(), 0, 'f', 1));

        _logMemory->setText(
            QString("Log: %1 kiB").arg(QString::number((context_->logs.size_bytes + 512) / 1024)));
    }

    auto value = _ui.logHistoryLocation->value();

    std::shared_ptr<LogFrame> live_frame;
    RJ::Time start_time;
    int minFrame = 0;
    int maxFrame = 0;

    // Grab frames
    {
        std::lock_guard<std::mutex> lock(*context__mutex);
        if (context_->logs.frames.empty()) {
            // No log frames, nothing else to update.
            return;
        }

        start_time = context_->logs.start_time;

        size_t num_dropped = context_->logs.dropped_frames;

        if (live()) {
            _doubleFrameNumber = static_cast<double>(context_->logs.frames.size() + num_dropped);
        } else {
            _doubleFrameNumber += *_playbackRate;
        }

        minFrame = num_dropped;
        maxFrame = static_cast<int>(num_dropped + context_->logs.frames.size()) - 1;

        if (_doubleFrameNumber < minFrame) {
            _doubleFrameNumber = minFrame;
        } else if (_doubleFrameNumber >= maxFrame) {
            _doubleFrameNumber = maxFrame;
            setLive();
        }

        _ui.logHistoryLocation->setMinimum(minFrame);
        _ui.logHistoryLocation->setMaximum(maxFrame);

        live_frame = context_->logs.frames.back();

        // Cast to ints so that subtraction doesn't overflow.
        int start = std::max(frameNumber() - kLongHistorySize, minFrame);

        // Read the latest frames
        _longHistory.assign(context_->logs.frames.begin() + start - num_dropped,
                            context_->logs.frames.begin() + frameNumber() - num_dropped + 1);
    }

    // Set the history vector by taking the last kHistorySize elements of the
    // "long" history, or fewer if _longHistory is shorter.
    _history.assign(
        _longHistory.end() - std::min(kHistorySize, static_cast<int>(_longHistory.size())),
        _longHistory.end());

    // Update field view
    _ui.fieldView->update();

    /**************************************************************************/
    /***************** Update the history/playback interface ******************/
    /**************************************************************************/
    _ui.logHistoryLocation->setTickInterval(60 * 60);  // interval is ~ 1 minute
    _ui.logHistoryLocation->setValue(static_cast<int>(_doubleFrameNumber));

    // enable playback buttons based on playback rate
    for (QPushButton* playbackBtn : _logPlaybackButtons) {
        playbackBtn->setEnabled(true);
    }
    _ui.logPlaybackLive->setEnabled(!live());

    if (live() || abs<float>(*_playbackRate) > 0.01) {
        _ui.logPlaybackPause->setAutoFillBackground(false);
    } else {
        _ui.logPlaybackPause->setAutoFillBackground(true);
    }

    //  enable previous frame button based on position in the log
    _ui.logPlaybackPrevFrame->setEnabled(_doubleFrameNumber >= 1);
    _ui.logPlaybackNextFrame->setEnabled(!live());

    // Update status indicator
    updateStatus();

    update_cache(_game_settings.paused, !live(), &_game_settings_valid);

    // Check if any debug layers have been added
    // (layers should never be removed)
    if (live_frame) {
        updateDebugLayers(*live_frame);
    }

    // Get the frame at the log playback time
    const std::shared_ptr<LogFrame> currentFrame = _history.back();

    // Update the playback labels
    if (currentFrame) {
        auto gametime = RJ::Time(chrono::microseconds(currentFrame->timestamp())) - start_time;
        auto minutes = chrono::duration_cast<chrono::minutes>(gametime);
        gametime -= minutes;
        auto seconds = chrono::duration_cast<chrono::seconds>(gametime);
        gametime -= seconds;
        auto deciseconds = chrono::duration_cast<chrono::duration<long, ratio<1, 100>>>(gametime);

        _ui.logTime->setText(QString::fromStdString(to_string(minutes.count()) + ":" +
                                                    to_string(seconds.count()) + "." +
                                                    to_string(deciseconds.count())));

        _ui.frameNumLabel->setText(
            QString("%1/%2").arg(QString::number(frameNumber())).arg(QString::number(maxFrame)));
    }

    /**************************************************************************/
    /***************** Update log tree and behavior tree **********************/
    /**************************************************************************/
    if (currentFrame != nullptr) {
        _ui.logTree->message(*currentFrame);

        // update the behavior tree view
        QString behaviorStr = QString::fromStdString(currentFrame->behavior_tree());
        if (_ui.behaviorTree->toPlainText() != behaviorStr) {
            _ui.behaviorTree->setPlainText(behaviorStr);
        }
    }

    /**************************************************************************/
    /******************** Update referee information **************************/
    /**************************************************************************/
    // TODO(Kyle): Get period
    _ui.refStage->setText("");
    _ui.refCommand->setText(QString::fromStdString(play_state.get_human_readout()));

    // Convert time left from ms to s and display it to two decimal places
    int timeSeconds = static_cast<int>(match_state.stage_time_left.count());
    int timeMinutes = timeSeconds / 60;
    timeSeconds = timeSeconds % 60;
    _ui.refTimeLeft->setText(
        tr("%1:%2").arg(QString::number(timeMinutes), QString::number(std::abs(timeSeconds))));

    // Get team information for the yellow and blue teams, and display it.
    TeamInfo blue_info = blue_team ? our_info : their_info;

    const char* blueName = blue_info.name.c_str();
    string blueFormatted = strlen(blueName) == 0 ? "Blue Team" : blueName;
    blueFormatted = formatLabelBold(Side::Blue, blueFormatted);
    _ui.refBlueName->setText(QString::fromStdString(blueFormatted));
    _ui.refBlueScore->setText(tr("%1").arg(blue_info.score));
    _ui.refBlueRedCards->setText(tr("%1").arg(blue_info.red_cards));
    _ui.refBlueYellowCards->setText(tr("%1").arg(blue_info.yellow_cards));
    _ui.refBlueTimeoutsLeft->setText(tr("%1").arg(blue_info.timeouts_left));
    _ui.refBlueGoalie->setText(tr("%1").arg(blue_info.goalie));

    TeamInfo yellow_info = !blue_team ? our_info : their_info;

    const char* yellowName = yellow_info.name.c_str();
    string yellowFormatted = strlen(yellowName) == 0 ? "Yellow Team" : yellowName;
    yellowFormatted = formatLabelBold(Side::Yellow, yellowFormatted);
    _ui.refYellowName->setText(QString::fromStdString(yellowFormatted));
    _ui.refYellowScore->setText(tr("%1").arg(yellow_info.score));
    _ui.refYellowRedCards->setText(tr("%1").arg(yellow_info.red_cards));
    _ui.refYellowYellowCards->setText(tr("%1").arg(yellow_info.yellow_cards));
    _ui.refYellowTimeoutsLeft->setText(tr("%1").arg(yellow_info.timeouts_left));
    _ui.refYellowGoalie->setText(tr("%1").arg(yellow_info.goalie));

    /**************************************************************************/
    /********************** Update robot status list **************************/
    /**************************************************************************/
    if (currentFrame != nullptr) {
        // update robot status list
        for (size_t shell = 0; shell < kNumShells; shell++) {
            // Search for the corresponding references.
            auto maybe_rx = [&]() -> std::optional<std::reference_wrapper<const Packet::RadioRx>> {
                for (int i = 0; i < currentFrame->radio_rx_size(); i++) {
                    if (currentFrame->radio_rx(i).robot_id() == shell) {
                        return currentFrame->radio_rx(i);
                    }
                }
                return std::nullopt;
            }();
            auto maybe_robot =
                [&]() -> std::optional<std::reference_wrapper<const Packet::LogFrame_Robot>> {
                for (int i = 0; i < currentFrame->self_size(); i++) {
                    if (currentFrame->self(i).shell() == shell) {
                        return currentFrame->self(i);
                    }
                }
                return std::nullopt;
            }();

            auto statusItemIt = _robotStatusItemMap.find(shell);

            // see if it's already in the robot status list widget
            bool displaying = statusItemIt != _robotStatusItemMap.end();

            // If rx is missing, exit early.
            if (!maybe_rx.has_value()) {
                if (displaying) {
                    // We need to remove it from the list.
                    QListWidgetItem* item = statusItemIt->second.get();

                    // Delete widget from list
                    for (int row = 0; row < _ui.robotStatusList->count(); row++) {
                        if (_ui.robotStatusList->item(row) == item) {
                            _ui.robotStatusList->takeItem(row);
                            break;
                        }
                    }

                    _robotStatusItemMap.erase(shell);
                }
                continue;
            }

            // If we get here, we know there is an rx.
            const auto& rx = maybe_rx.value().get();

            // The status widget corresponding to this robot
            RobotStatusWidget* statusWidget = nullptr;

            if (!displaying) {
                // add a widget to the list for this robot
                auto item_owned = std::make_unique<QListWidgetItem>();
                auto* item = item_owned.get();
                _robotStatusItemMap[shell] = std::move(item_owned);
                _ui.robotStatusList->addItem(item);

                // The item's widget is managed by Qt
                // (setItemWidget takes ownership).
                statusWidget = new RobotStatusWidget();  // NOLINT
                item->setSizeHint(statusWidget->minimumSizeHint());
                _ui.robotStatusList->setItemWidget(item, statusWidget);
            } else {
                statusWidget = dynamic_cast<RobotStatusWidget*>(
                    _ui.robotStatusList->itemWidget(statusItemIt->second.get()));
            }

            if (statusWidget != nullptr) {
                statusWidget->loadFromLogFrame(rx, maybe_robot, currentFrame->blue_team());
            }
        }
    }

    if (!_game_settings_valid) {
        auto game_settings_request = std::make_shared<rj_msgs::srv::SetGameSettings::Request>();
        game_settings_request->game_settings = _game_settings;
        _game_settings_valid = true;

        // If the request fails, we want to resend it, so mark game settings as invalid.
        _set_game_settings->async_send_request(
            game_settings_request,
            [this](rclcpp::Client<rj_msgs::srv::SetGameSettings>::SharedFuture result) {  // NOLINT
                if (!result.get()) {
                    _game_settings_valid = false;
                }
            });
    }

    // We restart this timer repeatedly instead of using a single shot timer in
    // order to guarantee a minimum time between redraws.  This will limit the
    // CPU usage on a fast computer.
    updateTimer.start(20);
}

void MainWindow::updateStatus() {
    // Guidelines:
    //    Status_Fail is used for severe, usually external, errors such as
    //    hardware or network failures.
    //    Status_Warning is used for configuration problems that prevent
    //    competition operation.
    //        These can be easily changed within the soccer program.
    //    Status_OK shall only be used for "COMPETITION".
    //
    // The order of these checks is important to help debugging.
    // More specific or unlikely problems should be tested earlier.

    if (_processor == nullptr) {
        status("NO PROCESSOR", StatusType::Status_Fail);
        return;
    }

#if 0
    if (_processor->gameplay_module()->check_playbook_status()) {
        playIndicatorStatus(false);
    }
#endif

    // Some conditions are different in simulation
    bool sim = _game_settings.simulation;

    // TODO(Kyle): Figure out later if radio works.
    //    if (!sim) {
    //        updateRadioBaseStatus(_processor->is_radio_open());
    //    }

    // Get processing thread status
    Processor::Status ps = _processor->status();
    RJ::Time curTime = RJ::now();

    // Determine if we are receiving packets from an external referee
    // TODO(#1557): if we stop getting referee packets, set this to false.
    bool referee_updated = _has_external_ref;

    for (size_t i = 0; i < kNumShells; i++) {
        // The list starts with None so i is 1 higher than the shell id
        QStandardItem* item = goalieModel->item(i + 1);
        if (context_->world_state.our_robots.at(i).visible) {
            item->setFlags(item->flags() | (Qt::ItemIsSelectable | Qt::ItemIsEnabled));
        } else {
            item->setFlags(item->flags() & ~(Qt::ItemIsSelectable | Qt::ItemIsEnabled));
        }
    }

    if (referee_updated) {
        // External Ref is connected and should be used
        _ui.fastHalt->setEnabled(false);
        _ui.fastStop->setEnabled(false);
        _ui.fastReady->setEnabled(false);
        _ui.fastForceStart->setEnabled(false);
        _ui.fastKickoffBlue->setEnabled(false);
        _ui.fastKickoffYellow->setEnabled(false);
        _ui.fastBlue->setEnabled(false);
    } else {
        _ui.fastHalt->setEnabled(true);
        _ui.fastStop->setEnabled(true);
        _ui.fastReady->setEnabled(true);
        _ui.fastForceStart->setEnabled(true);
        _ui.fastKickoffBlue->setEnabled(true);
        _ui.fastKickoffYellow->setEnabled(true);
        _ui.fastBlue->setEnabled(true);
    }

    updateFromRefPacket(_has_external_ref);

    // Is the processing thread running?
    if (curTime - ps.last_loop_time > RJ::Seconds(0.1)) {
        // Processing loop hasn't run recently.
        // Likely causes:
        //    Mutex deadlock (need a recursive mutex?)
        //    Excessive computation
        status("PROCESSING HUNG", StatusType::Status_Fail);
        return;
    }

    // Check network activity
    if (curTime - ps.last_vision_time > RJ::Seconds(0.1)) {
        // We must always have vision
        status("NO VISION", StatusType::Status_Fail);
        return;
    }

#if MANUAL
    if (context_->game_settings.joystick_config.manualID >= 0) {
        // Mixed auto/manual control
        status("MANUAL", StatusType::Status_Warning);
        return;
    }
#endif

    // Driving the robots helps isolate radio problems by verifying radio TX,
    // so test this after manual driving.
    if (curTime - ps.last_radio_rx_time > RJ::Seconds(1)) {
        // Allow a long timeout in case of poor radio performance
        status("NO RADIO RX", StatusType::Status_Fail);
        return;
    }

    if (_has_external_ref && !referee_updated) {
        // In simulation, we will often run without a referee, so just make
        // it a warning.
        // There is a separate status for non-simulation with internal
        // referee.
        status("NO REFEREE", StatusType::Status_Fail);
        return;
    }

    if (sim) {
        // Everything is good for simulation, but not for competition.
        status("SIMULATION", StatusType::Status_Warning);
        return;
    }

    if (!sim && context_->logs.state != Logs::State::kWriting) {
        // We should record logs during competition
        status("NOT RECORDING", StatusType::Status_Warning);
        return;
    }

    status("COMPETITION", StatusType::Status_OK);
}

void MainWindow::status(const QString& text, MainWindow::StatusType status) {
    // Assume that the status type alone won't change.
    if (_ui.statusLabel->text() != text) {
        _ui.statusLabel->setText(text);

        switch (status) {
            case StatusType::Status_OK:
                _ui.statusLabel->setStyleSheet("background-color: #00ff00");
                break;

            case StatusType::Status_Warning:
                _ui.statusLabel->setStyleSheet("background-color: #ffff00");
                break;

            case StatusType::Status_Fail:
                _ui.statusLabel->setStyleSheet("background-color: #ff4040");
                break;
        }
    }
}

void MainWindow::updateRadioBaseStatus(bool usbRadio) {
    QString label = QString(usbRadio ? "Radio Connected" : "Radio Disconnected");
    if (_ui.radioBaseStatus->text() != label) {
        _ui.radioBaseStatus->setText(label);
        _ui.radioBaseStatus->setStyleSheet(usbRadio ? QString("background-color: #00ff00")
                                                    : QString("background-color: #ff4040"));
    }
}

void MainWindow::on_fieldView_robotSelected(int shell) {
    if (context_->joystick_valid) {
        _ui.manualID->setCurrentIndex(shell + 1);

#if MANUAL
        std::lock_guard<std::mutex> lock(*context__mutex);
        _game_settings.joystick_config.manualID = shell;
#endif
    }
}

void MainWindow::on_actionRawBalls_toggled(bool state) {
    _ui.fieldView->showRawBalls = state;
    _ui.fieldView->update();
}

void MainWindow::on_actionRawRobots_toggled(bool state) {
    _ui.fieldView->showRawRobots = state;
    _ui.fieldView->update();
}

void MainWindow::on_actionCoords_toggled(bool state) {
    _ui.fieldView->showCoords = state;
    _ui.fieldView->update();
}

void MainWindow::on_actionDotPatterns_toggled(bool state) {
    _ui.fieldView->showDotPatterns = state;
    _ui.fieldView->update();
}

void MainWindow::on_actionTeam_Names_toggled(bool state) {
    _ui.fieldView->showTeamNames = state;
    _ui.fieldView->update();
}

void MainWindow::on_actionDefendMinusX_triggered() {
    update_cache(_game_settings.defend_plus_x, false, &_game_settings_valid);
}

void MainWindow::on_actionDefendPlusX_triggered() {
    update_cache(_game_settings.defend_plus_x, true, &_game_settings_valid);
}

void MainWindow::on_action0_triggered() { _ui.fieldView->rotate(0); }

void MainWindow::on_action90_triggered() { _ui.fieldView->rotate(1); }

void MainWindow::on_action180_triggered() { _ui.fieldView->rotate(2); }

void MainWindow::on_action270_triggered() { _ui.fieldView->rotate(3); }

void MainWindow::on_actionUseOurHalf_toggled(bool value) {
    update_cache(_game_settings.use_our_half, value, &_game_settings_valid);
}

void MainWindow::on_actionUseOpponentHalf_toggled(bool value) {
    update_cache(_game_settings.use_their_half, value, &_game_settings_valid);
}

void MainWindow::on_actionCenterBall_triggered() {
    _ui.fieldView->set_ball_position(rj_geometry::Point(0, 0));
}

void MainWindow::on_actionStopBall_triggered() {
    _ui.fieldView->set_ball_velocity(rj_geometry::Point(0, 0));
}

void MainWindow::on_actionResetField_triggered() {
    _ui.fieldView->set_robot_pose(rj_geometry::Pose(2, -1, 0), 0, false);
    _ui.fieldView->set_robot_pose(rj_geometry::Pose(2, -1, 0), 0, true);

    _ui.fieldView->set_robot_pose(rj_geometry::Pose(2, 0, 0), 1, false);
    _ui.fieldView->set_robot_pose(rj_geometry::Pose(2, 0, 0), 1, true);

    _ui.fieldView->set_robot_pose(rj_geometry::Pose(2, 1, 0), 2, false);
    _ui.fieldView->set_robot_pose(rj_geometry::Pose(2, 1, 0), 2, true);

    _ui.fieldView->set_robot_pose(rj_geometry::Pose(3, -1, 0), 3, false);
    _ui.fieldView->set_robot_pose(rj_geometry::Pose(3, -1, 0), 3, true);

    _ui.fieldView->set_robot_pose(rj_geometry::Pose(3, 0, 0), 4, false);
    _ui.fieldView->set_robot_pose(rj_geometry::Pose(3, 0, 0), 4, true);

    _ui.fieldView->set_robot_pose(rj_geometry::Pose(3, 1, 0), 5, false);
    _ui.fieldView->set_robot_pose(rj_geometry::Pose(3, 1, 0), 5, true);

    _ui.fieldView->set_ball_position(rj_geometry::Point(0.0, 0.0));
    _ui.fieldView->set_ball_velocity(rj_geometry::Point(0.0, 0.0));
}

void MainWindow::on_actionStopRobots_triggered() {}

void MainWindow::on_actionQuicksaveRobotLocations_triggered() {}

void MainWindow::on_actionQuickloadRobotLocations_triggered() {}

// Style Sheets

void MainWindow::on_actionNoneStyle_triggered() {
    StyleSheetManager::changeStyleSheet(this, "NONE");
}

void MainWindow::on_actionDarkStyle_triggered() {
    StyleSheetManager::changeStyleSheet(this, "DARK");
}

void MainWindow::on_actionDarculizedStyle_triggered() {
    StyleSheetManager::changeStyleSheet(this, "DARCULIZED");
}
void MainWindow::on_action1337h4x0rStyle_triggered() {
    StyleSheetManager::changeStyleSheet(this, "1337H4X0R");
}
void MainWindow::on_actionNyanStyle_triggered() {
    StyleSheetManager::changeStyleSheet(this, "NYAN");
}

// Manual control commands

void MainWindow::on_actionDampedRotation_toggled(bool value) {
#if MANUAL
    cout << "DampedRotation is ";
    if (value)
        cout << "Enabled" << endl;
    else
        cout << "Disabled" << endl;

    std::lock_guard<std::mutex> lock(*context__mutex);
    context_->game_settings.joystick_config.dampedRotation = value;
#endif
}

void MainWindow::on_actionDampedTranslation_toggled(bool value) {
#if MANUAL
    cout << "DampedTranslation is ";
    if (value)
        cout << "Enabled" << endl;
    else
        cout << "Disabled" << endl;

    std::lock_guard<std::mutex> lock(*context__mutex);
    context_->game_settings.joystick_config.dampedTranslation = value;
#endif
}

void MainWindow::on_actionRestartUpdateTimer_triggered() {
    printf("Update timer: active %d, singleShot %d, interval %d\n",
           static_cast<int>(updateTimer.isActive()), static_cast<int>(updateTimer.isSingleShot()),
           updateTimer.interval());
    updateTimer.stop();
    updateTimer.start(30);
}

void MainWindow::on_actionStart_Logging_triggered() {
    if (context_->logs.state != Logs::State::kWriting) {
        if (!QDir("logs").exists()) {
            QDir().mkdir("logs");
        }

        QString logFile =
            QString("logs/") + QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss.log");

        if (!_processor->open_log(logFile)) {
            printf("Failed to open %s: %m\n", (const char*)logFile.toLatin1());
        } else {
            _ui.actionStart_Logging->setText(QString("Now Logging to:") + logFile);
            _ui.actionStart_Logging->setEnabled(false);
        }
    }
}

// Gameplay commands

void MainWindow::on_actionSeed_triggered() {
    QString text = QInputDialog::getText(this, "Set Random Seed", "Hexadecimal seed:");
    if (!text.isNull()) {
        long seed = strtol(text.toLatin1(), nullptr, 16);
        printf("seed %016lx\n", seed);
        srand48(seed);
    }
}

// Joystick settings
void MainWindow::on_joystickKickOnBreakBeam_stateChanged() {
#if MANUAL
    std::lock_guard<std::mutex> lock(*context__mutex);
    context_->game_settings.joystick_config.useKickOnBreakBeam =
        _ui.joystickKickOnBreakBeam->checkState() == Qt::CheckState::Checked;
#endif
}

// choose between kick on break beam and immeditate

// Log controls
void MainWindow::on_logHistoryLocation_sliderMoved(int value) {
    // Sync frameNumber with logHistory slider
    _doubleFrameNumber = value;

    // pause playback
    setPlayBackRate(0);
}

void MainWindow::on_logHistoryLocation_sliderPressed() {
    on_logHistoryLocation_sliderMoved(_ui.logHistoryLocation->value());
}

void MainWindow::on_logHistoryLocation_sliderReleased() { on_logHistoryLocation_sliderPressed(); }

void MainWindow::on_logPlaybackRewind_clicked() {
    if (live()) {
        setPlayBackRate(-1);
    } else {
        *_playbackRate += -0.5;
    }
}

void MainWindow::on_logPlaybackPrevFrame_clicked() {
    setPlayBackRate(0);
    _doubleFrameNumber -= 1;
}

void MainWindow::on_logPlaybackPause_clicked() {
    if (live() || std::abs(*_playbackRate) > 0.1) {
        setPlayBackRate(0);
    } else {
        setPlayBackRate(1);
    }
}

void MainWindow::on_logPlaybackNextFrame_clicked() {
    setPlayBackRate(0);
    _doubleFrameNumber += 1;
}

void MainWindow::on_logPlaybackPlay_clicked() {
    if (!live()) {
        *_playbackRate += 0.5;
    }
}

void MainWindow::on_logPlaybackLive_clicked() { setLive(); }

void MainWindow::on_actionTeamBlue_triggered() {
    _ui.team->setText("BLUE");
    _ui.team->setStyleSheet("background-color: #4040ff; color: #ffffff");

    update_cache(_game_settings.request_blue_team, true, &_game_settings_valid);
}

void MainWindow::on_actionTeamYellow_triggered() {
    _ui.team->setText("YELLOW");
    _ui.team->setStyleSheet("background-color: #ffff00");

    update_cache(_game_settings.request_blue_team, false, &_game_settings_valid);
}

void MainWindow::on_manualID_currentIndexChanged(int value) {
#if MANUAL
    context_->game_settings.joystick_config.manualID = value - 1;
#endif
}

void MainWindow::on_actionUse_Field_Oriented_Controls_toggled(bool value) {
#if MANUAL
    context_->game_settings.joystick_config.useFieldOrientedDrive = value;
#endif
}

void MainWindow::on_actionUse_Multiple_Joysticks_toggled(bool value) {
    // TODO(Kyle): Reimplement multiple manual
}

void MainWindow::on_goalieID_currentIndexChanged(int value) {
    update_cache(_game_settings.request_goalie_id, value - 1, &_game_settings_valid);
}

////////////////
// Tab Widget Section

////////
// Debug layer list

void MainWindow::allDebugOn() {
    for (int i = 0; i < _ui.debugLayers->count(); ++i) {
        _ui.debugLayers->item(i)->setCheckState(Qt::Checked);
    }
}

void MainWindow::allDebugOff() {
    for (int i = 0; i < _ui.debugLayers->count(); ++i) {
        _ui.debugLayers->item(i)->setCheckState(Qt::Unchecked);
    }
}

void MainWindow::on_debugLayers_customContextMenuRequested(const QPoint& pos) {
    QListWidgetItem* item = _ui.debugLayers->itemAt(pos);

    QMenu menu;
    QAction* all = menu.addAction("All");
    QAction* none = menu.addAction("None");
    QAction* single = nullptr;
    QAction* notSingle = nullptr;
    if (item != nullptr) {
        single = menu.addAction("Only this");
        notSingle = menu.addAction("All except this");
    }

    QAction* act = menu.exec(_ui.debugLayers->mapToGlobal(pos));
    if (act == all) {
        allDebugOn();
    } else if (act == none) {
        allDebugOff();
    } else if ((single != nullptr) && act == single) {
        allDebugOff();
        item->setCheckState(Qt::Checked);
    } else if ((notSingle != nullptr) && act == notSingle) {
        allDebugOn();
        item->setCheckState(Qt::Unchecked);
    }
}

void MainWindow::on_debugLayers_itemChanged(QListWidgetItem* item) {
    int layer = item->data(Qt::UserRole).toInt();
    if (layer >= 0) {
        _ui.fieldView->layerVisible(layer, item->checkState() == Qt::Checked);
    }
    _ui.fieldView->update();
}

// NOLINTNEXTLINE(readability-make-member-function-const): this modifies state
void MainWindow::setUseRefChecked(bool /* use_ref */) {
    _ui.actionUse_Field_Oriented_Controls->setChecked(false);
}

void MainWindow::send_quick_command(const PlayState& state) {
    auto request = std::make_shared<rj_msgs::srv::QuickCommands::Request>();
    request->command = rj_convert::convert_to_ros(state);
    _quick_commands_srv->async_send_request(request);
    queued_command_ = std::nullopt;
}

void MainWindow::on_fastHalt_clicked() { send_quick_command(PlayState::halt()); }

void MainWindow::on_fastStop_clicked() { send_quick_command(PlayState::stop()); }

void MainWindow::on_fastReady_clicked() {
    if (queued_command_) {
        send_quick_command(*queued_command_);
    }
}

void MainWindow::on_fastForceStart_clicked() { send_quick_command(PlayState::playing()); }

void MainWindow::on_fastKickoffBlue_clicked() {
    send_quick_command(PlayState::setup_kickoff(context_->blue_team));
    queued_command_ = PlayState::ready_kickoff(context_->blue_team);
}

void MainWindow::on_fastKickoffYellow_clicked() {
    send_quick_command(PlayState::setup_kickoff(!context_->blue_team));
    queued_command_ = PlayState::ready_kickoff(context_->blue_team);
}

void MainWindow::on_fastBlue_clicked() {
    send_quick_command(PlayState::ready_free_kick(context_->blue_team));
}

void MainWindow::on_fastYellow_clicked() {
    send_quick_command(PlayState::ready_free_kick(!context_->blue_team));
}

bool MainWindow::live() { return !_playbackRate; }
void MainWindow::updateDebugLayers(const LogFrame& frame) {
    // Check if any debug layers have been added
    // (layers should never be removed)
    if (frame.debug_layers_size() > _ui.debugLayers->count()) {
        // Add the missing layers and turn them on
        for (int i = _ui.debugLayers->count(); i < frame.debug_layers_size(); ++i) {
            const QString name = QString::fromStdString(frame.debug_layers(i));
            bool enabled = !std::any_of(defaultHiddenLayers.begin(), defaultHiddenLayers.end(),
                                        [&](const QString& string) { return string == name; });
            addLayer(i, name, enabled);
        }

        _ui.debugLayers->sortItems();
    }
}
