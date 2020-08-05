#include "MainWindow.hpp"

#include <ctime>

#include <QActionGroup>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include <QString>
#include <google/protobuf/descriptor.h>

#include <gameplay/GameplayModule.hpp>
#include <rj_common/qt_utils.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_protos/grSim_Packet.pb.h>
#include <rj_protos/grSim_Replacement.pb.h>
#include <ui/StyleSheetManager.hpp>
#include <ui_MainWindow.h>

#include "BatteryProfile.hpp"
#include "Configuration.hpp"
#include "RobotStatusWidget.hpp"
#include "radio/Radio.hpp"

#include "rc-fshare/git_version.hpp"

using namespace std;
using namespace boost;
using namespace google::protobuf;
using namespace Packet;
using namespace Eigen;

constexpr int kHistorySize = 60 * 2;
constexpr int kLongHistorySize = 60 * 60 * 30;

static const std::vector<QString> kDefaultHiddenLayers{
    "MotionControl", "Global Obstacles", "Local Obstacles",
    "Planning0",     "Planning1",        "Planning2",
    "Planning3",     "Planning4",        "Planning5"};

void calcMinimumWidth(QWidget* widget, const QString& text) {
    QRect rect = QFontMetrics(widget->font()).boundingRect(text);
    widget->setMinimumWidth(rect.width());
}

MainWindow::MainWindow(Processor* processor, bool has_external_ref,
                       QWidget* parent)
    : QMainWindow(parent),
      _updateCount(0),
      _doubleFrameNumber(-1),
      _lastUpdateTime(RJ::now()),
      _processor(processor),
      _context(processor->context()),
      _has_external_ref(has_external_ref),
      _game_settings(rj_convert::convert_to_ros(_context->game_settings)) {
    _context_mutex = processor->loopMutex();

    qRegisterMetaType<QVector<int>>("QVector<int>");
    _ui.setupUi(this);
    _ui.fieldView->history(&_history);

    _ui.logTree->history(&_longHistory);
    _ui.logTree->mainWindow = this;
    _ui.logTree->updateTimer = &updateTimer;

    // Initialize live/non-live control styles

    _currentPlay = new QLabel(this);
    _currentPlay->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    _currentPlay->setToolTip("Current Play");
    _currentPlay->setAlignment(Qt::AlignCenter);
    _currentPlay->setObjectName("current_play_name");
    calcMinimumWidth(_currentPlay, "XXXXXXXXXXXXXXXX");
    statusBar()->addPermanentWidget(_currentPlay);

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

    QActionGroup* team_group = new QActionGroup(this);
    team_group->addAction(_ui.actionTeamBlue);
    team_group->addAction(_ui.actionTeamYellow);
    qActionGroups["teamGroup"] = team_group;

    QActionGroup* goal_group = new QActionGroup(this);
    goal_group->addAction(_ui.actionDefendMinusX);
    goal_group->addAction(_ui.actionDefendPlusX);
    qActionGroups["goalGroup"] = goal_group;

    QActionGroup* rotate_group = new QActionGroup(this);
    rotate_group->addAction(_ui.action0);
    rotate_group->addAction(_ui.action90);
    rotate_group->addAction(_ui.action180);
    rotate_group->addAction(_ui.action270);
    qActionGroups["rotateGroup"] = rotate_group;

    auto vision_channel_group = new QActionGroup(this);
    vision_channel_group->addAction(_ui.actionVisionPrimary_Half);
    vision_channel_group->addAction(_ui.actionVisionSecondary_Half);
    vision_channel_group->addAction(_ui.actionVisionFull_Field);
    qActionGroups["visionChannelGroup"] = vision_channel_group;

    auto radio_group = new QActionGroup(this);
    radio_group->addAction(_ui.action916MHz);
    radio_group->addAction(_ui.action918MHz);
    qActionGroups["radioGroup"] = radio_group;

    auto style_group = new QActionGroup(this);
    style_group->addAction(_ui.actionNoneStyle);
    style_group->addAction(_ui.actionDarkStyle);
    style_group->addAction(_ui.actionDarculizedStyle);
    style_group->addAction(_ui.action1337h4x0rStyle);
    qActionGroups["styleGroup"] = style_group;

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
    goalieModel =
        qobject_cast<const QStandardItemModel*>(_ui.goalieID->model());

    // Append short Git hash to the main window title with an asterisk if the
    // current Git index is dirty
    setWindowTitle(windowTitle() + " @ " + git_version_short_hash +
                   (git_version_dirty ? "*" : ""));

    // Pass context into fieldview
    // (apparently simfieldview is used even outside of simulation)
    _ui.fieldView->setContext(_context);

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

    _node = std::make_shared<rclcpp::Node>("main_window");
    _quick_commands_srv = _node->create_client<rj_msgs::srv::QuickCommands>(
        referee::topics::kQuickCommandsSrv);
    _quick_restart_srv = _node->create_client<rj_msgs::srv::QuickRestart>(
        referee::topics::kQuickRestartSrv);
    _set_game_settings = _node->create_client<rj_msgs::srv::SetGameSettings>(
        config_server::topics::kGameSettingsSrv);
    _executor.add_node(_node);
    _executor_thread = std::thread([this]() { _executor.spin(); });
}

void MainWindow::configuration(Configuration* config) {
    _config = config;
    _config->tree(_ui.configTree);
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
    for (const auto& frame : _context->logs.frames) {
        updateDebugLayers(*frame);
    }

    if (_context->logs.state == Logs::State::kReading) {
        _playbackRate = 0;
    }
}

void MainWindow::logFileChanged() {
    if (_context->logs.state == Logs::State::kWriting) {
        QString filename_q =
            QString::fromStdString(_context->logs.filename.value());
        _logFile->setText(filename_q);
        _ui.actionStart_Logging->setText(QString("Already Logging to: ") +
                                         filename_q);
        _ui.actionStart_Logging->setEnabled(false);
    } else {
        _logFile->setText("Not Recording");
    }
}

void MainWindow::addLayer(int i, const QString& name, bool checked) {
    auto* item = new QListWidgetItem(name);
    Qt::CheckState check_state = checked ? Qt::Checked : Qt::Unchecked;
    item->setCheckState(check_state);
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
    return "<html><head/><body><p><span style=\"color:" + color +
           "; font-weight: bold;\">" + label + "</span></p></body></html>";
}

void MainWindow::updateFromRefPacket(bool have_external_referee) {
    // update goalie from Packet
    if (have_external_referee) {
        // The External Ref is connected
        _ui.goalieID->setEnabled(false);
        // disable Blue/Yellow team
        qActionGroups["teamGroup"]->setEnabled(false);
        // disable Blue/Yellow team
        qActionGroups["teamGroup"]->setEnabled(false);

        // Changes the goalie INDEX which is 1 higher than the goalie ID
        if (_ui.goalieID->currentIndex() !=
            _game_settings.request_goalie_id + 1) {
            _ui.goalieID->setCurrentIndex(_game_settings.request_goalie_id + 1);
        }

        bool blue_team = _context->blue_team;
        if (_game_settings.request_blue_team != blue_team) {
            blue_team ? _ui.actionTeamBlue->trigger()
                      : _ui.actionTeamYellow->trigger();
        }
    } else {
        _ui.goalieID->setEnabled(true);
        qActionGroups["teamGroup"]->setEnabled(true);
    }
}

void MainWindow::updateViews() {
    // TODO(Kyle): Re-enable manual control
#if MANUAL
    int manual = _context->game_settings.joystick_config.manualID;
    if ((manual >= 0 || _ui.manualID->isEnabled()) &&
        !_context->joystick_valid) {
        // Joystick is gone - turn off manual control
        _ui.manualID->setCurrentIndex(0);
        _context->game_settings.joystick_config.manualID = -1;
        _ui.manualID->setEnabled(false);
        _ui.tabWidget->setTabEnabled(_ui.tabWidget->indexOf(_ui.joystickTab),
                                     false);
    } else if (!_ui.manualID->isEnabled() && _context->joystick_valid) {
        // Joystick reconnected
        _ui.manualID->setEnabled(true);
        _ui.joystickTab->setVisible(true);
        _ui.tabWidget->setTabEnabled(_ui.tabWidget->indexOf(_ui.joystickTab),
                                     true);
    }
#endif

    GameState game_state;
    TeamInfo our_info;
    TeamInfo their_info;
    bool blue_team = false;
    {
        std::lock_guard<std::mutex> lock(*_context_mutex);
        game_state = _context->game_state;
        blue_team = _context->blue_team;
        our_info = _context->our_info;
        their_info = _context->their_info;
    }

    // TODO(Kyle): Fix multiple manual
#if 0
    if (_processor->multipleManual() && manual < 0) {
        _ui.tabWidget->setTabEnabled(_ui.tabWidget->indexOf(_ui.joystickTab),
                                     false);
    } else {
        _ui.tabWidget->setTabEnabled(_ui.tabWidget->indexOf(_ui.joystickTab),
                                     true);
    }

    if (manual >= 0) {
        int index = 0;
        std::vector<int> manualIds = _processor->getJoystickRobotIds();
        auto info = std::find(manualIds.begin(), manualIds.end(), manual);
        if (info != manualIds.end()) {
            index = info - manualIds.begin();
        }

        auto valList = _processor->getJoystickControlValues();
        if (valList.size() > index) {
            JoystickControlValues vals = valList[index];
            _ui.joystickBodyXLabel->setText(tr("%1").arg(vals.translation.x()));
            _ui.joystickBodyYLabel->setText(tr("%1").arg(vals.translation.y()));
            _ui.joystickBodyWLabel->setText(tr("%1").arg(vals.rotation));
            _ui.joystickKickPowerLabel->setText(tr("%1").arg(vals.kickPower));
            _ui.joystickDibblerPowerLabel->setText(
                tr("%1").arg(vals.dribblerPower));
            _ui.joystickKickCheckBox->setChecked(vals.kick);
            _ui.joystickChipCheckBox->setChecked(vals.chip);
            _ui.joystickDribblerCheckBox->setChecked(vals.dribble);
        }
    }
#endif

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
        _procFPS->setText(
            QString("Proc: %1 fps").arg(_processor->framerate(), 0, 'f', 1));

        _logMemory->setText(QString("Log: %1 kiB")
                                .arg(QString::number(
                                    (_context->logs.size_bytes + 512) / 1024)));
    }

    auto value = _ui.logHistoryLocation->value();

    std::shared_ptr<LogFrame> live_frame;
    RJ::Time start_time;
    int min_frame = 0;
    int max_frame = 0;

    // Grab frames
    {
        std::lock_guard<std::mutex> lock(*_context_mutex);
        if (_context->logs.frames.empty()) {
            // No log frames, nothing else to update.
            return;
        }

        start_time = _context->logs.start_time;

        size_t num_dropped = _context->logs.dropped_frames;

        if (live()) {
            _doubleFrameNumber =
                static_cast<double>(_context->logs.frames.size() + num_dropped);
        } else {
            _doubleFrameNumber += *_playbackRate;
        }

        min_frame = num_dropped;
        max_frame =
            static_cast<int>(num_dropped + _context->logs.frames.size()) - 1;

        if (_doubleFrameNumber < min_frame) {
            _doubleFrameNumber = min_frame;
        } else if (_doubleFrameNumber >= max_frame) {
            _doubleFrameNumber = max_frame;
            setLive();
        }

        _ui.logHistoryLocation->setMinimum(min_frame);
        _ui.logHistoryLocation->setMaximum(max_frame);

        live_frame = _context->logs.frames.back();

        // Cast to ints so that subtraction doesn't overflow.
        int start = std::max(frameNumber() - kLongHistorySize, min_frame);

        // Read the latest frames
        _longHistory.assign(
            _context->logs.frames.begin() + start - num_dropped,
            _context->logs.frames.begin() + frameNumber() - num_dropped + 1);
    }

    // Set the history vector by taking the last kHistorySize elements of the
    // "long" history, or fewer if _longHistory is shorter.
    _history.assign(
        _longHistory.end() -
            std::min(kHistorySize, static_cast<int>(_longHistory.size())),
        _longHistory.end());

    // Update field view
    _ui.fieldView->update();

    /**************************************************************************/
    /***************** Update the history/playback interface ******************/
    /**************************************************************************/
    _ui.logHistoryLocation->setTickInterval(60 * 60);  // interval is ~ 1 minute
    _ui.logHistoryLocation->setValue(static_cast<int>(_doubleFrameNumber));

    // enable playback buttons based on playback rate
    for (QPushButton* playback_btn : _logPlaybackButtons) {
        playback_btn->setEnabled(true);
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
    const std::shared_ptr<LogFrame> current_frame = _history.back();

    // Update the playback labels
    if (current_frame) {
        auto gametime =
            RJ::Time(chrono::microseconds(current_frame->timestamp())) -
            start_time;
        auto minutes = chrono::duration_cast<chrono::minutes>(gametime);
        gametime -= minutes;
        auto seconds = chrono::duration_cast<chrono::seconds>(gametime);
        gametime -= seconds;
        auto deciseconds =
            chrono::duration_cast<chrono::duration<long, ratio<1, 100>>>(
                gametime);

        _ui.logTime->setText(QString::fromStdString(
            to_string(minutes.count()) + ":" + to_string(seconds.count()) +
            "." + to_string(deciseconds.count())));

        _ui.frameNumLabel->setText(QString("%1/%2")
                                       .arg(QString::number(frameNumber()))
                                       .arg(QString::number(max_frame)));
    }

    /**************************************************************************/
    /***************** Update log tree and behavior tree **********************/
    /**************************************************************************/
    if (current_frame != nullptr) {
        _ui.logTree->message(*current_frame);

        // update the behavior tree view
        QString behavior_str =
            QString::fromStdString(current_frame->behavior_tree());
        if (_ui.behaviorTree->toPlainText() != behavior_str) {
            _ui.behaviorTree->setPlainText(behavior_str);
        }
    }

    /**************************************************************************/
    /******************** Update referee information **************************/
    /**************************************************************************/
    // TODO(Kyle): Get these values from the protobuf.
    _ui.refStage->setText("");
    _ui.refCommand->setText("");
    //    _ui.refStage->setText(
    //        RefereeModuleEnums::stringFromStage(game_state.raw_stage).c_str());
    //    _ui.refCommand->setText(
    //        RefereeModuleEnums::stringFromCommand(game_state.raw_command).c_str());

    // Convert time left from ms to s and display it to two decimal places
    int time_seconds = static_cast<int>(game_state.stage_time_left.count());
    int time_minutes = time_seconds / 60;
    time_seconds = time_seconds % 60;
    _ui.refTimeLeft->setText(
        tr("%1:%2").arg(QString::number(time_minutes),
                        QString::number(std::abs(time_seconds))));

    // Get team information for the yellow and blue teams, and display it.
    TeamInfo blue_info = blue_team ? our_info : their_info;

    const char* blue_name = blue_info.name.c_str();
    string blue_formatted = strlen(blue_name) == 0 ? "Blue Team" : blue_name;
    blue_formatted = formatLabelBold(Side::Blue, blue_formatted);
    _ui.refBlueName->setText(QString::fromStdString(blue_formatted));
    _ui.refBlueScore->setText(tr("%1").arg(blue_info.score));
    _ui.refBlueRedCards->setText(tr("%1").arg(blue_info.red_cards));
    _ui.refBlueYellowCards->setText(tr("%1").arg(blue_info.yellow_cards));
    _ui.refBlueTimeoutsLeft->setText(tr("%1").arg(blue_info.timeouts_left));
    _ui.refBlueGoalie->setText(tr("%1").arg(blue_info.goalie));

    TeamInfo yellow_info = !blue_team ? our_info : their_info;

    const char* yellow_name = yellow_info.name.c_str();
    string yellow_formatted =
        strlen(yellow_name) == 0 ? "Yellow Team" : yellow_name;
    yellow_formatted = formatLabelBold(Side::Yellow, yellow_formatted);
    _ui.refYellowName->setText(QString::fromStdString(yellow_formatted));
    _ui.refYellowScore->setText(tr("%1").arg(yellow_info.score));
    _ui.refYellowRedCards->setText(tr("%1").arg(yellow_info.red_cards));
    _ui.refYellowYellowCards->setText(tr("%1").arg(yellow_info.yellow_cards));
    _ui.refYellowTimeoutsLeft->setText(tr("%1").arg(yellow_info.timeouts_left));
    _ui.refYellowGoalie->setText(tr("%1").arg(yellow_info.goalie));

    /**************************************************************************/
    /********************** Update robot status list **************************/
    /**************************************************************************/
    if (current_frame != nullptr) {
        // update robot status list
        for (int shell = 0; shell < Num_Shells; shell++) {
            // Search for the corresponding references.
            auto maybe_rx = [current_frame]()
                -> std::optional<
                    std::reference_wrapper<const Packet::RadioRx>> {
                for (int i = 0; i < current_frame->radio_rx_size(); i++) {
                    if (current_frame->radio_rx(i).robot_id() == shell) {
                        return current_frame->radio_rx(i);
                    }
                }
                return std::nullopt;
            }();
            auto maybe_robot = [current_frame]()
                -> std::optional<
                    std::reference_wrapper<const Packet::LogFrame_Robot>> {
                for (int i = 0; i < current_frame->self_size(); i++) {
                    if (current_frame->self(i).shell() == shell) {
                        return current_frame->self(i);
                    }
                }
                return std::nullopt;
            }();

            auto status_item_it = _robotStatusItemMap.find(shell);

            // see if it's already in the robot status list widget
            bool displaying = status_item_it != _robotStatusItemMap.end();

            // If rx is missing, exit early.
            if (!maybe_rx.has_value()) {
                if (displaying) {
                    // We need to remove it from the list.
                    QListWidgetItem* item = status_item_it->second.get();

                    // Delete widget from list
                    for (int row = 0; row < _ui.robotStatusList->count();
                         row++) {
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
            RobotStatusWidget* status_widget = nullptr;

            if (!displaying) {
                // add a widget to the list for this robot
                auto item_owned = std::make_unique<QListWidgetItem>();
                auto* item = item_owned.get();
                _robotStatusItemMap[shell] = std::move(item_owned);
                _ui.robotStatusList->addItem(item);

                // The item's widget is managed by Qt
                // (setItemWidget takes ownership).
                status_widget = new RobotStatusWidget();  // NOLINT
                item->setSizeHint(status_widget->minimumSizeHint());
                _ui.robotStatusList->setItemWidget(item, status_widget);
            } else {
                status_widget = dynamic_cast<RobotStatusWidget*>(
                    _ui.robotStatusList->itemWidget(
                        status_item_it->second.get()));
            }

            if (status_widget != nullptr) {
                status_widget->loadFromLogFrame(rx, maybe_robot,
                                                current_frame->blue_team());
            }
        }
    }

    if (!_game_settings_valid) {
        auto game_settings_request =
            std::make_shared<rj_msgs::srv::SetGameSettings::Request>();
        game_settings_request->game_settings = _game_settings;
        _set_game_settings->async_send_request(game_settings_request);
        _game_settings_valid = true;
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

    if (_processor->gameplayModule()->checkPlaybookStatus()) {
        playIndicatorStatus(false);
    }

    // Some conditions are different in simulation
    bool sim = _game_settings.simulation;

    if (!sim) {
        updateRadioBaseStatus(_processor->isRadioOpen());
    }

    // Get processing thread status
    Processor::Status ps = _processor->status();
    RJ::Time cur_time = RJ::now();

    // Determine if we are receiving packets from an external referee
    // TODO(#1557): if we stop getting referee packets, set this to false.
    bool referee_updated = _has_external_ref;

    std::vector<int> valid_ids = _processor->state()->ourValidIds();

    for (int i = 1; i <= Num_Shells; i++) {
        QStandardItem* item = goalieModel->item(i);
        if (std::find(valid_ids.begin(), valid_ids.end(), i - 1) !=
            valid_ids.end()) {
            // The list starts with None so i is 1 higher than the shell id
            item->setFlags(item->flags() |
                           (Qt::ItemIsSelectable | Qt::ItemIsEnabled));
        } else {
            item->setFlags(item->flags() &
                           ~(Qt::ItemIsSelectable | Qt::ItemIsEnabled));
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
        _ui.fastDirectBlue->setEnabled(false);
    } else {
        _ui.fastHalt->setEnabled(true);
        _ui.fastStop->setEnabled(true);
        _ui.fastReady->setEnabled(true);
        _ui.fastForceStart->setEnabled(true);
        _ui.fastKickoffBlue->setEnabled(true);
        _ui.fastKickoffYellow->setEnabled(true);
        _ui.fastDirectBlue->setEnabled(true);
    }

    updateFromRefPacket(_has_external_ref);

    // Is the processing thread running?
    if (cur_time - ps.lastLoopTime > RJ::Seconds(0.1)) {
        // Processing loop hasn't run recently.
        // Likely causes:
        //    Mutex deadlock (need a recursive mutex?)
        //    Excessive computation
        status("PROCESSING HUNG", StatusType::Status_Fail);
        return;
    }

    // Check network activity
    if (cur_time - ps.lastVisionTime > RJ::Seconds(0.1)) {
        // We must always have vision
        status("NO VISION", StatusType::Status_Fail);
        return;
    }

#if MANUAL
    if (_context->game_settings.joystick_config.manualID >= 0) {
        // Mixed auto/manual control
        status("MANUAL", StatusType::Status_Warning);
        return;
    }
#endif

    // Driving the robots helps isolate radio problems by verifying radio TX,
    // so test this after manual driving.
    if (cur_time - ps.lastRadioRxTime > RJ::Seconds(1)) {
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

    if (!sim && _context->logs.state != Logs::State::kWriting) {
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

void MainWindow::playIndicatorStatus(bool color) {
    if (color) {
        _ui.playIndicatorStatus->setStyleSheet("background-color: #00ff00");
    } else {
        _ui.playIndicatorStatus->setStyleSheet("background-color: #ff0000");
    }
}

void MainWindow::updateRadioBaseStatus(bool usb_radio) {
    QString label =
        QString(usb_radio ? "Radio Connected" : "Radio Disconnected");
    if (_ui.radioBaseStatus->text() != label) {
        _ui.radioBaseStatus->setText(label);
        _ui.radioBaseStatus->setStyleSheet(
            usb_radio ? QString("background-color: #00ff00")
                      : QString("background-color: #ff4040"));
    }
}

void MainWindow::on_fieldView_robotSelected(int shell) {
    if (_context->joystick_valid) {
        _ui.manualID->setCurrentIndex(shell + 1);

#if MANUAL
        std::lock_guard<std::mutex> lock(*_context_mutex);
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
    grSim_Packet sim_packet;
    grSim_BallReplacement* ball_replace =
        sim_packet.mutable_replacement()->mutable_ball();

    ball_replace->set_x(0);
    ball_replace->set_y(0);
    ball_replace->set_vx(0);
    ball_replace->set_vy(0);

    std::lock_guard<std::mutex> lock(*_context_mutex);
    _context->grsim_command = sim_packet;
}

void MainWindow::on_actionStopBall_triggered() {
    grSim_Packet sim_packet;
    grSim_BallReplacement* ball_replace =
        sim_packet.mutable_replacement()->mutable_ball();

    Geometry2d::Point ball_pos =
        _ui.fieldView->getTeamToWorld() * state()->ball->position;
    ball_replace->set_x(ball_pos.x());
    ball_replace->set_y(ball_pos.y());
    ball_replace->set_vx(0);
    ball_replace->set_vy(0);

    std::lock_guard<std::mutex> lock(*_context_mutex);
    _context->grsim_command = sim_packet;
}

void MainWindow::on_actionResetField_triggered() {
    grSim_Packet sim_packet;

    grSim_Replacement* replacement = sim_packet.mutable_replacement();
    for (int i = 0; i < Robots_Per_Team; ++i) {
        auto* rob = replacement->add_robots();

        const int num_cols = 2;
        const int robots_per_col = Robots_Per_Team / num_cols;

        double x_pos = -2.5 + i / robots_per_col;
        double y_pos = i % robots_per_col - robots_per_col / num_cols;

        rob->set_x(x_pos);
        rob->set_y(y_pos);
        rob->set_dir(0);
        rob->set_id(i);
        rob->set_yellowteam(false);
    }

    for (int i = 0; i < Robots_Per_Team; ++i) {
        auto rob = replacement->add_robots();

        const int num_cols = 2;
        const int robots_per_col = Robots_Per_Team / num_cols;

        double x_pos = +2.5 - i / robots_per_col;
        double y_pos = i % robots_per_col - robots_per_col / num_cols;

        rob->set_x(x_pos);
        rob->set_y(y_pos);
        rob->set_dir(180);
        rob->set_id(i);
        rob->set_yellowteam(true);
    }

    auto ball_replace = replacement->mutable_ball();
    ball_replace->set_x(0.0);
    ball_replace->set_y(0.0);
    ball_replace->set_vx(0.0);
    ball_replace->set_vy(0.0);

    std::lock_guard<std::mutex> lock(*_context_mutex);
    _context->grsim_command = sim_packet;
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

    std::lock_guard<std::mutex> lock(*_context_mutex);
    _context->game_settings.joystick_config.dampedRotation = value;
#endif
}

void MainWindow::on_actionDampedTranslation_toggled(bool value) {
#if MANUAL
    cout << "DampedTranslation is ";
    if (value)
        cout << "Enabled" << endl;
    else
        cout << "Disabled" << endl;

    std::lock_guard<std::mutex> lock(*_context_mutex);
    _context->game_settings.joystick_config.dampedTranslation = value;
#endif
}

void MainWindow::on_actionRestartUpdateTimer_triggered() {
    printf("Update timer: active %d, singleShot %d, interval %d\n",
           static_cast<int>(updateTimer.isActive()),
           static_cast<int>(updateTimer.isSingleShot()),
           updateTimer.interval());
    updateTimer.stop();
    updateTimer.start(30);
}

void MainWindow::on_actionStart_Logging_triggered() {
    if (_context->logs.state != Logs::State::kWriting) {
        if (!QDir("logs").exists()) {
            QDir().mkdir("logs");
        }

        QString log_file =
            QString("logs/") +
            QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss.log");

        if (!_processor->openLog(log_file)) {
            printf("Failed to open %s: %m\n", (const char*)log_file.toLatin1());
        } else {
            _ui.actionStart_Logging->setText(QString("Now Logging to:") +
                                             log_file);
            _ui.actionStart_Logging->setEnabled(false);
        }
    }
}

// Gameplay commands

void MainWindow::on_actionSeed_triggered() {
    QString text =
        QInputDialog::getText(this, "Set Random Seed", "Hexadecimal seed:");
    if (!text.isNull()) {
        long seed = strtol(text.toLatin1(), nullptr, 16);
        printf("seed %016lx\n", seed);
        srand48(seed);
    }
}

// Joystick settings
void MainWindow::on_joystickKickOnBreakBeam_stateChanged() {
#if MANUAL
    std::lock_guard<std::mutex> lock(*_context_mutex);
    _context->game_settings.joystick_config.useKickOnBreakBeam =
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

void MainWindow::on_logHistoryLocation_sliderReleased() {
    on_logHistoryLocation_sliderPressed();
}

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

    update_cache(_game_settings.request_blue_team, false,
                 &_game_settings_valid);
}

void MainWindow::on_manualID_currentIndexChanged(int value) {
#if MANUAL
    _context->game_settings.joystick_config.manualID = value - 1;
#endif
}

void MainWindow::on_actionUse_Field_Oriented_Controls_toggled(bool value) {
#if MANUAL
    _context->game_settings.joystick_config.useFieldOrientedDrive = value;
#endif
}

void MainWindow::on_actionUse_Multiple_Joysticks_toggled(bool value) {
    // TODO(Kyle): Reimplement multiple manual
}

void MainWindow::on_goalieID_currentIndexChanged(int value) {
    update_cache(_game_settings.request_goalie_id, value - 1,
                 &_game_settings_valid);
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
    QAction* not_single = nullptr;
    if (item != nullptr) {
        single = menu.addAction("Only this");
        not_single = menu.addAction("All except this");
    }

    QAction* act = menu.exec(_ui.debugLayers->mapToGlobal(pos));
    if (act == all) {
        allDebugOn();
    } else if (act == none) {
        allDebugOff();
    } else if ((single != nullptr) && act == single) {
        allDebugOff();
        item->setCheckState(Qt::Checked);
    } else if ((not_single != nullptr) && act == not_single) {
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

void MainWindow::on_configTree_itemChanged(QTreeWidgetItem* item, int column) {}

void MainWindow::on_loadConfig_clicked() {
    QString filename = QFileDialog::getOpenFileName(this, "Load Configuration");
    if (!filename.isNull()) {
        QString error;
        if (!_config->load(filename, error)) {
            QMessageBox::critical(this, "Load Configuration", error);
        }
    }
}

void MainWindow::on_saveConfig_clicked() {
    QString filename = QFileDialog::getSaveFileName(this, "Save Configuration");
    if (!filename.isNull()) {
        QString error;
        if (!_config->save(filename, error)) {
            QMessageBox::critical(this, "Save Configuration", error);
        }
    }
}

void MainWindow::on_loadPlaybook_clicked() {
    QString filename = QFileDialog::getOpenFileName(
        this, "Load Playbook",
        ApplicationRunDirectory().filePath("../soccer/gameplay/playbooks/"));
    if (!filename.isNull()) {
        try {
            _processor->gameplayModule()->loadPlaybook(filename.toStdString(),
                                                       true);
            playIndicatorStatus(true);
        } catch (const runtime_error&) {
            QMessageBox::critical(this, "File not found",
                                  QString("File not found: %1").arg(filename));
        }
    }
}

void MainWindow::on_savePlaybook_clicked() {
    QString filename = QFileDialog::getSaveFileName(
        this, "Save Playbook",
        ApplicationRunDirectory().filePath("../soccer/gameplay/playbooks/"));
    if (!filename.isNull()) {
        try {
            _processor->gameplayModule()->savePlaybook(filename.toStdString(),
                                                       true);
            playIndicatorStatus(true);
        } catch (const runtime_error&) {
            QMessageBox::critical(this, "File not found",
                                  QString("File not found: %1").arg(filename));
        }
    }
}

void MainWindow::on_clearPlays_clicked() {
    _processor->gameplayModule()->clearPlays();
    playIndicatorStatus(true);
}

////////
// Testing Tab

void MainWindow::on_testRun_clicked() {
    _processor->gameplayModule()->loadTest();
}

void MainWindow::on_addToTable_clicked() {
    _processor->gameplayModule()->addTests();
}

void MainWindow::on_removeFromTable_clicked() {
    _processor->gameplayModule()->removeTest();
}

void MainWindow::on_testNext_clicked() {
    _processor->gameplayModule()->nextTest();
}

// NOLINTNEXTLINE(readability-make-member-function-const): this modifies state
void MainWindow::setUseRefChecked(bool /* use_ref */) {
    _ui.actionUse_Field_Oriented_Controls->setChecked(false);
}

void MainWindow::send_quick_command(int command) {
    auto request = std::make_shared<rj_msgs::srv::QuickCommands::Request>();
    request->state = command;
    _quick_commands_srv->async_send_request(request);
}

void MainWindow::send_quick_restart(int restart, bool blue_team) {
    auto request = std::make_shared<rj_msgs::srv::QuickRestart::Request>();
    request->restart = restart;
    request->blue_team = blue_team;
    _quick_restart_srv->async_send_request(request);
}

void MainWindow::on_fastHalt_clicked() {
    send_quick_command(rj_msgs::srv::QuickCommands::Request::COMMAND_HALT);
}

void MainWindow::on_fastStop_clicked() {
    send_quick_command(rj_msgs::srv::QuickCommands::Request::COMMAND_STOP);
}

void MainWindow::on_fastReady_clicked() {
    send_quick_command(rj_msgs::srv::QuickCommands::Request::COMMAND_READY);
}

void MainWindow::on_fastForceStart_clicked() {
    send_quick_command(rj_msgs::srv::QuickCommands::Request::COMMAND_PLAY);
}

void MainWindow::on_fastKickoffBlue_clicked() {
    send_quick_restart(rj_msgs::srv::QuickRestart::Request::RESTART_KICKOFF,
                       true);
}

void MainWindow::on_fastKickoffYellow_clicked() {
    send_quick_restart(rj_msgs::srv::QuickRestart::Request::RESTART_KICKOFF,
                       false);
}

void MainWindow::on_fastDirectBlue_clicked() {
    send_quick_restart(rj_msgs::srv::QuickRestart::Request::RESTART_DIRECT,
                       true);
}

void MainWindow::on_fastDirectYellow_clicked() {
    send_quick_restart(rj_msgs::srv::QuickRestart::Request::RESTART_DIRECT,
                       false);
}

void MainWindow::on_fastIndirectBlue_clicked() {
    send_quick_restart(rj_msgs::srv::QuickRestart::Request::RESTART_INDIRECT,
                       true);
}

void MainWindow::on_fastIndirectYellow_clicked() {
    send_quick_restart(rj_msgs::srv::QuickRestart::Request::RESTART_INDIRECT,
                       false);
}

bool MainWindow::live() { return !_playbackRate; }
void MainWindow::updateDebugLayers(const LogFrame& frame) {
    // Check if any debug layers have been added
    // (layers should never be removed)
    if (frame.debug_layers_size() > _ui.debugLayers->count()) {
        // Add the missing layers and turn them on
        for (int i = _ui.debugLayers->count(); i < frame.debug_layers_size();
             ++i) {
            const QString name = QString::fromStdString(frame.debug_layers(i));
            bool enabled = !std::any_of(
                kDefaultHiddenLayers.begin(), kDefaultHiddenLayers.end(),
                [&](const QString& string) { return string == name; });
            addLayer(i, name, enabled);
        }

        _ui.debugLayers->sortItems();
    }
}
