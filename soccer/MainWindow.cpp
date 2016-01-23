#include <gameplay/GameplayModule.hpp>
#include "MainWindow.hpp"
#include "Configuration.hpp"
#include "QuaternionDemo.hpp"
#include "radio/Radio.hpp"
#include <Utils.hpp>
#include <Robot.hpp>
#include <joystick/Joystick.hpp>
#include "RobotStatusWidget.hpp"
#include "BatteryProfile.hpp"
#include <Network.hpp>

#include <QInputDialog>
#include <QFileDialog>
#include <QActionGroup>
#include <QMessageBox>

#include <iostream>
#include <ctime>

#include <google/protobuf/descriptor.h>

using namespace std;
using namespace boost;
using namespace google::protobuf;
using namespace Packet;
using namespace Eigen;

// Style sheets used for live/non-live controls
QString LiveStyle("border:2px solid transparent");
QString NonLiveStyle("border:2px solid red");

static const std::vector<QString> defaultHiddenLayers{
    "MotionControl", "Global Obstacles", "Local Obstacles",
    "Planning0",     "Planning1",        "Planning2",
    "Planning3",     "Planning4",        "Planning5"};

void calcMinimumWidth(QWidget* widget, QString text) {
    QRect rect = QFontMetrics(widget->font()).boundingRect(text);
    widget->setMinimumWidth(rect.width());
}

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    qRegisterMetaType<QVector<int>>("QVector<int>");

    _quaternion_demo = nullptr;

    _updateCount = 0;
    _processor = nullptr;
    _autoExternalReferee = true;
    _doubleFrameNumber = -1;

    _lastUpdateTime = RJ::timestamp();
    _history.resize(2 * 60);

    _ui.setupUi(this);
    _ui.fieldView->history(&_history);

    // Set the width of logTime to prevent it from vibrating
    calcMinimumWidth(_ui.logTime, "888:88.8");

    _ui.logTree->history(&_history);
    _ui.logTree->mainWindow = this;
    _ui.logTree->updateTimer = &updateTimer;

    // Initialize live/non-live control styles
    _live = false;
    live(true);

    _currentPlay = new QLabel();
    _currentPlay->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    _currentPlay->setToolTip("Current Play");
    _currentPlay->setAlignment(Qt::AlignCenter);
    _currentPlay->setObjectName("current_play_name");
    calcMinimumWidth(_currentPlay, "XXXXXXXXXXXXXXXX");
    statusBar()->addPermanentWidget(_currentPlay);

    _logFile = new QLabel();
    _logFile->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    _logFile->setToolTip("Log File");
    statusBar()->addPermanentWidget(_logFile);

    _viewFPS = new QLabel();
    _viewFPS->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    _viewFPS->setToolTip("Display Framerate");
    calcMinimumWidth(_viewFPS, "View: 00.0 fps");
    statusBar()->addPermanentWidget(_viewFPS);

    _procFPS = new QLabel();
    _procFPS->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    _procFPS->setToolTip("Processing Framerate");
    calcMinimumWidth(_procFPS, "Proc: 00.0 fps");
    statusBar()->addPermanentWidget(_procFPS);

    _logMemory = new QLabel();
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

    QActionGroup* goalGroup = new QActionGroup(this);
    goalGroup->addAction(_ui.actionDefendMinusX);
    goalGroup->addAction(_ui.actionDefendPlusX);

    QActionGroup* rotateGroup = new QActionGroup(this);
    rotateGroup->addAction(_ui.action0);
    rotateGroup->addAction(_ui.action90);
    rotateGroup->addAction(_ui.action180);
    rotateGroup->addAction(_ui.action270);

    connect(_ui.manualID, SIGNAL(currentIndexChanged(int)), this,
            SLOT(on_manualID_currentIndexChanged(int)));

    channel(0);

    updateTimer.setSingleShot(true);
    connect(&updateTimer, SIGNAL(timeout()), SLOT(updateViews()));
    updateTimer.start(30);

    // put all log playback buttons into a vector for easy access later
    _logPlaybackButtons.push_back(_ui.logPlaybackRewind);
    _logPlaybackButtons.push_back(_ui.logPlaybackPrevFrame);
    _logPlaybackButtons.push_back(_ui.logPlaybackPause);
    _logPlaybackButtons.push_back(_ui.logPlaybackNextFrame);
    _logPlaybackButtons.push_back(_ui.logPlaybackPlay);
    _logPlaybackButtons.push_back(_ui.logPlaybackLive);
}

void MainWindow::configuration(Configuration* config) {
    _config = config;
    _config->tree(_ui.configTree);
}

void MainWindow::processor(Processor* value) {
    // This should only happen once
    assert(!_processor);

    _processor = value;

    // External referee
    // on_externalReferee_toggled(_ui.externalReferee->isChecked());

    // Team
    if (_processor->blueTeam()) {
        _ui.actionTeamBlue->trigger();
    } else {
        _ui.actionTeamYellow->trigger();
    }

    _ui.logHistoryLocation->setMaximum(_processor->logger().maxFrames());
    _ui.logHistoryLocation->setTickInterval(60 * 60);  // interval is ~ 1 minute
}

void MainWindow::logFileChanged() {
    if (_processor->logger().recording()) {
        _logFile->setText(_processor->logger().filename());
    } else {
        _logFile->setText("Not Recording");
    }
}

void MainWindow::live(bool value) {
    if (_live != value) {
        _live = value;

        // Change styles for controls that can show historical data
        _ui.fieldView->live = _live;
        if (_live) {
            _ui.logTree->setStyleSheet(
                QString("QTreeWidget{%1}").arg(LiveStyle));
        } else {
            _ui.logTree->setStyleSheet(
                QString("QTreeWidget{%1}").arg(NonLiveStyle));
        }
    }
}

void MainWindow::addLayer(int i, QString name, bool checked) {
    QListWidgetItem* item = new QListWidgetItem(name);
    Qt::CheckState checkState = checked ? Qt::Checked : Qt::Unchecked;
    item->setCheckState(checkState);
    item->setData(Qt::UserRole, i);
    _ui.debugLayers->addItem(item);
    on_debugLayers_itemChanged(item);
}

void MainWindow::updateViews() {
    int manual = _processor->manualID();
    if ((manual >= 0 || _ui.manualID->isEnabled()) &&
        !_processor->joystickValid()) {
        // Joystick is gone - turn off manual control
        _ui.manualID->setCurrentIndex(0);
        _processor->manualID(-1);
        _ui.manualID->setEnabled(false);
        _ui.tabWidget->setTabEnabled(_ui.tabWidget->indexOf(_ui.joystickTab),
                                     false);
    } else if (!_ui.manualID->isEnabled() && _processor->joystickValid()) {
        // Joystick reconnected
        _ui.manualID->setEnabled(true);
        _ui.joystickTab->setVisible(true);
        _ui.tabWidget->setTabEnabled(_ui.tabWidget->indexOf(_ui.joystickTab),
                                     true);
    }
    if (manual >= 0) {
        JoystickControlValues vals = _processor->getJoystickControlValues();
        _ui.joystickBodyXLabel->setText(tr("%1").arg(vals.translation.x));
        _ui.joystickBodyYLabel->setText(tr("%1").arg(vals.translation.y));
        _ui.joystickBodyWLabel->setText(tr("%1").arg(vals.rotation));
        _ui.joystickKickPowerLabel->setText(tr("%1").arg(vals.kickPower));
        _ui.joystickDibblerPowerLabel->setText(
            tr("%1").arg(vals.dribblerPower));
        _ui.joystickKickCheckBox->setChecked(vals.kick);
        _ui.joystickChipCheckBox->setChecked(vals.chip);
        _ui.joystickDribblerCheckBox->setChecked(vals.dribble);
    }

    // Time since last update
    RJ::Time time = RJ::timestamp();
    int delta_us = time - _lastUpdateTime;
    _lastUpdateTime = time;
    double framerate = 1000000.0 / delta_us;

    ++_updateCount;
    if (_updateCount == 4) {
        _updateCount = 0;

        _viewFPS->setText(QString("View: %1 fps").arg(framerate, 0, 'f', 1));
        _procFPS->setText(
            QString("Proc: %1 fps").arg(_processor->framerate(), 0, 'f', 1));

        _logMemory->setText(
            QString("Log: %1/%2 %3 kiB")
                .arg(QString::number(_processor->logger().numFrames()),
                     QString::number(_processor->logger().maxFrames()),
                     QString::number((_processor->logger().spaceUsed() + 512) /
                                     1024)));
    }

    // Advance log history
    int liveFrameNumber = _processor->logger().lastFrameNumber();
    if (_live) {
        _doubleFrameNumber = liveFrameNumber;
    } else {
        _doubleFrameNumber += _playbackRate;

        int minFrame = _processor->logger().firstFrameNumber();
        int maxFrame = _processor->logger().lastFrameNumber();

        if (_doubleFrameNumber < minFrame) {
            _doubleFrameNumber = minFrame;
        } else if (_doubleFrameNumber > maxFrame) {
            _doubleFrameNumber = maxFrame;
        }
    }

    // update history slider in ui
    emit historyLocationChanged(_doubleFrameNumber -
                                _processor->logger().firstFrameNumber());

    // Read recent history from the log
    _processor->logger().getFrames(frameNumber(), _history);

    // Update field view
    _ui.fieldView->update();

    // enable playback buttons based on playback rate
    for (QPushButton* playbackBtn : _logPlaybackButtons)
        playbackBtn->setEnabled(true);
    if (_live)
        _ui.logPlaybackLive->setEnabled(false);
    else if (_playbackRate < -0.1)
        _ui.logPlaybackRewind->setEnabled(false);
    else if (abs<float>(_playbackRate) < 0.01)
        _ui.logPlaybackPause->setEnabled(false);
    if (_playbackRate > 0.1 || _live) _ui.logPlaybackPlay->setEnabled(false);

    //  enable previous frame button based on position in the log
    _ui.logPlaybackPrevFrame->setEnabled(_doubleFrameNumber >= 1);
    _ui.logPlaybackNextFrame->setEnabled(!_live);

    // Update status indicator
    updateStatus();

    // Check if any debug layers have been added
    // (layers should never be removed)
    const std::shared_ptr<LogFrame> liveFrame =
        _processor->logger().lastFrame();
    if (liveFrame &&
        liveFrame->debug_layers_size() > _ui.debugLayers->count()) {
        // Add the missing layers and turn them on
        for (int i = _ui.debugLayers->count();
             i < liveFrame->debug_layers_size(); ++i) {
            const QString name =
                QString::fromStdString(liveFrame->debug_layers(i));
            bool enabled =
                !std::any_of(defaultHiddenLayers.begin(),
                             defaultHiddenLayers.end(),
                             [&](QString string) { return string == name; });
            addLayer(i, name, enabled);
        }

        _ui.debugLayers->sortItems();
    }

    // Get the frame at the log playback time
    const std::shared_ptr<LogFrame> currentFrame = _history[0];

    if (currentFrame) {
        if (_firstLogTimestamp == -1)
            _firstLogTimestamp = currentFrame->timestamp();
        uint64_t gametime_ms =
            (currentFrame->timestamp() - _firstLogTimestamp) / 1000;
        uint64_t minutes = gametime_ms / 60000;
        uint64_t seconds = (gametime_ms % 60000) / 1000;
        uint64_t deciseconds = (gametime_ms % 1000) / 100;
        _ui.logTime->setText(QString::fromStdString(to_string(minutes) + " : " +
                                                    to_string(seconds) + "." +
                                                    to_string(deciseconds)));

        // Update the orientation demo view
        if (_quaternion_demo && manual >= 0 &&
            currentFrame->radio_rx().size() &&
            currentFrame->radio_rx(0).has_quaternion()) {
            const RadioRx* manualRx = nullptr;
            for (const RadioRx& rx : currentFrame->radio_rx()) {
                if ((int)rx.robot_id() == manual) {
                    manualRx = &rx;
                    break;
                }
            }
            if (manualRx) {
                const Packet::Quaternion& q = manualRx->quaternion();
                _quaternion_demo->q =
                    Quaternionf(q.q0(), q.q1(), q.q2(), q.q3());
                if (!_quaternion_demo->initialized) {
                    _quaternion_demo->ref = _quaternion_demo->q;
                    _quaternion_demo->initialized = true;
                }
                _quaternion_demo->update();
            }
        }

        // Update non-message tree items
        _frameNumberItem->setData(ProtobufTree::Column_Value, Qt::DisplayRole,
                                  frameNumber());
        int elapsedMillis =
            (currentFrame->command_time() - _processor->firstLogTime + 500) /
            1000;
        QTime elapsedTime = QTime().addMSecs(elapsedMillis);
        _elapsedTimeItem->setText(ProtobufTree::Column_Value,
                                  elapsedTime.toString("hh:mm:ss.zzz"));

        // Sort the tree by tag if items have been added
        if (_ui.logTree->message(*currentFrame)) {
            // Items have been added, so sort again on tag number
            _ui.logTree->sortItems(ProtobufTree::Column_Tag,
                                   Qt::AscendingOrder);
        }

        // update the behavior tree view
        _ui.behaviorTree->setPlainText(
            QString::fromStdString(currentFrame->behavior_tree()));
    }

    if (std::time(nullptr) -
            (_processor->refereeModule()->received_time / 1000000) >
        1) {
        _ui.fastHalt->setEnabled(true);
        _ui.fastStop->setEnabled(true);
        _ui.fastReady->setEnabled(true);
        _ui.fastForceStart->setEnabled(true);
        _ui.fastKickoffBlue->setEnabled(true);
        _ui.fastKickoffYellow->setEnabled(true);
    } else {
        _ui.fastHalt->setEnabled(false);
        _ui.fastStop->setEnabled(false);
        _ui.fastReady->setEnabled(false);
        _ui.fastForceStart->setEnabled(false);
        _ui.fastKickoffBlue->setEnabled(false);
        _ui.fastKickoffYellow->setEnabled(false);
    }

    _ui.refStage->setText(NewRefereeModuleEnums::stringFromStage(
                              _processor->refereeModule()->stage).c_str());
    _ui.refCommand->setText(NewRefereeModuleEnums::stringFromCommand(
                                _processor->refereeModule()->command).c_str());

    // convert time left from ms to s and display it to two decimal places
    _ui.refTimeLeft->setText(tr("%1 s").arg(QString::number(
        _processor->refereeModule()->stage_time_left / 1000.0f, 'f', 2)));

    const char* blueName = _processor->refereeModule()->blue_info.name.c_str();
    _ui.refBlueName->setText(strlen(blueName) == 0 ? "<Blue Team>" : blueName);
    _ui.refBlueScore->setText(
        tr("%1").arg(_processor->refereeModule()->blue_info.score));
    _ui.refBlueRedCards->setText(
        tr("%1").arg(_processor->refereeModule()->blue_info.red_cards));
    _ui.refBlueYellowCards->setText(
        tr("%1").arg(_processor->refereeModule()->blue_info.yellow_cards));
    _ui.refBlueTimeoutsLeft->setText(
        tr("%1").arg(_processor->refereeModule()->blue_info.timeouts_left));
    _ui.refBlueGoalie->setText(
        tr("%1").arg(_processor->refereeModule()->blue_info.goalie));

    const char* yellowName =
        _processor->refereeModule()->yellow_info.name.c_str();
    _ui.refYellowName->setText(strlen(yellowName) == 0 ? "<Yellow Team>"
                                                       : yellowName);
    _ui.refYellowScore->setText(
        tr("%1").arg(_processor->refereeModule()->yellow_info.score));
    _ui.refYellowRedCards->setText(
        tr("%1").arg(_processor->refereeModule()->yellow_info.red_cards));
    _ui.refYellowYellowCards->setText(
        tr("%1").arg(_processor->refereeModule()->yellow_info.yellow_cards));
    _ui.refYellowTimeoutsLeft->setText(
        tr("%1").arg(_processor->refereeModule()->yellow_info.timeouts_left));
    _ui.refYellowGoalie->setText(
        tr("%1").arg(_processor->refereeModule()->yellow_info.goalie));

    _ui.actionUse_External_Referee->setChecked(
        _processor->refereeModule()->useExternalReferee());

    // update robot status list
    for (const OurRobot* robot : _processor->state()->self) {
        // a robot shows up in the status list if it's reachable via radio
        bool shouldDisplay = robot->rxIsFresh();

        // see if it's already in the robot status list widget
        bool displaying = _robotStatusItemMap.find(robot->shell()) !=
                          _robotStatusItemMap.end();

        if (shouldDisplay && !displaying) {
            // add a widget to the list for this robot

            QListWidgetItem* item = new QListWidgetItem();
            _robotStatusItemMap[robot->shell()] = item;
            _ui.robotStatusList->addItem(item);

            RobotStatusWidget* statusWidget = new RobotStatusWidget();
            item->setSizeHint(statusWidget->minimumSizeHint());
            _ui.robotStatusList->setItemWidget(item, statusWidget);

            // set shell ID
            statusWidget->setShellID(robot->shell());

            // set team
            statusWidget->setBlueTeam(processor()->blueTeam());

            // TODO: set board ID

            // set robot model
            QString robotModel;
            switch (robot->radioRx().hardware_version()) {
                case RJ2008:
                    robotModel = "RJ2008";
                    break;
                case RJ2011:
                    robotModel = "RJ2011";
                    break;
                case RJ2015:
                    robotModel = "RJ2015";
                    break;
                default:
                    robotModel = "Unknown Bot";
            }
            statusWidget->setRobotModel(robotModel);

// uncomment this #define to test the display of a variety of different errors
// #define DEMO_ROBOT_STATUS

#ifdef DEMO_ROBOT_STATUS
            // set board ID
            QString hex("");
            for (int i = 0; i < 4; i++)
                hex += QString::number(rand() % 16, 16).toUpper();
            statusWidget->setBoardID(hex);

            // fake vision
            bool vision = rand() % 5 != 0;
            statusWidget->setHasVision(vision);

            // fake battery
            float battery = robot->shell() / 6.0f;
            statusWidget->setBatteryLevel(battery);

            // fake radio
            bool radio = rand() % 5 != 0;
            statusWidget->setHasRadio(radio);

            // fake error text
            QString error = "Kicker Fault, Hall Fault FR, Ball Sense Fault";
            statusWidget->setErrorText(error);

            // fake ball status
            bool ball = rand() % 4 == 0;
            statusWidget->setHasBall(ball);

            // fake ball sense error
            bool ballFault = rand() % 4 == 0;
            statusWidget->setBallSenseFault(ballFault);
            bool hasWheelFault = false;
            if (rand() % 4 == 0) {
                statusWidget->setWheelFault(rand() % 4);
                hasWheelFault = true;
            }

            bool showstopper =
                !vision || !radio || hasWheelFault || battery < 0.25;
            statusWidget->setShowstopper(showstopper);
#endif
        } else if (!shouldDisplay && displaying) {
            // remove the widget for this robot from the list

            QListWidgetItem* item = _robotStatusItemMap[robot->shell()];

            // delete widget from list
            for (int row = 0; row < _ui.robotStatusList->count(); row++) {
                if (_ui.robotStatusList->item(row) == item) {
                    _ui.robotStatusList->takeItem(row);
                    break;
                }
            }

            _robotStatusItemMap.erase(robot->shell());
            delete item;
        }

        // update displayed attributes for valid robots
        if (shouldDisplay) {
            QListWidgetItem* item = _robotStatusItemMap[robot->shell()];
            RobotStatusWidget* statusWidget =
                (RobotStatusWidget*)_ui.robotStatusList->itemWidget(item);

            // We make a copy of the robot's RadioRx package b/c the original
            // might change during the course of this method b/c radio comm
            // happens on a different thread.
            RadioRx rx(robot->radioRx());

#ifndef DEMO_ROBOT_STATUS
            // radio status
            bool hasRadio = robot->rxIsFresh();
            statusWidget->setHasRadio(hasRadio);

            // vision status
            bool hasVision = robot->visible;
            statusWidget->setHasVision(hasVision);

            // build a list of errors to display in the widget
            QStringList errorList;

            // motor faults
            // each motor fault is shown as text in the error text display as
            // well as being drawn as a red X on the graphic of a robot
            bool hasMotorFault = false;
            if (rx.motor_status().size() == 5) {
                const char* motorNames[] = {"FR", "FL", "BL", "BR", "Dribbler"};

                // examine status of each motor (including the dribbler)
                for (int i = 0; i < 5; ++i) {
                    bool motorIFault = true;
                    switch (rx.motor_status(i)) {
                        case Packet::Hall_Failure:
                            errorList
                                << QString("Hall Fault %1").arg(motorNames[i]);
                            break;
                        case Packet::Stalled:
                            errorList << QString("Stall %1").arg(motorNames[i]);
                            break;
                        case Packet::Encoder_Failure:
                            errorList << QString("Encoder Fault %1")
                                             .arg(motorNames[i]);
                            break;

                        default:
                            motorIFault = false;
                            break;
                    }

                    // show wheel faults (exluding dribbler, which is index 4)
                    if (i != 4) statusWidget->setWheelFault(i, motorIFault);

                    hasMotorFault = hasMotorFault || motorIFault;

                    // show dribbler fault on painted robot widget
                    if (i == 4) statusWidget->setBallSenseFault(motorIFault);
                }
            }

            // check for kicker error code
            bool kickerFault =
                rx.has_kicker_status() && (rx.kicker_status() & 0x80);
            bool ballSenseFault = rx.has_ball_sense_status() &&
                                  !(rx.ball_sense_status() == Packet::NoBall ||
                                    rx.ball_sense_status() == Packet::HasBall);
            if (kickerFault) errorList << "Kicker Fault";
            if (ballSenseFault) errorList << "Ball Sense Fault";
            statusWidget->setBallSenseFault(ballSenseFault);

            // display error text
            statusWidget->setErrorText(errorList.join(", "));

            // show the ball in the robot's mouth if it has one
            bool hasBall = rx.has_ball_sense_status() &&
                           rx.ball_sense_status() == Packet::HasBall;
            statusWidget->setHasBall(hasBall);

            // battery
            // convert battery voltage to a percentage and show it with the
            // battery indicator
            float batteryLevel = 1;
            if (rx.has_battery()) {
                if (rx.hardware_version() == RJ2008 ||
                    rx.hardware_version() == RJ2011) {
                    batteryLevel =
                        RJ2008BatteryProfile.getChargeLevel(rx.battery());
                } else if (rx.hardware_version() == RJ2015) {
                    batteryLevel =
                        RJ2015BatteryProfile.getChargeLevel(rx.battery());
                } else {
                    cerr << "Unknown hardware revision "
                         << rx.hardware_version()
                         << ", unable to calculate battery %" << endl;
                }
            }
            statusWidget->setBatteryLevel(batteryLevel);

            // if there is an error bad enough that we should get this robot
            // off the field, alert the user through the UI that there is a
            // "showstopper"
            bool showstopper = !hasVision || !hasRadio || hasMotorFault ||
                               kickerFault || ballSenseFault ||
                               (batteryLevel < 0.25);
            statusWidget->setShowstopper(showstopper);

#endif
        }
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

    if (!_processor) {
        status("NO PROCESSOR", Status_Fail);
        return;
    }

    // Some conditions are different in simulation
    bool sim = _processor->simulation();

    // Get processing thread status
    Processor::Status ps = _processor->status();
    RJ::Time curTime = RJ::timestamp();

    // Determine if we are receiving packets from an external referee
    bool haveExternalReferee = (curTime - ps.lastRefereeTime) < 500 * 1000;

    /*if (_autoExternalReferee && haveExternalReferee &&
    !_ui.externalReferee->isChecked())
    {
        _ui.externalReferee->setChecked(true);
    }*/

    // Is the processing thread running?
    if (curTime - ps.lastLoopTime > 100 * 1000) {
        // Processing loop hasn't run recently.
        // Likely causes:
        //    Mutex deadlock (need a recursive mutex?)
        //    Excessive computation
        status("PROCESSING HUNG", Status_Fail);
        return;
    }

    // Check network activity
    if (curTime - ps.lastVisionTime > 100 * 1000) {
        // We must always have vision
        status("NO VISION", Status_Fail);
        return;
    }

    if (_processor->manualID() >= 0) {
        // Mixed auto/manual control
        status("MANUAL", Status_Warning);
        return;
    }

    // Driving the robots helps isolate radio problems by verifying radio TX,
    // so test this after manual driving.
    if (curTime - ps.lastRadioRxTime > 1000 * 1000) {
        // Allow a long timeout in case of poor radio performance
        status("NO RADIO RX", Status_Fail);
        return;
    }

    if ((!sim || _processor->externalReferee()) && !haveExternalReferee) {
        if (_autoExternalReferee && _processor->externalReferee()) {
            // Automatically turn off external referee
            //_ui.externalReferee->setChecked(false);
        } else {
            // In simulation, we will often run without a referee, so just make
            // it a warning.
            // There is a separate status for non-simulation with internal
            // referee.
            status("NO REFEREE", Status_Fail);
        }
        return;
    }

    if (sim) {
        // Everything is good for simulation, but not for competition.
        status("SIMULATION", Status_Warning);
        return;
    }

    if (!sim && !_processor->externalReferee()) {
        // Competition must use external referee
        status("INTERNAL REF", Status_Warning);
        return;
    }

    if (!sim && !_processor->logger().recording()) {
        // We should record logs during competition
        status("NOT RECORDING", Status_Warning);
        return;
    }

    status("COMPETITION", Status_OK);
}

void MainWindow::status(QString text, MainWindow::StatusType status) {
    // Assume that the status type alone won't change.
    if (_ui.statusLabel->text() != text) {
        _ui.statusLabel->setText(text);

        switch (status) {
            case Status_OK:
                _ui.statusLabel->setStyleSheet("background-color: #00ff00");
                break;

            case Status_Warning:
                _ui.statusLabel->setStyleSheet("background-color: #ffff00");
                break;

            case Status_Fail:
                _ui.statusLabel->setStyleSheet("background-color: #ff4040");
                break;
        }
    }
}

void MainWindow::on_fieldView_robotSelected(int shell) {
    if (_processor->joystickValid()) {
        _ui.manualID->setCurrentIndex(shell + 1);
        _processor->manualID(shell);
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
    _processor->defendPlusX(false);
}

void MainWindow::on_actionDefendPlusX_triggered() {
    _processor->defendPlusX(true);
}

void MainWindow::on_action0_triggered() { _ui.fieldView->rotate(0); }

void MainWindow::on_action90_triggered() { _ui.fieldView->rotate(1); }

void MainWindow::on_action180_triggered() { _ui.fieldView->rotate(2); }

void MainWindow::on_action270_triggered() { _ui.fieldView->rotate(3); }

void MainWindow::on_actionUseOurHalf_toggled(bool value) {
    _processor->useOurHalf(value);
}

void MainWindow::on_actionUseOpponentHalf_toggled(bool value) {
    _processor->useOpponentHalf(value);
}

void MainWindow::on_action904MHz_triggered() {
    channel(0);
    _ui.action904MHz->setChecked(true);
    _ui.action906MHz->setChecked(false);
}

void MainWindow::on_action906MHz_triggered() {
    channel(10);
    _ui.action904MHz->setChecked(false);
    _ui.action906MHz->setChecked(true);
}

void MainWindow::channel(int n) {
    if (_processor && _processor->radio()) {
        _processor->radio()->channel(n);
    }
    _ui.radioLabel->setText(QString("%1MHz").arg(904.0 + 0.2 * n, 0, 'f', 1));
}

// Simulator commands

void MainWindow::on_actionCenterBall_triggered() {
    SimCommand cmd;
    cmd.mutable_ball_pos()->set_x(0);
    cmd.mutable_ball_pos()->set_y(0);
    cmd.mutable_ball_vel()->set_x(0);
    cmd.mutable_ball_vel()->set_y(0);
    _ui.fieldView->sendSimCommand(cmd);
}

void MainWindow::on_actionStopBall_triggered() {
    SimCommand cmd;
    cmd.mutable_ball_vel()->set_x(0);
    cmd.mutable_ball_vel()->set_y(0);
    _ui.fieldView->sendSimCommand(cmd);
}

void MainWindow::on_actionResetField_triggered() {
    SimCommand cmd;
    cmd.set_reset(true);
    _ui.fieldView->sendSimCommand(cmd);
}

void MainWindow::on_actionStopRobots_triggered() {
    SimCommand cmd;
    // TODO: check that this handles threads properly
    for (OurRobot* robot : state()->self) {
        SimCommand::Robot* r = cmd.add_robots();
        r->set_shell(robot->shell());
        r->set_blue_team(processor()->blueTeam());
        r->mutable_vel()->set_x(0);
        r->mutable_vel()->set_y(0);
        r->set_w(0);
    }
    for (OpponentRobot* robot : state()->opp) {
        SimCommand::Robot* r = cmd.add_robots();
        r->set_shell(robot->shell());
        r->set_blue_team(processor()->blueTeam());
        r->mutable_vel()->set_x(0);
        r->mutable_vel()->set_y(0);
        r->set_w(0);
    }
    _ui.fieldView->sendSimCommand(cmd);
}

// Manual control commands

void MainWindow::on_actionDampedRotation_toggled(bool value) {
    cout << "DampedRotation is ";
    if (value)
        cout << "Enabled" << endl;
    else
        cout << "Disabled" << endl;
    _processor->dampedRotation(value);
}

void MainWindow::on_actionDampedTranslation_toggled(bool value) {
    cout << "DampedTranslation is ";
    if (value)
        cout << "Enabled" << endl;
    else
        cout << "Disabled" << endl;
    _processor->dampedTranslation(value);
}

// Debug commands

void MainWindow::on_actionRestartUpdateTimer_triggered() {
    printf("Update timer: active %d, singleShot %d, interval %d\n",
           updateTimer.isActive(), updateTimer.isSingleShot(),
           updateTimer.interval());
    updateTimer.stop();
    updateTimer.start(30);
}

void MainWindow::on_actionQuaternion_Demo_toggled(bool value) {
    if (value) {
        if (_quaternion_demo) delete _quaternion_demo;
        cout << "Starting Quaternion Demo" << endl;
        _quaternion_demo = new QuaternionDemo(this);
        _quaternion_demo->resize(640, 480);
    } else {
        cout << "Stopping Quaternion Demo" << endl;
        if (_quaternion_demo) delete _quaternion_demo;
    }
}

// Gameplay commands

void MainWindow::on_menu_Gameplay_aboutToShow() {}

void MainWindow::on_actionSeed_triggered() {
    QString text =
        QInputDialog::getText(this, "Set Random Seed", "Hexadecimal seed:");
    if (!text.isNull()) {
        long seed = strtol(text.toLatin1(), nullptr, 16);
        printf("seed %016lx\n", seed);
        srand48(seed);
    }
}

// Log controls
void MainWindow::on_logHistoryLocation_sliderMoved(int value) {
    // update current frame
    int minFrame = _processor->logger().firstFrameNumber();
    int maxFrame = _processor->logger().lastFrameNumber();
    _doubleFrameNumber = value + minFrame;
    _doubleFrameNumber = min<double>(maxFrame, _doubleFrameNumber);

    emit historyLocationChanged(_doubleFrameNumber - minFrame);

    // pause playback
    live(false);
    _playbackRate = 0;
}

void MainWindow::on_logPlaybackRewind_clicked() {
    live(false);
    _playbackRate = -1;
}

void MainWindow::on_logPlaybackPrevFrame_clicked() {
    live(false);
    _playbackRate = 0;
    _doubleFrameNumber -= 1;
}

void MainWindow::on_logPlaybackPause_clicked() {
    live(false);
    _playbackRate = 0;
}

void MainWindow::on_logPlaybackNextFrame_clicked() {
    live(false);
    _playbackRate = 0;
    _doubleFrameNumber += 1;
}

void MainWindow::on_logPlaybackPlay_clicked() {
    live(false);
    _playbackRate = 1;
}

void MainWindow::on_logPlaybackLive_clicked() { live(true); }

void MainWindow::on_actionTeamBlue_triggered() {
    _ui.team->setText("BLUE");
    _ui.team->setStyleSheet("background-color: #4040ff; color: #ffffff");
    _processor->blueTeam(true);
}

void MainWindow::on_actionTeamYellow_triggered() {
    _ui.team->setText("YELLOW");
    _ui.team->setStyleSheet("background-color: #ffff00");
    _processor->blueTeam(false);
}

void MainWindow::on_manualID_currentIndexChanged(int value) {
    _processor->manualID(value - 1);
    if (_quaternion_demo) {
        if (value == 0) {
            _quaternion_demo->hide();
        } else {
            _quaternion_demo->show();
            _quaternion_demo->initialized = false;
        }
    }
}

void MainWindow::on_actionUse_Field_Oriented_Controls_toggled(bool value) {
    _processor->setUseFieldOrientedManualDrive(value);
}

void MainWindow::on_goalieID_currentIndexChanged(int value) {
    _processor->goalieID(value - 1);
}

void MainWindow::on_actionUse_External_Referee_toggled(bool value) {
    _processor->refereeModule()->useExternalReferee(value);
}

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
    QAction* single = nullptr, * notSingle = nullptr;
    if (item) {
        single = menu.addAction("Only this");
        notSingle = menu.addAction("All except this");
    }

    QAction* act = menu.exec(_ui.debugLayers->mapToGlobal(pos));
    if (act == all) {
        allDebugOn();
    } else if (act == none) {
        allDebugOff();
    } else if (single && act == single) {
        allDebugOff();
        item->setCheckState(Qt::Checked);
    } else if (notSingle && act == notSingle) {
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
        this, "Load Playbook", "../soccer/gameplay/playbooks/");
    if (!filename.isNull()) {
        try {
            _processor->gameplayModule()->loadPlaybook(filename.toStdString(),
                                                       true);
        } catch (runtime_error* error) {
            QMessageBox::critical(this, "File not found",
                                  QString("File not found: %1").arg(filename));
        }
    }
}

void MainWindow::on_savePlaybook_clicked() {
    QString filename = QFileDialog::getSaveFileName(
        this, "Save Playbook", "../soccer/gameplay/playbooks/");
    if (!filename.isNull()) {
        try {
            _processor->gameplayModule()->savePlaybook(filename.toStdString(),
                                                       true);
        } catch (runtime_error* error) {
            QMessageBox::critical(this, "File not found",
                                  QString("File not found: %1").arg(filename));
        }
    }
}

void MainWindow::setRadioChannel(RadioChannels channel) {
    switch (channel) {
        case RadioChannels::MHz_904:
            this->on_action904MHz_triggered();
            break;
        case RadioChannels::MHz_906:
            this->on_action906MHz_triggered();
            break;
    }
}

void MainWindow::setUseRefChecked(bool use_ref) {
    _ui.actionUse_Field_Oriented_Controls->setChecked(false);
}

void MainWindow::on_fastHalt_clicked() {
    _processor->refereeModule()->command = NewRefereeModuleEnums::HALT;
}

void MainWindow::on_fastStop_clicked() {
    _processor->refereeModule()->command = NewRefereeModuleEnums::STOP;
}

void MainWindow::on_fastReady_clicked() {
    _processor->refereeModule()->command = NewRefereeModuleEnums::NORMAL_START;
}

void MainWindow::on_fastForceStart_clicked() {
    _processor->refereeModule()->command = NewRefereeModuleEnums::FORCE_START;
}

void MainWindow::on_fastKickoffBlue_clicked() {
    _processor->refereeModule()->command =
        NewRefereeModuleEnums::PREPARE_KICKOFF_BLUE;
}

void MainWindow::on_fastKickoffYellow_clicked() {
    _processor->refereeModule()->command =
        NewRefereeModuleEnums::PREPARE_KICKOFF_YELLOW;
}

void MainWindow::on_actionVisionPrimary_Half_triggered() {
    _processor->changeVisionChannel(SharedVisionPortSinglePrimary);
    _processor->setFieldDimensions(Field_Dimensions::Single_Field_Dimensions);
    _ui.actionVisionPrimary_Half->setChecked(true);
    _ui.actionVisionSecondary_Half->setChecked(false);
    _ui.actionVisionFull_Field->setChecked(false);
}

void MainWindow::on_actionVisionSecondary_Half_triggered() {
    _processor->changeVisionChannel(SharedVisionPortSingleSecondary);
    _processor->setFieldDimensions(Field_Dimensions::Single_Field_Dimensions);
    _ui.actionVisionPrimary_Half->setChecked(false);
    _ui.actionVisionSecondary_Half->setChecked(true);
    _ui.actionVisionFull_Field->setChecked(false);
}

void MainWindow::on_actionVisionFull_Field_triggered() {
    _processor->changeVisionChannel(SharedVisionPortDoubleOld);
    _processor->setFieldDimensions(Field_Dimensions::Double_Field_Dimensions);
    _ui.actionVisionPrimary_Half->setChecked(false);
    _ui.actionVisionSecondary_Half->setChecked(false);
    _ui.actionVisionFull_Field->setChecked(true);
}
