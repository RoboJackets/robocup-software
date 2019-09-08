#include "MainWindow.hpp"
#include <Network.hpp>
#include <Robot.hpp>
#include <Utils.hpp>
#include <gameplay/GameplayModule.hpp>
#include <joystick/GamepadController.hpp>
#include <joystick/Joystick.hpp>
#include <ui/StyleSheetManager.hpp>
#include "BatteryProfile.hpp"
#include "Configuration.hpp"
#include "RobotStatusWidget.hpp"
#include "rc-fshare/git_version.hpp"
#include "radio/Radio.hpp"

#include <QActionGroup>
#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QString>

#include <ctime>
#include <iostream>
#include <string>

#include <google/protobuf/descriptor.h>
#include <protobuf/grSim_Commands.pb.h>
#include <protobuf/grSim_Packet.pb.h>
#include <protobuf/grSim_Replacement.pb.h>
#include <ui_MainWindow.h>

using namespace std;
using namespace boost;
using namespace google::protobuf;
using namespace Packet;
using namespace Eigen;

static const std::vector<QString> defaultHiddenLayers{
    "MotionControl", "Global Obstacles", "Local Obstacles",
    "Planning0",     "Planning1",        "Planning2",
    "Planning3",     "Planning4",        "Planning5"};

void calcMinimumWidth(QWidget* widget, QString text) {
    QRect rect = QFontMetrics(widget->font()).boundingRect(text);
    widget->setMinimumWidth(rect.width());
}

MainWindow::MainWindow(Processor* processor, QWidget* parent)
    : QMainWindow(parent),
      _updateCount(0),
      _autoExternalReferee(true),
      _doubleFrameNumber(-1),
      _lastUpdateTime(RJ::now()),
      _history(2 * 60),
      _longHistory(10000),
      _processor(processor) {
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
    goalieModel =
        qobject_cast<const QStandardItemModel*>(_ui.goalieID->model());

    // Append short Git hash to the main window title with an asterisk if the
    // current Git index is dirty
    setWindowTitle(windowTitle() + " @ " + git_version_short_hash +
                   (git_version_dirty ? "*" : ""));

    if (!_processor->simulation()) {
        _ui.menu_Simulator->setEnabled(false);
    } else {
        // reset the field initially, grSim will start out in some weird
        // pattern and we want to keep it consistent
        on_actionResetField_triggered();
    }

    // disabled because of lack of grSim support
    _ui.actionQuickloadRobotLocations->setEnabled(false);
    _ui.actionQuicksaveRobotLocations->setEnabled(false);
    //_ui.actionResetField->setEnabled(false);
    _ui.actionStopRobots->setEnabled(false);
}

void MainWindow::configuration(Configuration* config) {
    _config = config;
    _config->tree(_ui.configTree);
}

void MainWindow::initialize() {
    // Team
    if (_processor->blueTeam()) {
        _ui.actionTeamBlue->trigger();
    } else {
        _ui.actionTeamYellow->trigger();
    }

    if (_processor->logger().recording()) {
        _ui.actionStart_Logging->setText(QString("Already Logging to: ") +
                                         _processor->logger().filename());
        _ui.actionStart_Logging->setEnabled(false);
    }

    // Initialize to ui defaults
    on_goalieID_currentIndexChanged(_ui.goalieID->currentIndex());

    qActionGroups["teamGroup"]->checkedAction()->trigger();
    qActionGroups["rotateGroup"]->checkedAction()->trigger();
    qActionGroups["radioGroup"]->checkedAction()->trigger();

    // Default to FullField on Simulator
    if (_processor->simulation()) {
        _ui.actionVisionFull_Field->trigger();
    }

    updateTimer.setSingleShot(true);
    connect(&updateTimer, SIGNAL(timeout()), SLOT(updateViews()));
    updateTimer.start(30);

    _autoExternalReferee = _processor->externalReferee();

    if (_processor->defendPlusX()) {
        on_actionDefendPlusX_triggered();
        _ui.actionDefendPlusX->setChecked(true);
    } else {
        on_actionDefendMinusX_triggered();
        _ui.actionDefendMinusX->setChecked(true);
    }

    switch (_processor->visionChannel()) {
        case 0:
            on_actionVisionPrimary_Half_triggered();
            _ui.actionVisionPrimary_Half->setChecked(true);
            break;
        case 1:
            on_actionVisionSecondary_Half_triggered();
            _ui.actionVisionSecondary_Half->setChecked(true);
            break;
        case 2:
            on_actionVisionFull_Field_triggered();
            _ui.actionVisionFull_Field->setChecked(true);
            break;
    }
}

void MainWindow::logFileChanged() {
    if (_processor->logger().recording()) {
        _logFile->setText(_processor->logger().filename());
        _ui.actionStart_Logging->setText(QString("Already Logging to: ") +
                                         _processor->logger().filename());
        _ui.actionStart_Logging->setEnabled(false);
    } else {
        _logFile->setText("Not Recording");
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

string MainWindow::formatLabelBold(Side side, string label) {
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

void MainWindow::updateFromRefPacket(bool haveExternalReferee) {
    // update goalie from Packet
    if (haveExternalReferee) {
        // The External Ref is connected
        _ui.goalieID->setEnabled(false);
        // disable Blue/Yellow team
        qActionGroups["teamGroup"]->setEnabled(false);

        // Changes the goalie INDEX which is 1 higher than the goalie ID
        if (_ui.goalieID->currentIndex() !=
            _processor->context()->game_state.getGoalieId() + 1) {
            _ui.goalieID->setCurrentIndex(
                _processor->context()->game_state.getGoalieId() + 1);
        }

        bool blueTeam = _processor->refereeModule()->blueTeam();
        if (_processor->blueTeam() != blueTeam) {
            blueTeam ? _ui.actionTeamBlue->trigger()
                     : _ui.actionTeamYellow->trigger();
        }
    } else {
        _ui.goalieID->setEnabled(true);
        qActionGroups["teamGroup"]->setEnabled(true);
    }
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

    // Time since last update
    RJ::Time now = RJ::now();
    auto delta_time = now - _lastUpdateTime;
    _lastUpdateTime = now;
    double framerate = RJ::Seconds(1) / delta_time;

    ++_updateCount;
    if (_updateCount == 4) {
        _updateCount = 0;

        _viewFPS->setText(QString("View: %1 fps").arg(framerate, 0, 'f', 1));
        _procFPS->setText(
            QString("Proc: %1 fps").arg(_processor->framerate(), 0, 'f', 1));

        _logMemory->setText(
            QString("Log: %1/%2 %3 kiB")
                .arg(QString::number(_processor->logger().size()),
                     QString::number(_processor->logger().capacity()),
                     QString::number((_processor->logger().spaceUsed() + 512) /
                                     1024)));
    }

    auto value = _ui.logHistoryLocation->value();

    // Advance log history
    int liveFrameNumber = _processor->logger().currentFrameNumber();
    if (live()) {
        _doubleFrameNumber = liveFrameNumber;
    } else {
        _doubleFrameNumber += *_playbackRate;

        int minFrame = _processor->logger().firstFrameNumber() + 10;
        int maxFrame = _processor->logger().currentFrameNumber();

        if (_doubleFrameNumber < minFrame) {
            _doubleFrameNumber = minFrame;
            setPlayBackRate(1.0);
        } else if (_doubleFrameNumber > maxFrame) {
            _doubleFrameNumber = maxFrame;
            setLive();
        }
    }

    _ui.logHistoryLocation->setMinimum(_processor->logger().firstFrameNumber());
    _ui.logHistoryLocation->setMaximum(
        _processor->logger().currentFrameNumber());
    _ui.logHistoryLocation->setTickInterval(60 * 60);  // interval is ~ 1 minute
    _ui.logHistoryLocation->setValue(_doubleFrameNumber);

    // update history slider in ui

    // Read recent history from the log
    _processor->logger().getFrames(frameNumber(), _longHistory.size(),
                                   _longHistory.begin());

    // Set the original history vector
    _history.assign(_longHistory.begin(),
                    _longHistory.begin() + _history.size());

    // Update field view
    _ui.fieldView->update();

    // enable playback buttons based on playback rate
    for (QPushButton* playbackBtn : _logPlaybackButtons)
        playbackBtn->setEnabled(true);
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
        auto gametime =
            (RJ::Time(chrono::microseconds(currentFrame->timestamp())) -
             _processor->logger().startTime());
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

        auto frameNum = _processor->logger().currentFrameNumber();

        _ui.frameNumLabel->setText(QString("%1/%2")
                                       .arg(QString::number(frameNumber()))
                                       .arg(QString::number(frameNum)));

        // Update non-message tree items
        _frameNumberItem->setData(ProtobufTree::Column_Value, Qt::DisplayRole,
                                  frameNumber());
        int elapsedMillis = (currentFrame->command_time() -
                             RJ::timestamp(*_processor->firstLogTime)) /
                            1000;

        QTime elapsedTime = QTime::fromMSecsSinceStartOfDay(elapsedMillis);
        _elapsedTimeItem->setText(ProtobufTree::Column_Value,
                                  elapsedTime.toString("hh:mm:ss.zzz"));

        // Sort the tree by tag if items have been added
        if (_ui.logTree->message(*currentFrame)) {
            // Items have been added, so sort again on tag number
            _ui.logTree->sortItems(ProtobufTree::Column_Tag,
                                   Qt::AscendingOrder);
        }

        // update the behavior tree view
        QString behaviorStr =
            QString::fromStdString(currentFrame->behavior_tree());
        if (_ui.behaviorTree->toPlainText() != behaviorStr) {
            _ui.behaviorTree->setPlainText(behaviorStr);
        }
    }

    _ui.refStage->setText(NewRefereeModuleEnums::stringFromStage(
                              _processor->refereeModule()->stage).c_str());
    _ui.refCommand->setText(NewRefereeModuleEnums::stringFromCommand(
                                _processor->refereeModule()->command).c_str());

    // convert time left from ms to s and display it to two decimal places
    int timeSeconds =
        _processor->refereeModule()->stage_time_left.count() / 1000;
    int timeMinutes = timeSeconds / 60;
    timeSeconds = timeSeconds % 60;
    _ui.refTimeLeft->setText(tr("%1:%2").arg(
        QString::number(timeMinutes), QString::number(std::abs(timeSeconds))));

    const char* blueName = _processor->refereeModule()->blue_info.name.c_str();
    string blueFormatted = strlen(blueName) == 0 ? "Blue Team" : blueName;
    blueFormatted = formatLabelBold(Side::Blue, blueFormatted);
    _ui.refBlueName->setText(QString::fromStdString(blueFormatted));
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
    string yellowFormatted =
        strlen(yellowName) == 0 ? "Yellow Team" : yellowName;
    yellowFormatted = formatLabelBold(Side::Yellow, yellowFormatted);
    _ui.refYellowName->setText(QString::fromStdString(yellowFormatted));
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
            statusWidget->setBlueTeam(_processor->blueTeam());

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

// uncomment this #define to test the display of a variety of
// different errors #define DEMO_ROBOT_STATUS

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
            QString error = "Kicker Fault, Motor Fault FR, Ball Sense Fault";
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
            RadioRx rx = robot->radioRx();

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
                const char* motorNames[] = {"FL", "BL", "BR", "FR", "Dribbler"};

                // examine status of each motor (including the dribbler)
                for (int i = 0; i < 5; ++i) {
                    bool motorIFault = true;
                    switch (rx.motor_status(i)) {
                        case Packet::Hall_Failure:
                            errorList
                                << QString("Motor Fault %1").arg(motorNames[i]);
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
            bool kickerFault = rx.has_kicker_status() &&
                               !(rx.kicker_status() & Kicker_Enabled);

            bool kicker_charging =
                rx.has_kicker_status() && rx.kicker_status() & 0x01;
            statusWidget->setKickerState(kicker_charging);
            bool ballSenseFault = rx.has_ball_sense_status() &&
                                  !(rx.ball_sense_status() == Packet::NoBall ||
                                    rx.ball_sense_status() == Packet::HasBall);
            if (kickerFault) errorList << "Kicker Fault";
            if (ballSenseFault) errorList << "Ball Sense Fault";
            statusWidget->setBallSenseFault(ballSenseFault);

            // check fpga status
            bool fpgaWorking = true;
            if (rx.has_fpga_status() && rx.fpga_status() != Packet::FpgaGood) {
                if (rx.fpga_status() == Packet::FpgaNotInitialized) {
                    errorList << "FPGA not initialized";
                } else {
                    errorList << "FPGA error";
                }
                fpgaWorking = false;
            }

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
                               (batteryLevel < 0.25) || !fpgaWorking;
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

    if (_processor->gameplayModule()->checkPlaybookStatus()) {
        playIndicatorStatus(false);
    }

    // Some conditions are different in simulation
    bool sim = _processor->simulation();

    if (!sim) {
        updateRadioBaseStatus(_processor->isRadioOpen());
    }

    // Get processing thread status
    Processor::Status ps = _processor->status();
    RJ::Time curTime = RJ::now();

    // Determine if we are receiving packets from an external referee
    bool haveExternalReferee = (curTime - ps.lastRefereeTime) < RJ::Seconds(1);

    std::vector<int> validIds = _processor->state()->ourValidIds();

    for (int i = 1; i <= Num_Shells; i++) {
        QStandardItem* item = goalieModel->item(i);
        if (std::find(validIds.begin(), validIds.end(), i - 1) !=
            validIds.end()) {
            // The list starts with None so i is 1 higher than the shell id
            item->setFlags(item->flags() |
                           (Qt::ItemIsSelectable | Qt::ItemIsEnabled));
        } else {
            item->setFlags(item->flags() &
                           ~(Qt::ItemIsSelectable | Qt::ItemIsEnabled));
        }
    }

    if (haveExternalReferee && _autoExternalReferee) {
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

    updateFromRefPacket(haveExternalReferee);

    // Is the processing thread running?
    if (curTime - ps.lastLoopTime > RJ::Seconds(0.1)) {
        // Processing loop hasn't run recently.
        // Likely causes:
        //    Mutex deadlock (need a recursive mutex?)
        //    Excessive computation
        status("PROCESSING HUNG", Status_Fail);
        return;
    }

    // Check network activity
    if (curTime - ps.lastVisionTime > RJ::Seconds(0.1)) {
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
    if (curTime - ps.lastRadioRxTime > RJ::Seconds(1)) {
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
            return;
        }
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

void MainWindow::playIndicatorStatus(bool color) {
    if (color) {
        _ui.playIndicatorStatus->setStyleSheet("background-color: #00ff00");
    } else {
        _ui.playIndicatorStatus->setStyleSheet("background-color: #ff0000");
    }
}

void MainWindow::updateRadioBaseStatus(bool usbRadio) {
    QString label =
        QString(usbRadio ? "Radio Connected" : "Radio Disconnected");
    if (_ui.radioBaseStatus->text() != label) {
        _ui.radioBaseStatus->setText(label);
        _ui.radioBaseStatus->setStyleSheet(usbRadio
                                               ? "background-color: #00ff00"
                                               : "background-color: #ff4040");
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

void MainWindow::on_action916MHz_triggered() { channel(0); }

void MainWindow::on_action918MHz_triggered() { channel(1); }

void MainWindow::channel(int n) {
    if (_processor && _processor->radio()) {
        _processor->radio()->channel(n);
    }
    _ui.radioLabel->setText(QString("%1MHz").arg(916.0 + 0.2 * n, 0, 'f', 1));
}

// Simulator commands

void MainWindow::on_actionCenterBall_triggered() {
    grSim_Packet simPacket;
    grSim_BallReplacement* ball_replace =
        simPacket.mutable_replacement()->mutable_ball();

    ball_replace->mutable_pos()->set_x(0);
    ball_replace->mutable_pos()->set_y(0);
    ball_replace->mutable_vel()->set_x(0);
    ball_replace->mutable_vel()->set_y(0);

    _ui.fieldView->sendSimCommand(simPacket);
}

void MainWindow::on_actionStopBall_triggered() {
    grSim_Packet simPacket;
    grSim_BallReplacement* ball_replace =
        simPacket.mutable_replacement()->mutable_ball();

    Geometry2d::Point ballPos =
        _ui.fieldView->getTeamToWorld() * state()->ball.pos;
    ball_replace->mutable_pos()->set_x(ballPos.x());
    ball_replace->mutable_pos()->set_y(ballPos.y());
    ball_replace->mutable_vel()->set_x(0);
    ball_replace->mutable_vel()->set_y(0);
    _ui.fieldView->sendSimCommand(simPacket);
}

void MainWindow::on_actionResetField_triggered() {
    grSim_Packet simPacket;

    grSim_Replacement* replacement = simPacket.mutable_replacement();
    for (int i = 0; i < Robots_Per_Team; ++i) {
        auto rob = replacement->add_robots();

        const int NUM_COLS = 2;
        const int ROBOTS_PER_COL = Robots_Per_Team / NUM_COLS;

        double x_pos = -2.5 + i / ROBOTS_PER_COL;
        double y_pos = i % ROBOTS_PER_COL - ROBOTS_PER_COL / NUM_COLS;

        rob->set_x(x_pos);
        rob->set_y(y_pos);
        rob->set_dir(0);
        rob->set_id(i);
        rob->set_yellowteam(false);
    }

    for (int i = 0; i < Robots_Per_Team; ++i) {
        auto rob = replacement->add_robots();

        const int NUM_COLS = 2;
        const int ROBOTS_PER_COL = Robots_Per_Team / NUM_COLS;

        double x_pos = +2.5 - i / ROBOTS_PER_COL;
        double y_pos = i % ROBOTS_PER_COL - ROBOTS_PER_COL / NUM_COLS;

        rob->set_x(x_pos);
        rob->set_y(y_pos);
        rob->set_dir(180);
        rob->set_id(i);
        rob->set_yellowteam(true);
    }

    auto ball_replace = replacement->mutable_ball();
    ball_replace->mutable_pos()->set_x(0.0);
    ball_replace->mutable_pos()->set_y(0.0);
    ball_replace->mutable_vel()->set_x(0.0);
    ball_replace->mutable_vel()->set_y(0.0);

    _ui.fieldView->sendSimCommand(simPacket);
}

void MainWindow::on_actionStopRobots_triggered() {
    // TODO: check that this handles threads properly
    /*
    for (OurRobot* robot : state()->self) {
        if (robot->visible) {
            SimCommand::Robot* r = cmd.add_robots();
            r->set_shell(robot->shell());
            r->set_blue_team(_processor->blueTeam());
            Geometry2d::Point newPos =
                _ui.fieldView->getTeamToWorld() * robot->pos;
            r->mutable_pos()->set_x(newPos.x());
            r->mutable_pos()->set_y(newPos.y());
            r->mutable_vel()->set_x(0);
            r->mutable_vel()->set_y(0);
            r->set_w(0);
        }
    }
    for (OpponentRobot* robot : state()->opp) {
        if (robot->visible) {
            SimCommand::Robot* r = cmd.add_robots();
            r->set_shell(robot->shell());
            r->set_blue_team(!_processor->blueTeam());
            Geometry2d::Point newPos =
                _ui.fieldView->getTeamToWorld() * robot->pos;
            r->mutable_pos()->set_x(newPos.x());
            r->mutable_pos()->set_y(newPos.y());
            r->mutable_vel()->set_x(0);
            r->mutable_vel()->set_y(0);
            r->set_w(0);
        }
    }
    */
    //_ui.fieldView->sendSimCommand(cmd);
}

void MainWindow::on_actionQuicksaveRobotLocations_triggered() {
    /*
    _ui.actionQuickloadRobotLocations->setEnabled(true);
    _quickLoadCmd.reset();
    for (OurRobot* robot : state()->self) {
        if (robot->visible) {
            SimCommand::Robot* r = _quickLoadCmd.add_robots();
            r->set_shell(robot->shell());
            r->set_blue_team(_processor->blueTeam());
            Geometry2d::Point newPos =
                _ui.fieldView->getTeamToWorld() * robot->pos;
            r->mutable_pos()->set_x(newPos.x());
            r->mutable_pos()->set_y(newPos.y());
            r->mutable_vel()->set_x(0);
            r->mutable_vel()->set_y(0);
            r->set_w(0);
        }
    }
    for (OpponentRobot* robot : state()->opp) {
        if (robot->visible) {
            SimCommand::Robot* r = _quickLoadCmd.add_robots();
            r->set_shell(robot->shell());
            r->set_blue_team(!_processor->blueTeam());
            Geometry2d::Point newPos =
                _ui.fieldView->getTeamToWorld() * robot->pos;
            r->mutable_pos()->set_x(newPos.x());
            r->mutable_pos()->set_y(newPos.y());
            r->mutable_vel()->set_x(0);
            r->mutable_vel()->set_y(0);
            r->set_w(0);
        }
    }

    Geometry2d::Point ballPos =
        _ui.fieldView->getTeamToWorld() * state()->ball.pos;
    _quickLoadCmd.mutable_ball_pos()->set_x(ballPos.x());
    _quickLoadCmd.mutable_ball_pos()->set_y(ballPos.y());
    _quickLoadCmd.mutable_ball_vel()->set_x(0);
    _quickLoadCmd.mutable_ball_vel()->set_y(0);
    */
}

void MainWindow::on_actionQuickloadRobotLocations_triggered() {
    //_ui.fieldView->sendSimCommand(_quickLoadCmd);
}

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

void MainWindow::on_actionRestartUpdateTimer_triggered() {
    printf("Update timer: active %d, singleShot %d, interval %d\n",
           updateTimer.isActive(), updateTimer.isSingleShot(),
           updateTimer.interval());
    updateTimer.stop();
    updateTimer.start(30);
}

void MainWindow::on_actionStart_Logging_triggered() {
    if (!_processor->logger().recording()) {
        if (!QDir("logs").exists()) {
            QDir().mkdir("logs");
        }

        QString logFile =
            QString("logs/") +
            QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss.log");

        if (!_processor->openLog(logFile)) {
            printf("Failed to open %s: %m\n", (const char*)logFile.toLatin1());
        } else {
            _ui.actionStart_Logging->setText(QString("Now Logging to:") +
                                             logFile);
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
    _processor->joystickKickOnBreakBeam(
        _ui.joystickKickOnBreakBeam->checkState());
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
    _processor->blueTeam(true);
}

void MainWindow::on_actionTeamYellow_triggered() {
    _ui.team->setText("YELLOW");
    _ui.team->setStyleSheet("background-color: #ffff00");
    _processor->blueTeam(false);
}

void MainWindow::on_manualID_currentIndexChanged(int value) {
    _processor->manualID(value - 1);
}

void MainWindow::on_actionUse_Field_Oriented_Controls_toggled(bool value) {
    _processor->setUseFieldOrientedManualDrive(value);
}

void MainWindow::on_actionUse_Multiple_Joysticks_toggled(bool value) {
    _processor->multipleManual(value);
    _processor->setupJoysticks();
}

void MainWindow::on_goalieID_currentIndexChanged(int value) {
    _processor->goalieID(value - 1);
}

void MainWindow::on_actionUse_External_Referee_toggled(bool value) {
    _autoExternalReferee = value;
    _processor->externalReferee(value);
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
        this, "Load Playbook",
        ApplicationRunDirectory().filePath("../soccer/gameplay/playbooks/"));
    if (!filename.isNull()) {
        try {
            _processor->gameplayModule()->loadPlaybook(filename.toStdString(),
                                                       true);
            playIndicatorStatus(true);
        } catch (runtime_error* error) {
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
        } catch (runtime_error* error) {
            QMessageBox::critical(this, "File not found",
                                  QString("File not found: %1").arg(filename));
        }
    }
}

void MainWindow::on_clearPlays_clicked() {
    _processor->gameplayModule()->clearPlays();
    playIndicatorStatus(true);
}

void MainWindow::setRadioChannel(RadioChannels channel) {
    switch (channel) {
        case RadioChannels::MHz_916:
            this->on_action916MHz_triggered();
            break;
        case RadioChannels::MHz_918:
            this->on_action918MHz_triggered();
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

void MainWindow::on_fastDirectBlue_clicked() {
    _processor->refereeModule()->command =
        NewRefereeModuleEnums::DIRECT_FREE_BLUE;
}

void MainWindow::on_fastDirectYellow_clicked() {
    _processor->refereeModule()->command =
        NewRefereeModuleEnums::DIRECT_FREE_YELLOW;
}

void MainWindow::on_fastIndirectBlue_clicked() {
    _processor->refereeModule()->command =
        NewRefereeModuleEnums::INDIRECT_FREE_BLUE;
}

void MainWindow::on_fastIndirectYellow_clicked() {
    _processor->refereeModule()->command =
        NewRefereeModuleEnums::INDIRECT_FREE_YELLOW;
}

void MainWindow::on_actionVisionPrimary_Half_triggered() {
    _processor->changeVisionChannel(SharedVisionPortSinglePrimary);
    _processor->setFieldDimensions(Field_Dimensions::Single_Field_Dimensions);
}

void MainWindow::on_actionVisionSecondary_Half_triggered() {
    _processor->changeVisionChannel(SharedVisionPortSingleSecondary);
    _processor->setFieldDimensions(Field_Dimensions::Single_Field_Dimensions);
}

void MainWindow::on_actionVisionFull_Field_triggered() {
    _processor->changeVisionChannel(SharedVisionPortDoubleNew);
    _processor->setFieldDimensions(Field_Dimensions::Double_Field_Dimensions);
}

bool MainWindow::live() { return !_playbackRate; }
