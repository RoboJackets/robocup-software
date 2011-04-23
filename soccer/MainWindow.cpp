// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include "MainWindow.hpp"

#include "PlayConfigTab.hpp"
#include "TestResultTab.hpp"
#include "RefereeModule.hpp"
#include "Configuration.hpp"
#include <Utils.hpp>
#include <gameplay/GameplayModule.hpp>
#include <framework/RobotConfig.hpp>

#include <QInputDialog>
#include <QFileDialog>
#include <QActionGroup>
#include <QMessageBox>

#include <boost/foreach.hpp>

#include <google/protobuf/descriptor.h>

using namespace std;
using namespace boost;
using namespace google::protobuf;
using namespace Packet;

// Style sheets used for live/non-live controls
QString LiveStyle("border:2px solid transparent");
QString NonLiveStyle("border:2px solid red");

void calcMinimumWidth(QWidget *widget, QString text)
{
	QRect rect = QFontMetrics(widget->font()).boundingRect(text);
	widget->setMinimumWidth(rect.width());
}

MainWindow::MainWindow(QWidget *parent):
	QMainWindow(parent)
{
	_haveRadioChannel = false;
	_updateCount = 0;
	_processor = 0;
	_autoExternalReferee = true;
	_doubleFrameNumber = -1;
	_robotConfig2008 = 0;
	_robotConfig2011 = 0;
	
	_lastUpdateTime = Utils::timestamp();
	_history.resize(2 * 60);
	
	_ui.setupUi(this);
	_ui.fieldView->history(&_history);
	
	_ui.logTree->history(&_history);
	_ui.logTree->mainWindow = this;
	_ui.logTree->updateTimer = &updateTimer;
	
	// Initialize live/non-live control styles
	_live = false;
	live(true);
	
	_refereeLabel = new QLabel();
	_refereeLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
	_refereeLabel->setToolTip("Last Referee Packet");
	_refereeLabel->setAlignment(Qt::AlignCenter);
	calcMinimumWidth(_refereeLabel, "XXXXXXXXXXXXXXXX");
	statusBar()->addPermanentWidget(_refereeLabel);
	
	_currentPlay = new QLabel();
	_currentPlay->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
	_currentPlay->setToolTip("Current Play");
	_currentPlay->setAlignment(Qt::AlignCenter);
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
	
	// Connect shortcut buttons to regular referee buttons
	connect(_ui.fastHalt, SIGNAL(clicked()), _ui.refHalt, SLOT(click()));
	connect(_ui.fastStop, SIGNAL(clicked()), _ui.refStop, SLOT(click()));
	connect(_ui.fastReady, SIGNAL(clicked()), _ui.refReady, SLOT(click()));
	connect(_ui.fastForceStart, SIGNAL(clicked()), _ui.refForceStart, SLOT(click()));
	connect(_ui.fastKickoffBlue, SIGNAL(clicked()), _ui.refKickoffBlue, SLOT(click()));
	connect(_ui.fastKickoffYellow, SIGNAL(clicked()), _ui.refKickoffYellow, SLOT(click()));
	
	QActionGroup *teamGroup = new QActionGroup(this);
	teamGroup->addAction(_ui.actionTeamBlue);
	teamGroup->addAction(_ui.actionTeamYellow);
	
	QActionGroup *goalGroup = new QActionGroup(this);
	goalGroup->addAction(_ui.actionDefendMinusX);
	goalGroup->addAction(_ui.actionDefendPlusX);
	
	QActionGroup *rotateGroup = new QActionGroup(this);
	rotateGroup->addAction(_ui.action0);
	rotateGroup->addAction(_ui.action90);
	rotateGroup->addAction(_ui.action180);
	rotateGroup->addAction(_ui.action270);
	
	_ui.splitter->setStretchFactor(0, 98);
	_ui.splitter->setStretchFactor(1, 10);

	updateTimer.setSingleShot(true);
	connect(&updateTimer, SIGNAL(timeout()), SLOT(updateViews()));
	updateTimer.start(30);
}

void MainWindow::configuration(Configuration* config)
{
	_config = config;
	_config->tree(_ui.configTree);

	// Revision-specific configuration
	_robotConfig2008 = new RobotConfig(config, "Rev2008");
	_robotConfig2008->motion.output_coeffs.resize(4);
	_robotConfig2008->motion.output_coeffs.set(0, 10);
	
	_robotConfig2011 = new RobotConfig(config, "Rev2011");
	_robotConfig2011->motion.output_coeffs.resize(4);
	_robotConfig2011->motion.output_coeffs.set(0, 10);
	
	for (size_t i = 0; i < Num_Shells; ++i)
	{
		_robot2011[i] = new ConfigBool(config, QString("Is2011/%1").arg(i));
	}
	
	updateRobotConfigs();
}

void MainWindow::processor(Processor* value)
{
	// This should only happen once
	assert(!_processor);
	
	_processor = value;
	
	// External referee
	on_externalReferee_toggled(_ui.externalReferee->isChecked());
	
	// Team
	if (_processor->blueTeam())
	{
		_ui.actionTeamBlue->trigger();
	} else {
		_ui.actionTeamYellow->trigger();
	}
	
	// Add Plays tab
	_playConfigTab = new PlayConfigTab();
	_ui.tabWidget->addTab(_playConfigTab, tr("Plays"));
	_playConfigTab->setup(_processor->gameplayModule());

	// Add Test Results tab
	_testResultTab = new TestResultTab();
	_ui.tabWidget->addTab(_testResultTab, tr("Test Results"));
//	_testResultTab->setup(_processor->gameplayModule()); // FIXME: this should actually exist
	
	updateRobotConfigs();
}

void MainWindow::updateRobotConfigs()
{
	if (_processor)
	{
		QMutexLocker lock(&_processor->loopMutex());
		BOOST_FOREACH(OurRobot *robot, state()->self)
		{
			robot->config = (*_robot2011[robot->shell()]) ? _robotConfig2011 : _robotConfig2008;
		}
	}
}

void MainWindow::logFileChanged()
{
	if (_processor->logger().recording())
	{
		_logFile->setText(_processor->logger().filename());
	} else {
		_logFile->setText("Not Recording");
	}
}

void MainWindow::live(bool value)
{
	if (_live != value)
	{
		_live = value;
		
		// Change styles for controls that can show historical data
		_ui.fieldView->live = _live;
		if (_live)
		{
			_ui.logTree->setStyleSheet(QString("QTreeWidget{%1}").arg(LiveStyle));
		} else {
			_ui.logTree->setStyleSheet(QString("QTreeWidget{%1}").arg(NonLiveStyle));
		}
	}
}

void MainWindow::updateViews()
{
	_refereeLabel->setText(_processor->refereeModule()->lastPacketDescription());
	
	// Radio channel
	if (!_haveRadioChannel)
	{
		//FIXME - Not like this any more
#if 0
		int r = _processor->radio();
		if (r >= 0)
		{
			_haveRadioChannel = true;
			_ui.radioLabel->setText(QString("Radio %1").arg(r));
		}
#endif
	}

	int manual =_processor->manualID();
	if ((manual >= 0 || _ui.manualID->isEnabled()) && !_processor->joystickValid())
	{
		// Joystick is gone - turn off manual control
		_ui.manualID->setCurrentIndex(0);
		_processor->manualID(-1);
		_ui.manualID->setEnabled(false);
	} else if (!_ui.manualID->isEnabled() && _processor->joystickValid())
	{
		// Joystick reconnected
		_ui.manualID->setEnabled(true);
	}
	
	// Time since last update
	uint64_t time = Utils::timestamp();
	int delta_us = time - _lastUpdateTime;
	_lastUpdateTime = time;
	double framerate = 1000000.0 / delta_us;
	
	// Status bar
	QString play = _processor->gameplayModule()->playName();
	if (play.isNull())
	{
		play = "(no play)";
	}
	_currentPlay->setText(play);
	
	++_updateCount;
	if (_updateCount == 4)
	{
		_updateCount = 0;
		
		_viewFPS->setText(QString("View: %1 fps").arg(framerate, 0, 'f', 1));
		_procFPS->setText(QString("Proc: %1 fps").arg(_processor->framerate(), 0, 'f', 1));
		
		_logMemory->setText(QString("Log: %1/%2 %3 kiB").arg(
			QString::number(_processor->logger().numFrames()),
			QString::number(_processor->logger().maxFrames()),
			QString::number((_processor->logger().spaceUsed() + 512) / 1024)
		));
	}
	
	// Advance log playback time
	int liveFrameNumber = _processor->logger().lastFrameNumber();
	if (_live)
	{
		_doubleFrameNumber = liveFrameNumber;
	} else {
		double rate = _ui.playbackRate->value();
		_doubleFrameNumber += rate / framerate;
		
		int minFrame = _processor->logger().firstFrameNumber();
		int maxFrame = _processor->logger().lastFrameNumber();
		if (_doubleFrameNumber < minFrame)
		{
			_doubleFrameNumber = minFrame;
		} else if (_doubleFrameNumber > maxFrame)
		{
			_doubleFrameNumber = maxFrame;
		}
	}
	
	// Read recent history from the log
	_processor->logger().getFrames(frameNumber(), _history);
	
	// Update field view
	_ui.fieldView->update();
	
	// Update log controls
	_ui.logLive->setEnabled(!_live);
	_ui.logStop->setEnabled(_live);
	
	// Update status indicator
	updateStatus();
	
	// Update play list
	_playConfigTab->frameUpdate();
	
	// Check if any debug layers have been added
	// (layers should never be removed)
	const shared_ptr<LogFrame> liveFrame = _processor->logger().lastFrame();
	if (liveFrame && liveFrame->debug_layers_size() > _ui.debugLayers->count())
	{
		// Add the missing layers and turn them on
		for (int i = _ui.debugLayers->count(); i < liveFrame->debug_layers_size(); ++i)
		{
			QListWidgetItem *item = new QListWidgetItem(QString::fromStdString(liveFrame->debug_layers(i)));
			item->setCheckState(_ui.fieldView->layerVisible(i) ? Qt::Checked : Qt::Unchecked);
			item->setData(Qt::UserRole, i);
			_ui.debugLayers->addItem(item);
		}
		
		_ui.debugLayers->sortItems();
	}
	
	// Get the frame at the log playback time
	const shared_ptr<LogFrame> currentFrame = _history[0];
	
	if (currentFrame)
	{
		// Update non-message tree items
		_frameNumberItem->setData(ProtobufTree::Column_Value, Qt::DisplayRole, frameNumber());
		int elapsedMillis = (currentFrame->start_time() - _processor->firstLogTime + 500) / 1000;
		QTime elapsedTime = QTime().addMSecs(elapsedMillis);
		_elapsedTimeItem->setText(ProtobufTree::Column_Value, elapsedTime.toString("hh:mm:ss.zzz"));
		
		// Sort the tree by tag if items have been added
		if (_ui.logTree->message(*currentFrame))
		{
			// Items have been added, so sort again on tag number
			_ui.logTree->sortItems(ProtobufTree::Column_Tag, Qt::AscendingOrder);
		}
	}
	
	// We restart this timer repeatedly instead of using a single shot timer in order
	// to guarantee a minimum time between redraws.  This will limit the CPU usage on a fast computer.
	updateTimer.start(20);
}

void MainWindow::updateStatus()
{
	// Guidelines:
	//    Status_Fail is used for severe, usually external, errors such as hardware or network failures.
	//    Status_Warning is used for configuration problems that prevent competition operation.
	//        These can be easily changed within the soccer program.
	//    Status_OK shall only be used for "COMPETITION".
	//
	// The order of these checks is important to help debugging.
	// More specific or unlikely problems should be tested earlier.
	
	if (!_processor)
	{
		status("NO PROCESSOR", Status_Fail);
		return;
	}
	
	// Some conditions are different in simulation
	bool sim = _processor->simulation();
	
	// Get processing thread status
	Processor::Status ps = _processor->status();
	uint64_t curTime = Utils::timestamp();
	
	// Determine if we are receiving packets from an external referee
	bool haveExternalReferee = (curTime - ps.lastRefereeTime) < 500 * 1000;
	
	if (_autoExternalReferee && haveExternalReferee && !_ui.externalReferee->isChecked())
	{
		_ui.externalReferee->setChecked(true);
	}
	
	// Is the processing thread running?
	if (curTime - ps.lastLoopTime > 100 * 1000)
	{
		// Processing loop hasn't run recently.
		// Likely causes:
		//    Mutex deadlock (need a recursive mutex?)
		//    Excessive computation
		status("PROCESSING HUNG", Status_Fail);
		return;
	}

	// Check network activity
	if (curTime - ps.lastVisionTime > 100 * 1000)
	{
		// We must always have vision
		status("NO VISION", Status_Fail);
		return;
	}
	
	// Manual checks happen as soon as we can be expected to have vision.
	// Vision is not required for manual driving, but I think it's more likely
	// that you will stop on purpose when vision doesn't work than you will
	// want to switch between autonomous and manual without vision.
	if (!_processor->autonomous())
	{
		// No autonomous robots
		status("STOPPED", Status_Warning);
		return;
	}
	
	if (_processor->manualID() >= 0)
	{
		// Mixed auto/manual control
		status("MANUAL", Status_Warning);
		return;
	}

	// Driving the robots helps isolate radio problems by verifying radio TX,
	// so test this after manual driving.
	if (curTime - ps.lastRadioRxTime > 1000 * 1000)
	{
		// Allow a long timeout in case of poor radio performance
		status("NO RADIO RX", Status_Fail);
		return;
	}
	
	if ((!sim || _processor->externalReferee()) && !haveExternalReferee)
	{
		if (_autoExternalReferee && _processor->externalReferee())
		{
			// Automatically turn off external referee
			_ui.externalReferee->setChecked(false);
		} else {
			// In simulation, we will often run without a referee, so just make it a warning.
			// There is a separate status for non-simulation with internal referee.
			status("NO REFEREE", Status_Fail);
		}
		return;
	}

	if (sim)
	{
		// Everything is good for simulation, but not for competition.
		status("SIMULATION", Status_Warning);
		return;
	}
	
	if (!sim && !_processor->externalReferee())
	{
		// Competition must use external referee
		status("INTERNAL REF", Status_Warning);
		return;
	}
	
	if (!sim && !_processor->gameplayModule()->goalie())
	{
		// No goalie.  Not checked in simulation because this is common during development.
		status("NO GOALIE", Status_Warning);
		return;
	}
	
	//FIXME - Can we validate or flag the playbook?
	
	if (!sim && !_processor->logger().recording())
	{
		// We should record logs during competition
		status("NOT RECORDING", Status_Warning);
		return;
	}
	
	status("COMPETITION", Status_OK);
}

void MainWindow::status(QString text, MainWindow::StatusType status)
{
	// Assume that the status type alone won't change.
	if (_ui.statusLabel->text() != text)
	{
		_ui.statusLabel->setText(text);
		
		switch (status)
		{
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

void MainWindow::on_fieldView_robotSelected(int shell)
{
	if (_processor->joystickValid())
	{
		_ui.manualID->setCurrentIndex(shell + 1);
		_processor->manualID(shell);
	}
}

void MainWindow::on_actionRawBalls_toggled(bool state)
{
	_ui.fieldView->showRawBalls = state;
	_ui.fieldView->update();
}

void MainWindow::on_actionRawRobots_toggled(bool state)
{
	_ui.fieldView->showRawRobots = state;
	_ui.fieldView->update();
}

void MainWindow::on_actionCoords_toggled(bool state)
{
	_ui.fieldView->showCoords = state;
	_ui.fieldView->update();
}

void MainWindow::on_actionDefendMinusX_triggered()
{
	_processor->defendPlusX(false);
}

void MainWindow::on_actionDefendPlusX_triggered()
{
	_processor->defendPlusX(true);
}

void MainWindow::on_action0_triggered()
{
	_ui.fieldView->rotate(0);
}

void MainWindow::on_action90_triggered()
{
	_ui.fieldView->rotate(1);
}

void MainWindow::on_action180_triggered()
{
	_ui.fieldView->rotate(2);
}

void MainWindow::on_action270_triggered()
{
	_ui.fieldView->rotate(3);
}

void MainWindow::on_actionUseOurHalf_toggled(bool value)
{
	_processor->useOurHalf(value);
}

void MainWindow::on_actionUseOpponentHalf_toggled(bool value)
{
	_processor->useOpponentHalf(value);
}

void MainWindow::on_actionCenterBall_triggered()
{
	SimCommand cmd;
	cmd.mutable_ball_pos()->set_x(0);
	cmd.mutable_ball_pos()->set_y(0);
	cmd.mutable_ball_vel()->set_x(0);
	cmd.mutable_ball_vel()->set_y(0);
	_ui.fieldView->sendSimCommand(cmd);
}

void MainWindow::on_actionStopBall_triggered()
{
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
	BOOST_FOREACH(OurRobot* robot, state()->self)
	{
		SimCommand::Robot *r = cmd.add_robots();
		r->set_shell(robot->shell());
		r->set_blue_team(processor()->blueTeam());
		r->mutable_vel()->set_x(0);
		r->mutable_vel()->set_y(0);
		r->set_w(0);
	}
	BOOST_FOREACH(OpponentRobot* robot, state()->opp)
	{
		SimCommand::Robot *r = cmd.add_robots();
		r->set_shell(robot->shell());
		r->set_blue_team(processor()->blueTeam());
		r->mutable_vel()->set_x(0);
		r->mutable_vel()->set_y(0);
		r->set_w(0);
	}
	_ui.fieldView->sendSimCommand(cmd);
}

// Debug commands

void MainWindow::on_actionRestartUpdateTimer_triggered()
{
	printf("Update timer: active %d, singleShot %d, interval %d\n", updateTimer.isActive(), updateTimer.isSingleShot(), updateTimer.interval());
	updateTimer.stop();
	updateTimer.start(30);
}

// Gameplay commands

void MainWindow::on_menu_Gameplay_aboutToShow()
{
}

void MainWindow::on_actionSeed_triggered()
{
	QString text = QInputDialog::getText(this, "Set Random Seed", "Hexadecimal seed:");
	if (!text.isNull())
	{
		long seed = strtol(text.toAscii(), 0, 16);
		printf("seed %016lx\n", seed);
		srand48(seed);
	}
}


// Log controls
void MainWindow::on_playbackRate_sliderPressed()
{
	// Stop playback
	live(false);
}

void MainWindow::on_playbackRate_sliderMoved(int value)
{
	live(false);
}

void MainWindow::on_playbackRate_sliderReleased()
{
	// Center the slider and stop playback
	_ui.playbackRate->setValue(0);
}

void MainWindow::on_logNext_clicked()
{
	on_logStop_clicked();
	frameNumber(frameNumber() + 1);
}

void MainWindow::on_logPrev_clicked()
{
	on_logStop_clicked();
	frameNumber(frameNumber() - 1);
}

void MainWindow::on_logStop_clicked()
{
	live(false);
	_ui.playbackRate->setValue(0);
}

void MainWindow::on_logFirst_clicked()
{
	on_logStop_clicked();
	frameNumber(_processor->logger().firstFrameNumber());
}

void MainWindow::on_logLive_clicked()
{
	live(true);
}

void MainWindow::on_actionTeamBlue_triggered()
{
	_ui.team->setText("BLUE");
	_ui.team->setStyleSheet("background-color: #4040ff; color: #ffffff");
	_processor->blueTeam(true);
}

void MainWindow::on_actionTeamYellow_triggered()
{
	_ui.team->setText("YELLOW");
	_ui.team->setStyleSheet("background-color: #ffff00");
	_processor->blueTeam(false);
}

void MainWindow::on_manualID_currentIndexChanged(int value)
{
	_processor->manualID(value - 1);
}

////////
// Debug layer list

void MainWindow::allDebugOn()
{
	for (int i = 0; i < _ui.debugLayers->count(); ++i)
	{
		_ui.debugLayers->item(i)->setCheckState(Qt::Checked);
	}
}

void MainWindow::allDebugOff()
{
	for (int i = 0; i < _ui.debugLayers->count(); ++i)
	{
		_ui.debugLayers->item(i)->setCheckState(Qt::Unchecked);
	}
}

void MainWindow::on_debugLayers_customContextMenuRequested(const QPoint& pos)
{
	QListWidgetItem *item = _ui.debugLayers->itemAt(pos);
	
	QMenu menu;
	QAction *all = menu.addAction("All");
	QAction *none = menu.addAction("None");
	QAction *single = 0, *notSingle = 0;
	if (item)
	{
		single = menu.addAction("Only this");
		notSingle = menu.addAction("All except this");
	}
	
	QAction *act = menu.exec(_ui.debugLayers->mapToGlobal(pos));
	if (act == all)
	{
		allDebugOn();
	} else if (act == none)
	{
		allDebugOff();
	} else if (single && act == single)
	{
		allDebugOff();
		item->setCheckState(Qt::Checked);
	} else if (notSingle && act == notSingle)
	{
		allDebugOn();
		item->setCheckState(Qt::Unchecked);
	}
}

void MainWindow::on_debugLayers_itemChanged(QListWidgetItem* item)
{
	int layer = item->data(Qt::UserRole).toInt();
	if (layer >= 0)
	{
		_ui.fieldView->layerVisible(layer, item->checkState() == Qt::Checked);
	}
	_ui.fieldView->update();
}

////////
// Referee controls

void MainWindow::on_externalReferee_clicked(bool value)
{
	// The user has taken control of external/internal referee selection.
	_autoExternalReferee = false;
}

void MainWindow::on_externalReferee_toggled(bool value)
{
	_processor->externalReferee(value);
	
	// Enable/disable buttons in the Referee tab
	QList<QPushButton *> buttons = _ui.refereeTab->findChildren<QPushButton *>();
	BOOST_FOREACH(QPushButton *btn, buttons)
	{
		btn->setEnabled(!value);
	}

	// Enable/disable shortcut buttons
	buttons = _ui.refShortcuts->findChildren<QPushButton *>();
	BOOST_FOREACH(QPushButton *btn, buttons)
	{
		btn->setEnabled(!value);
	}
}

void MainWindow::on_refHalt_clicked()
{
	_processor->internalRefCommand(RefereeCommands::Halt);
}

void MainWindow::on_refReady_clicked()
{
	_processor->internalRefCommand(RefereeCommands::Ready);
}

void MainWindow::on_refStop_clicked()
{
	_processor->internalRefCommand(RefereeCommands::Stop);
}

void MainWindow::on_refForceStart_clicked()
{
	_processor->internalRefCommand(RefereeCommands::ForceStart);
}

void MainWindow::on_refFirstHalf_clicked()
{
	_processor->internalRefCommand(RefereeCommands::FirstHalf);
}

void MainWindow::on_refOvertime1_clicked()
{
	_processor->internalRefCommand(RefereeCommands::Overtime1);
}

void MainWindow::on_refHalftime_clicked()
{
	_processor->internalRefCommand(RefereeCommands::Halftime);
}

void MainWindow::on_refOvertime2_clicked()
{
	_processor->internalRefCommand(RefereeCommands::Overtime2);
}

void MainWindow::on_refSecondHalf_clicked()
{
	_processor->internalRefCommand(RefereeCommands::SecondHalf);
}

void MainWindow::on_refPenaltyShootout_clicked()
{
	_processor->internalRefCommand(RefereeCommands::PenaltyShootout);
}

void MainWindow::on_refTimeoutBlue_clicked()
{
	_processor->internalRefCommand(RefereeCommands::TimeoutBlue);
}

void MainWindow::on_refTimeoutYellow_clicked()
{
	_processor->internalRefCommand(RefereeCommands::TimeoutYellow);
}

void MainWindow::on_refTimeoutEnd_clicked()
{
	_processor->internalRefCommand(RefereeCommands::TimeoutEnd);
}

void MainWindow::on_refTimeoutCancel_clicked()
{
	_processor->internalRefCommand(RefereeCommands::TimeoutCancel);
}

void MainWindow::on_refKickoffBlue_clicked()
{
	_processor->internalRefCommand(RefereeCommands::KickoffBlue);
}

void MainWindow::on_refKickoffYellow_clicked()
{
	_processor->internalRefCommand(RefereeCommands::KickoffYellow);
}

void MainWindow::on_refDirectBlue_clicked()
{
	_processor->internalRefCommand(RefereeCommands::DirectBlue);
}

void MainWindow::on_refDirectYellow_clicked()
{
	_processor->internalRefCommand(RefereeCommands::DirectYellow);
}

void MainWindow::on_refIndirectBlue_clicked()
{
	_processor->internalRefCommand(RefereeCommands::IndirectBlue);
}

void MainWindow::on_refIndirectYellow_clicked()
{
	_processor->internalRefCommand(RefereeCommands::IndirectYellow);
}

void MainWindow::on_refPenaltyBlue_clicked()
{
	_processor->internalRefCommand(RefereeCommands::PenaltyBlue);
}

void MainWindow::on_refPenaltyYellow_clicked()
{
	_processor->internalRefCommand(RefereeCommands::PenaltyYellow);
}

void MainWindow::on_refGoalBlue_clicked()
{
	_processor->internalRefCommand(RefereeCommands::GoalBlue);
}

void MainWindow::on_refSubtractGoalBlue_clicked()
{
	_processor->internalRefCommand(RefereeCommands::SubtractGoalBlue);
}

void MainWindow::on_refGoalYellow_clicked()
{
	_processor->internalRefCommand(RefereeCommands::GoalYellow);
}

void MainWindow::on_refSubtractGoalYellow_clicked()
{
	_processor->internalRefCommand(RefereeCommands::SubtractGoalYellow);
}

void MainWindow::on_refYellowCardBlue_clicked()
{
	_processor->internalRefCommand(RefereeCommands::YellowCardBlue);
}

void MainWindow::on_refYellowCardYellow_clicked()
{
	_processor->internalRefCommand(RefereeCommands::YellowCardYellow);
}

void MainWindow::on_refRedCardBlue_clicked()
{
	_processor->internalRefCommand(RefereeCommands::RedCardBlue);
}

void MainWindow::on_refRedCardYellow_clicked()
{
	_processor->internalRefCommand(RefereeCommands::RedCardYellow);
}

void MainWindow::on_configTree_itemChanged(QTreeWidgetItem* item, int column)
{
	updateRobotConfigs();
}

void MainWindow::on_loadConfig_clicked()
{
	QString filename = QFileDialog::getOpenFileName(this, "Load Configuration");
	if (!filename.isNull())
	{
		QString error;
		if (!_config->load(filename, error))
		{
			QMessageBox::critical(this, "Load Configuration", error);
		}
	}
}

void MainWindow::on_saveConfig_clicked()
{
	QString filename = QFileDialog::getSaveFileName(this, "Save Configuration");
	if (!filename.isNull())
	{
		QString error;
		if (!_config->save(filename, error))
		{
			QMessageBox::critical(this, "Save Configuration", error);
		}
	}
}
