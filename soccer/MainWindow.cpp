// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

//FIXME - One time the display quit updating, like the timer died.  GUI was OK but frame # did not advance, etc.  Status was SIMULATION.

#include "MainWindow.hpp"
#include "MainWindow.moc"

#include "PlayConfigTab.hpp"

#include <QInputDialog>
#include <QActionGroup>
#include <boost/foreach.hpp>

#include <google/protobuf/descriptor.h>

using namespace std;
using namespace boost;
using namespace google::protobuf;
using namespace Packet;

// Style sheets used for live/non-live controls
QString LiveStyle("border:2px solid transparent");
QString NonLiveStyle("border:2px solid red");

MainWindow::MainWindow(QWidget *parent):
	QMainWindow(parent)
{
// 	_processor = 0;
	_autoExternalReferee = true;
	_doubleFrameNumber = -1;
	_lastUpdateTime = Utils::timestamp();
	_history.resize(2 * 60);
	
	ui.setupUi(this);
	ui.fieldView->history(&_history);
	
	// Initialize live/non-live control styles
	_live = false;
	live(true);
	
	_currentPlay = new QLabel();
	_currentPlay->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
	_currentPlay->setToolTip("Current Play");
	statusBar()->addPermanentWidget(_currentPlay);
	
	_logFile = new QLabel();
	_logFile->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
	_logFile->setToolTip("Log File");
	statusBar()->addPermanentWidget(_logFile);
	
	_viewFPS = new QLabel();
	_viewFPS->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
	_viewFPS->setToolTip("Display Framerate");
	statusBar()->addPermanentWidget(_viewFPS);
	
	_procFPS = new QLabel();
	_procFPS->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
	_procFPS->setToolTip("Processing Framerate");
	statusBar()->addPermanentWidget(_procFPS);
	
	_logMemory = new QLabel();
	_logMemory->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
	_logMemory->setToolTip("Log Memory Usage");
	statusBar()->addPermanentWidget(_logMemory);
	
	_frameNumberItem = new QTreeWidgetItem(ui.tree);
	_frameNumberItem->setText(ProtobufTree::Column_Field, "Frame");
	_frameNumberItem->setData(ProtobufTree::Column_Tag, Qt::DisplayRole, -2);
	
	_elapsedTimeItem = new QTreeWidgetItem(ui.tree);
	_elapsedTimeItem->setText(ProtobufTree::Column_Field, "Elapsed Time");
	_elapsedTimeItem->setData(ProtobufTree::Column_Tag, Qt::DisplayRole, -1);
	
	ui.debugLayers->setContextMenuPolicy(Qt::CustomContextMenu);
	
	// Connect shortcut buttons to regular referee buttons
	connect(ui.fastHalt, SIGNAL(clicked()), ui.refHalt, SLOT(click()));
	connect(ui.fastStop, SIGNAL(clicked()), ui.refStop, SLOT(click()));
	connect(ui.fastReady, SIGNAL(clicked()), ui.refReady, SLOT(click()));
	connect(ui.fastForceStart, SIGNAL(clicked()), ui.refForceStart, SLOT(click()));
	connect(ui.fastKickoffBlue, SIGNAL(clicked()), ui.refKickoffBlue, SLOT(click()));
	connect(ui.fastKickoffYellow, SIGNAL(clicked()), ui.refKickoffYellow, SLOT(click()));
	
	QActionGroup *teamGroup = new QActionGroup(this);
	teamGroup->addAction(ui.actionTeamBlue);
	teamGroup->addAction(ui.actionTeamYellow);
	
	QActionGroup *goalGroup = new QActionGroup(this);
	goalGroup->addAction(ui.actionDefendMinusX);
	goalGroup->addAction(ui.actionDefendPlusX);
	
	QActionGroup *rotateGroup = new QActionGroup(this);
	rotateGroup->addAction(ui.action0);
	rotateGroup->addAction(ui.action90);
	rotateGroup->addAction(ui.action180);
	rotateGroup->addAction(ui.action270);
	
	ui.splitter->setStretchFactor(0, 98);
	ui.splitter->setStretchFactor(1, 10);

	_updateTimer.setSingleShot(true);
	connect(&_updateTimer, SIGNAL(timeout()), SLOT(updateViews()));
	_updateTimer.start(30);
}

void MainWindow::processor(Processor* value)
{
	// This should only happen once
	assert(!_processor);
	
	_processor = value;
	
	// External referee
	on_externalReferee_toggled(ui.externalReferee->isChecked());
	
	// Team
	if (_processor->blueTeam())
	{
		ui.actionTeamBlue->trigger();
	} else {
		ui.actionTeamYellow->trigger();
	}
	
	// Add Plays tab
	_playConfigTab = new PlayConfigTab();
	ui.tabWidget->addTab(_playConfigTab, tr("Plays"));
	_playConfigTab->setup(_processor->gameplayModule());
	
	// Add Configuration tab
	_configFileTab = new ConfigFileTab(_processor->configFile());
	ui.tabWidget->addTab(_configFileTab, tr("Configuration"));

	// Radio channel
	ui.radioLabel->setText(QString("Radio %1").arg(_processor->radio()));
}

void MainWindow::logFileChanged()
{
	if (_processor->logger.recording())
	{
		_logFile->setText(_processor->logger.filename());
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
		if (_live)
		{
			ui.fieldView->setStyleSheet(QString());
			ui.tree->setStyleSheet(QString("QTreeWidget{%1}").arg(LiveStyle));
		} else {
			ui.fieldView->setStyleSheet(NonLiveStyle);
			ui.tree->setStyleSheet(QString("QTreeWidget{%1}").arg(NonLiveStyle));
		}
	}
}

void MainWindow::updateViews()
{
	int manual =_processor->manualID();
	if ((manual >= 0 || ui.manualID->isEnabled()) && !_processor->joystickValid())
	{
		// Joystick is gone - turn off manual control
		_processor->manualID(-1);
		ui.manualID->setEnabled(false);
	} else if (!ui.manualID->isEnabled() && _processor->joystickValid())
	{
		// Joystick reconnected
		ui.manualID->setEnabled(true);
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
	if (_processor->gameplayModule()->forcePlay())
	{
		play += " (forced)";
	}
	_currentPlay->setText(play);
	
	_viewFPS->setText(QString("View: %1 fps").arg(framerate, 0, 'f', 1));
	_procFPS->setText(QString("Proc: %1 fps").arg(_processor->framerate(), 0, 'f', 1));
	
	_logMemory->setText(QString("Log: %1/%2 %3 kiB").arg(
		QString::number(_processor->logger.numFrames()),
		QString::number(_processor->logger.maxFrames()),
		QString::number((_processor->logger.spaceUsed() + 512) / 1024)
	));
	
	// Advance log playback time
	int liveFrameNumber = _processor->logger.lastFrame();
	if (_live)
	{
		_doubleFrameNumber = liveFrameNumber;
	} else {
		double rate = ui.playbackRate->value();
		_doubleFrameNumber += rate / framerate;
		
		int minFrame = _processor->logger.firstFrame();
		int maxFrame = _processor->logger.lastFrame();
		if (_doubleFrameNumber < minFrame)
		{
			_doubleFrameNumber = minFrame;
		} else if (_doubleFrameNumber > maxFrame)
		{
			_doubleFrameNumber = maxFrame;
		}
	}
	
	// Read recent history from the log
	_processor->logger.getFrames(frameNumber(), _history);
	
	// Get the frame at the log playback time
	const LogFrame &currentFrame = _history[0];
	
	// Update field view
	ui.fieldView->repaint(ui.fieldView->rect());
	
	// Get the live frame from FieldView or the Logger
	LogFrame liveFrameStorage;
	const LogFrame *liveFrame;
	if (_live)
	{
		liveFrame = &_history[0];
	} else {
		liveFrame = &liveFrameStorage;
		
		// Not live, so we have to read the live frame ourselves
		int i = _processor->logger.lastFrame();
		if (i >= 0)
		{
			_processor->logger.getFrame(i, liveFrameStorage);
		}
	}
	
	// Update log controls
	ui.logLive->setEnabled(!_live);
	ui.logStop->setEnabled(_live);
	
	// Update status indicator
	updateStatus();
	
	// Update play list
	_playConfigTab->frameUpdate();
	
	// Check if any debug layers have been added
	// (layers should never be removed)
	if (liveFrame->debug_layers_size() > ui.debugLayers->count())
	{
		// Add the missing layers and turn them on
		for (int i = ui.debugLayers->count(); i < liveFrame->debug_layers_size(); ++i)
		{
			QListWidgetItem *item = new QListWidgetItem(QString::fromStdString(liveFrame->debug_layers(i)));
			item->setCheckState(ui.fieldView->layerVisible(i) ? Qt::Checked : Qt::Unchecked);
			item->setData(Qt::UserRole, i);
			ui.debugLayers->addItem(item);
		}
		
		ui.debugLayers->sortItems();
	}
	
	// Update non-message tree items
	_frameNumberItem->setData(ProtobufTree::Column_Value, Qt::DisplayRole, frameNumber());
	int elapsedMillis = (currentFrame.start_time() - _processor->firstLogTime + 500) / 1000;
	QTime elapsedTime = QTime().addMSecs(elapsedMillis);
	_elapsedTimeItem->setText(ProtobufTree::Column_Value, elapsedTime.toString("hh:mm:ss.zzz"));
	
	// Sort the tree by tag if items have been added
	if (ui.tree->message(currentFrame))
	{
		// Items have been added, so sort again on tag number
		ui.tree->sortItems(ProtobufTree::Column_Tag, Qt::AscendingOrder);
	}
	
	// We restart this timer repeatedly instead of using a single shot timer because
	// we don't want to use 100% CPU redrawing the view if it takes too long.
	_updateTimer.start(30);
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
	
	if (_autoExternalReferee && haveExternalReferee && !ui.externalReferee->isChecked())
	{
		ui.externalReferee->setChecked(true);
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
			ui.externalReferee->setChecked(false);
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
	
	if (!sim && !_processor->logger.recording())
	{
		// We should record logs during competition
		status("NOT RECORDING", Status_Warning);
		return;
	}
	
	status("COMPETITION", Status_OK);
}

void MainWindow::status(QString text, MainWindow::StatusType status)
{
	ui.statusLabel->setText(text);
	
	switch (status)
	{
		case Status_OK:
			ui.statusLabel->setStyleSheet("background-color: #00ff00");
			break;
		
		case Status_Warning:
			ui.statusLabel->setStyleSheet("background-color: #ffff00");
			break;
		
		case Status_Fail:
			ui.statusLabel->setStyleSheet("background-color: #ff4040");
			break;
	}
}

void MainWindow::on_fieldView_robotSelected(int shell)
{
	ui.manualID->setCurrentIndex(shell + 1);
	_processor->manualID(shell);
}

void MainWindow::on_actionRawBalls_toggled(bool state)
{
	ui.fieldView->showRawBalls = state;
	ui.fieldView->update();
}

void MainWindow::on_actionRawRobots_toggled(bool state)
{
	ui.fieldView->showRawRobots = state;
	ui.fieldView->update();
}

void MainWindow::on_actionCoords_toggled(bool state)
{
	ui.fieldView->showCoords = state;
	ui.fieldView->update();
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
	ui.fieldView->rotate(0);
}

void MainWindow::on_action90_triggered()
{
	ui.fieldView->rotate(1);
}

void MainWindow::on_action180_triggered()
{
	ui.fieldView->rotate(2);
}

void MainWindow::on_action270_triggered()
{
	ui.fieldView->rotate(3);
}

void MainWindow::on_actionCenterBall_triggered()
{
	SimCommand cmd;
	cmd.mutable_ball_pos()->set_x(0);
	cmd.mutable_ball_pos()->set_y(0);
	cmd.mutable_ball_vel()->set_x(0);
	cmd.mutable_ball_vel()->set_y(0);
	ui.fieldView->sendSimCommand(cmd);
}

void MainWindow::on_actionStopBall_triggered()
{
	SimCommand cmd;
	cmd.mutable_ball_vel()->set_x(0);
	cmd.mutable_ball_vel()->set_y(0);
	ui.fieldView->sendSimCommand(cmd);
}

// Debug commands

void MainWindow::on_actionRestartUpdateTimer_triggered()
{
	printf("Update timer: active %d, singleShot %d, interval %d\n", _updateTimer.isActive(), _updateTimer.isSingleShot(), _updateTimer.interval());
	_updateTimer.stop();
	_updateTimer.start(30);
}

// Gameplay commands

void MainWindow::on_menu_Gameplay_aboutToShow()
{
	ui.actionUnforce->setEnabled(_processor->gameplayModule()->forcePlay());
}

void MainWindow::on_actionUnforce_triggered()
{
	_processor->gameplayModule()->forcePlay(0);
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
	ui.playbackRate->setValue(0);
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
	ui.playbackRate->setValue(0);
}

void MainWindow::on_logFirst_clicked()
{
	on_logStop_clicked();
	frameNumber(_processor->logger.firstFrame());
}

void MainWindow::on_logLive_clicked()
{
	live(true);
}

void MainWindow::on_actionTeamBlue_triggered()
{
	ui.team->setText("BLUE");
	ui.team->setStyleSheet("background-color: #4040ff; color: #ffffff");
	_processor->blueTeam(true);
}

void MainWindow::on_actionTeamYellow_triggered()
{
	ui.team->setText("YELLOW");
	ui.team->setStyleSheet("background-color: #ffff00");
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
	for (int i = 0; i < ui.debugLayers->count(); ++i)
	{
		ui.debugLayers->item(i)->setCheckState(Qt::Checked);
	}
}

void MainWindow::allDebugOff()
{
	for (int i = 0; i < ui.debugLayers->count(); ++i)
	{
		ui.debugLayers->item(i)->setCheckState(Qt::Unchecked);
	}
}

void MainWindow::on_debugLayers_customContextMenuRequested(const QPoint& pos)
{
	QListWidgetItem *item = ui.debugLayers->itemAt(pos);
	
	QMenu menu;
	QAction *all = menu.addAction("All");
	QAction *none = menu.addAction("None");
	QAction *single = 0, *notSingle = 0;
	if (item)
	{
		single = menu.addAction("Only this");
		notSingle = menu.addAction("All except this");
	}
	
	QAction *act = menu.exec(ui.debugLayers->mapToGlobal(pos));
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
		ui.fieldView->layerVisible(layer, item->checkState() == Qt::Checked);
	}
	ui.fieldView->update();
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
	QList<QPushButton *> buttons = ui.refereeTab->findChildren<QPushButton *>();
	BOOST_FOREACH(QPushButton *btn, buttons)
	{
		btn->setEnabled(!value);
	}

	// Enable/disable shortcut buttons
	buttons = ui.refShortcuts->findChildren<QPushButton *>();
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
