
#include <gameplay/GameplayModule.hpp>
#include "MainWindow.hpp"

#include "RefereeModule.hpp"
#include "Configuration.hpp"
#include "QuaternionDemo.hpp"
#include "radio/Radio.hpp"
#include <Utils.hpp>
#include <Robot.hpp>

#include <QInputDialog>
#include <QFileDialog>
#include <QActionGroup>
#include <QMessageBox>

#include <iostream>
#include <boost/foreach.hpp>

#include <ctime>

#include <google/protobuf/descriptor.h>
#include <Network.hpp>
#include <Joystick.hpp>

using namespace std;
using namespace boost;
using namespace google::protobuf;
using namespace Packet;
using namespace Eigen;

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
	_quaternion_demo = 0;
	
	_updateCount = 0;
	_processor = 0;
	_autoExternalReferee = true;
	_doubleFrameNumber = -1;
	
	_lastUpdateTime = timestamp();
	_history.resize(2 * 60);
	
	_ui.setupUi(this);
	_ui.fieldView->history(&_history);
	
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
	
	_ui.splitter->setStretchFactor(0, 88);
	_ui.splitter->setStretchFactor(1, 20);
	
	connect(_ui.manualID, SIGNAL(currentIndexChanged(int)), this, SLOT(on_manualID_currentIndexChanged(int)));

	channel(0);

	updateTimer.setSingleShot(true);
	connect(&updateTimer, SIGNAL(timeout()), SLOT(updateViews()));
	updateTimer.start(30);
}

void MainWindow::configuration(Configuration* config)
{
	_config = config;
	_config->tree(_ui.configTree);
}

void MainWindow::processor(Processor* value)
{
	// This should only happen once
	assert(!_processor);
	
	_processor = value;
	
	// External referee
	//on_externalReferee_toggled(_ui.externalReferee->isChecked());
	
	// Team
	if (_processor->blueTeam())
	{
		_ui.actionTeamBlue->trigger();
	} else {
		_ui.actionTeamYellow->trigger();
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
	int manual =_processor->manualID();
	if ((manual >= 0 || _ui.manualID->isEnabled()) && !_processor->joystickValid())
	{
		// Joystick is gone - turn off manual control
		_ui.manualID->setCurrentIndex(0);
		_processor->manualID(-1);
		_ui.manualID->setEnabled(false);
		_ui.tabWidget->setTabEnabled(2, false);
	} else if (!_ui.manualID->isEnabled() && _processor->joystickValid())
	{
		// Joystick reconnected
		_ui.manualID->setEnabled(true);
		_ui.joystickTab->setVisible(true);
		_ui.tabWidget->setTabEnabled(2, true);
	}
	if(manual >= 0) {
		JoystickControlValues vals = _processor->joystickControlValues();
		_ui.joystickBodyXLabel->setText(tr("%1").arg(vals.bodyX));
		_ui.joystickBodyYLabel->setText(tr("%1").arg(vals.bodyY));
		_ui.joystickBodyWLabel->setText(tr("%1").arg(vals.bodyW));
		_ui.joystickKickPowerLabel->setText(tr("%1").arg(vals.kickPower));
		_ui.joystickDibblerPowerLabel->setText(tr("%1").arg(vals.dribblerPower));
		_ui.joystickKickCheckBox->setChecked(vals.kick);
		_ui.joystickChipCheckBox->setChecked(vals.chip);
		_ui.joystickDribblerCheckBox->setChecked(vals.dribble);
	}
	
	// Time since last update
	uint64_t time = timestamp();
	int delta_us = time - _lastUpdateTime;
	_lastUpdateTime = time;
	double framerate = 1000000.0 / delta_us;
	
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
	
	// Check if any debug layers have been added
	// (layers should never be removed)
	const std::shared_ptr<LogFrame> liveFrame = _processor->logger().lastFrame();
	if (liveFrame && liveFrame->debug_layers_size() > _ui.debugLayers->count())
	{
		// Add the missing layers and turn them on
		for (int i = _ui.debugLayers->count(); i < liveFrame->debug_layers_size(); ++i)
		{
			QListWidgetItem *item = new QListWidgetItem(QString::fromStdString(liveFrame->debug_layers(i)));
			item->setCheckState(Qt::Checked);
			item->setData(Qt::UserRole, i);
			_ui.debugLayers->addItem(item);
		}
		
		_ui.debugLayers->sortItems();
	}
	
	// Get the frame at the log playback time
	const std::shared_ptr<LogFrame> currentFrame = _history[0];
	
	if (currentFrame)
	{
		// Update the orientation demo view
		if (_quaternion_demo && manual >= 0 && currentFrame->radio_rx().size() && currentFrame->radio_rx(0).has_quaternion())
		{
			const RadioRx *manualRx = 0;
			BOOST_FOREACH(const RadioRx &rx, currentFrame->radio_rx())
			{
				if ((int)rx.robot_id() == manual)
				{
					manualRx = &rx;
					break;
				}
			}
			if (manualRx)
			{
				const Packet::Quaternion &q = manualRx->quaternion();
				_quaternion_demo->q = Quaternionf(q.q0(), q.q1(), q.q2(), q.q3());
				if (!_quaternion_demo->initialized)
				{
					_quaternion_demo->ref = _quaternion_demo->q;
					_quaternion_demo->initialized = true;
				}
				_quaternion_demo->update();
			}
		}

		// Update non-message tree items
		_frameNumberItem->setData(ProtobufTree::Column_Value, Qt::DisplayRole, frameNumber());
		int elapsedMillis = (currentFrame->command_time() - _processor->firstLogTime + 500) / 1000;
		QTime elapsedTime = QTime().addMSecs(elapsedMillis);
		_elapsedTimeItem->setText(ProtobufTree::Column_Value, elapsedTime.toString("hh:mm:ss.zzz"));
		
		// Sort the tree by tag if items have been added
		if (_ui.logTree->message(*currentFrame))
		{
			// Items have been added, so sort again on tag number
			_ui.logTree->sortItems(ProtobufTree::Column_Tag, Qt::AscendingOrder);
		}

		//	update the behavior tree view
		_ui.behaviorTree->setPlainText(QString::fromStdString(currentFrame->behavior_tree()));
	}

	if(std::time(0) - (_processor->refereeModule()->received_time/1000000) > 1)
	{
		_ui.fastHalt->setEnabled(true);
		_ui.fastStop->setEnabled(true);
		_ui.fastReady->setEnabled(true);
		_ui.fastForceStart->setEnabled(true);
		_ui.fastKickoffBlue->setEnabled(true);
		_ui.fastKickoffYellow->setEnabled(true);
	}
	else
	{
		_ui.fastHalt->setEnabled(false);
		_ui.fastStop->setEnabled(false);
		_ui.fastReady->setEnabled(false);
		_ui.fastForceStart->setEnabled(false);
		_ui.fastKickoffBlue->setEnabled(false);
		_ui.fastKickoffYellow->setEnabled(false);
	}

	_ui.refStage->setText(NewRefereeModuleEnums::stringFromStage(_processor->refereeModule()->stage).c_str());
	_ui.refCommand->setText(NewRefereeModuleEnums::stringFromCommand(_processor->refereeModule()->command).c_str());

	_ui.refTimeLeft->setText(tr("%1 ms").arg(_processor->refereeModule()->stage_time_left));

	_ui.refBlueName->setText(_processor->refereeModule()->blue_info.name.c_str());
	_ui.refBlueScore->setText(tr("%1").arg(_processor->refereeModule()->blue_info.score));
	_ui.refBlueRedCards->setText(tr("%1").arg(_processor->refereeModule()->blue_info.red_cards));
	_ui.refBlueYellowCards->setText(tr("%1").arg(_processor->refereeModule()->blue_info.yellow_cards));
	_ui.refBlueTimeoutsLeft->setText(tr("%1").arg(_processor->refereeModule()->blue_info.timeouts_left));
	_ui.refBlueGoalie->setText(tr("%1").arg(_processor->refereeModule()->blue_info.goalie));
	
	_ui.refYellowName->setText(_processor->refereeModule()->yellow_info.name.c_str());
	_ui.refYellowScore->setText(tr("%1").arg(_processor->refereeModule()->yellow_info.score));
	_ui.refYellowRedCards->setText(tr("%1").arg(_processor->refereeModule()->yellow_info.red_cards));
	_ui.refYellowYellowCards->setText(tr("%1").arg(_processor->refereeModule()->yellow_info.yellow_cards));
	_ui.refYellowTimeoutsLeft->setText(tr("%1").arg(_processor->refereeModule()->yellow_info.timeouts_left));
	_ui.refYellowGoalie->setText(tr("%1").arg(_processor->refereeModule()->yellow_info.goalie));

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
	uint64_t curTime = timestamp();
	
	// Determine if we are receiving packets from an external referee
	bool haveExternalReferee = (curTime - ps.lastRefereeTime) < 500 * 1000;
	
	/*if (_autoExternalReferee && haveExternalReferee && !_ui.externalReferee->isChecked())
	{
		_ui.externalReferee->setChecked(true);
	}*/
	
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
			//_ui.externalReferee->setChecked(false);
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
	
	//	FIXME: this was disabled in the transition to python for high-level stuff
	//			once that's figured out, we should re-enable this status text
	// if (!sim && !_processor->gameplayModule()->goalie())
	// {
	// 	// No goalie.  Not checked in simulation because this is common during development.
	// 	status("NO GOALIE", Status_Warning);
	// 	return;
	// }
	
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

void MainWindow::on_actionDotPatterns_toggled(bool state)
{
    _ui.fieldView->showDotPatterns = state;
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

void MainWindow::on_action904MHz_triggered()
{
	channel(0);
    _ui.action904MHz->setChecked(true);
    _ui.action906MHz->setChecked(false);
}

void MainWindow::on_action906MHz_triggered()
{
	channel(10);
    _ui.action904MHz->setChecked(false);
    _ui.action906MHz->setChecked(true);
}

void MainWindow::channel(int n)
{
	if (_processor && _processor->radio())
	{
		_processor->radio()->channel(n);
	}
	_ui.radioLabel->setText(QString("%1MHz").arg(904.0 + 0.2 * n, 0, 'f', 1));
}

// Simulator commands

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

// Manual control commands

void MainWindow::on_actionDampedRotation_toggled(bool value)
{
	cout << "DampedRotation is ";
	if (value)
		cout << "Enabled" << endl;
	else
		cout << "Disabled" << endl;
	_processor->dampedRotation(value);
}

void MainWindow::on_actionDampedTranslation_toggled(bool value)
{
	cout << "DampedTranslation is ";
	if (value)
		cout << "Enabled" << endl;
	else
		cout << "Disabled" << endl;
	_processor->dampedTranslation(value);
}

// Debug commands

void MainWindow::on_actionRestartUpdateTimer_triggered()
{
	printf("Update timer: active %d, singleShot %d, interval %d\n", updateTimer.isActive(), updateTimer.isSingleShot(), updateTimer.interval());
	updateTimer.stop();
	updateTimer.start(30);
}

void MainWindow::on_actionQuaternion_Demo_toggled(bool value)
{
	if (value)
	{
		if (_quaternion_demo)
			delete _quaternion_demo;
		cout << "Starting Quaternion Demo" << endl;
		_quaternion_demo = new QuaternionDemo(this);
		_quaternion_demo->resize(640, 480);
	} else
	{
		cout << "Stopping Quaternion Demo" << endl;
		if (_quaternion_demo)
			delete _quaternion_demo;
	}
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
	if (_quaternion_demo)
	{
		if (value == 0)
		{
			_quaternion_demo->hide();
		} else {
			_quaternion_demo->show();
			_quaternion_demo->initialized = false;
		}
	}
}

void MainWindow::on_goalieID_currentIndexChanged(int value)
{
	_processor->goalieID(value - 1);
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

void MainWindow::on_configTree_itemChanged(QTreeWidgetItem* item, int column)
{
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

void MainWindow::setRadioChannel(RadioChannels channel)
{
    switch(channel)
    {
    case RadioChannels::MHz_904:
        this->on_action904MHz_triggered();
        break;
    case RadioChannels::MHz_906:
        this->on_action906MHz_triggered();
        break;
    }
}

void MainWindow::on_fastHalt_clicked()
{
	_processor->refereeModule()->command = NewRefereeModuleEnums::HALT;
}

void MainWindow::on_fastStop_clicked()
{
	_processor->refereeModule()->command = NewRefereeModuleEnums::STOP;
}

void MainWindow::on_fastReady_clicked()
{
	_processor->refereeModule()->command = NewRefereeModuleEnums::NORMAL_START;
}

void MainWindow::on_fastForceStart_clicked()
{
	_processor->refereeModule()->command = NewRefereeModuleEnums::FORCE_START;
}

void MainWindow::on_fastKickoffBlue_clicked()
{
	_processor->refereeModule()->command = NewRefereeModuleEnums::PREPARE_KICKOFF_BLUE;
}

void MainWindow::on_fastKickoffYellow_clicked()
{
	_processor->refereeModule()->command = NewRefereeModuleEnums::PREPARE_KICKOFF_YELLOW;
}

void MainWindow::on_actionVisionFirst_Half_triggered()
{
	_processor->changeVisionChannel(SharedVisionPortFirstHalf);
	_ui.actionVisionFirst_Half->setChecked(true);
	_ui.actionVisionSecond_Half->setChecked(false);
}

void MainWindow::on_actionVisionSecond_Half_triggered()
{
	_processor->changeVisionChannel(SharedVisionPortSecondHalf);
	_ui.actionVisionFirst_Half->setChecked(false);
	_ui.actionVisionSecond_Half->setChecked(true);
}