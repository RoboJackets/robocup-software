// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include "MainWindow.hpp"
#include "MainWindow.moc"

#include "PlayConfigTab.hpp"

#include <QActionGroup>
#include <boost/foreach.hpp>

#include <google/protobuf/descriptor.h>

using namespace std;
using namespace boost;
using namespace google::protobuf;
using namespace Packet;

MainWindow::MainWindow(QWidget *parent):
	QMainWindow(parent)
{
	_processor = 0;
	_autoExternalReferee = true;
	_treeInitialized = false;
	
	ui.setupUi(this);
	
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
	ui.fieldView->logger = &_processor->logger;
	
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

void MainWindow::updateViews()
{
	ui.fieldView->update();
	updateStatus();
	_playConfigTab->frameUpdate();
	
	// Check if any debug layers have been added
	// (layers should never be removed)
	LogFrame frame;
	int i = _processor->logger.lastFrame();
	if (i >= 0)
	{
		// At least one log frame is available, so read it.
		_processor->logger.getFrame(i, frame);
		
		if (frame.debug_layers_size() > ui.debugLayers->count())
		{
			// FieldView does this, but we may have received another frame between frameUpdate() and here.
			ui.fieldView->layerVisible.resize(frame.debug_layers_size());
			
			// Add the missing layers and turn them on
			for (int i = ui.debugLayers->count(); i < frame.debug_layers_size(); ++i)
			{
				QListWidgetItem *item = new QListWidgetItem(QString::fromStdString(frame.debug_layers(i)));
				item->setCheckState(Qt::Checked);
				item->setData(Qt::UserRole, i);
				ui.debugLayers->addItem(item);
				
				ui.fieldView->layerVisible[i] = true;
			}
			
			ui.debugLayers->sortItems();
		}
		
		// Update the tree
		ui.tree->clear();
		addTreeData(ui.tree->invisibleRootItem(), frame);
		if (!_treeInitialized)
		{
			ui.tree->resizeColumnToContents(0);
			ui.tree->resizeColumnToContents(1);
			_treeInitialized = true;
		}
	}
	
	//FIXME - Restart the timer AFTER the view has been drawn.  update() does not block on graphics (because it doesn't actually draw).
	// We restart this timer repeatedly instead of using a single shot timer because
	// we don't want to use 100% CPU redrawing the view if it takes too long.
	_updateTimer.start(30);
}

void MainWindow::addTreeData(QTreeWidgetItem *parent, const google::protobuf::Message& msg)
{
	const Reflection *ref = msg.GetReflection();
	vector<const FieldDescriptor *> fields;
	ref->ListFields(msg, &fields);
	
	BOOST_FOREACH(const FieldDescriptor *field, fields)
	{
		QTreeWidgetItem *item = new QTreeWidgetItem(parent);
		item->setText(0, QString::fromStdString(field->name()));
		
		if (field->is_repeated())
		{
			// Repeated field
			int n = ref->FieldSize(msg, field);
			
			// Show the number of elements as the value for the field itself
			item->setText(1, QString("<%1>").arg(n));
			
			// Add each element as a child
			for (int i = 0; i < n; ++i)
			{
				QTreeWidgetItem *element = new QTreeWidgetItem(item);
				element->setText(0, QString("[%1]").arg(i));
				
				switch (field->type())
				{
					case FieldDescriptor::TYPE_INT32:
					case FieldDescriptor::TYPE_SINT32:
					case FieldDescriptor::TYPE_FIXED32:
					case FieldDescriptor::TYPE_SFIXED32:
						element->setText(1, QString::number(ref->GetRepeatedInt32(msg, field, i)));
						break;
					
					case FieldDescriptor::TYPE_INT64:
					case FieldDescriptor::TYPE_SINT64:
					case FieldDescriptor::TYPE_FIXED64:
					case FieldDescriptor::TYPE_SFIXED64:
						element->setText(1, QString::number(ref->GetRepeatedInt64(msg, field, i)));
						break;
					
					case FieldDescriptor::TYPE_UINT32:
						element->setText(1, QString::number(ref->GetRepeatedUInt32(msg, field, i)));
						break;
					
					case FieldDescriptor::TYPE_UINT64:
						element->setText(1, QString::number(ref->GetRepeatedUInt64(msg, field, i)));
						break;
					
					case FieldDescriptor::TYPE_FLOAT:
						element->setText(1, QString::number(ref->GetRepeatedFloat(msg, field, i)));
						break;
					
					case FieldDescriptor::TYPE_DOUBLE:
						element->setText(1, QString::number(ref->GetRepeatedDouble(msg, field, i)));
						break;
					
					case FieldDescriptor::TYPE_BOOL:
						element->setCheckState(1, ref->GetRepeatedBool(msg, field, i) ? Qt::Checked : Qt::Unchecked);
						break;
					
					case FieldDescriptor::TYPE_ENUM:
					{
						const EnumValueDescriptor *ev = ref->GetRepeatedEnum(msg, field, i);
						element->setText(1, QString::fromStdString(ev->name()));
						break;
					}
					
					case FieldDescriptor::TYPE_STRING:
						element->setText(1, QString::fromStdString(ref->GetRepeatedString(msg, field, i)));
						break;
					
					case FieldDescriptor::TYPE_MESSAGE:
						addTreeData(element, ref->GetRepeatedMessage(msg, field, i));
						break;
					
					default:
						element->setText(1, "???");
						break;
				}
			}
		} else switch (field->type())
		{
			case FieldDescriptor::TYPE_INT32:
			case FieldDescriptor::TYPE_SINT32:
			case FieldDescriptor::TYPE_FIXED32:
			case FieldDescriptor::TYPE_SFIXED32:
				item->setText(1, QString::number(ref->GetInt32(msg, field)));
				break;
			
			case FieldDescriptor::TYPE_INT64:
			case FieldDescriptor::TYPE_SINT64:
			case FieldDescriptor::TYPE_FIXED64:
			case FieldDescriptor::TYPE_SFIXED64:
				item->setText(1, QString::number(ref->GetInt64(msg, field)));
				break;
			
			case FieldDescriptor::TYPE_UINT32:
				item->setText(1, QString::number(ref->GetUInt32(msg, field)));
				break;
			
			case FieldDescriptor::TYPE_UINT64:
				item->setText(1, QString::number(ref->GetUInt64(msg, field)));
				break;
			
			case FieldDescriptor::TYPE_FLOAT:
				item->setText(1, QString::number(ref->GetFloat(msg, field)));
				break;
			
			case FieldDescriptor::TYPE_DOUBLE:
				item->setText(1, QString::number(ref->GetDouble(msg, field)));
				break;
			
			case FieldDescriptor::TYPE_BOOL:
				item->setCheckState(1, ref->GetBool(msg, field) ? Qt::Checked : Qt::Unchecked);
				break;
			
			case FieldDescriptor::TYPE_ENUM:
			{
				const EnumValueDescriptor *ev = ref->GetEnum(msg, field);
				item->setText(1, QString::fromStdString(ev->name()));
				break;
			}
			
			case FieldDescriptor::TYPE_STRING:
				item->setText(1, QString::fromStdString(ref->GetString(msg, field)));
				break;
			
			case FieldDescriptor::TYPE_MESSAGE:
				addTreeData(item, ref->GetMessage(msg, field));
				ui.tree->expandItem(item);
				break;
			
			default:
				item->setText(1, "???");
				break;
		}
	}
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
	
	if (_autoExternalReferee && ps.lastRefereeTime && !ui.externalReferee->isChecked())
	{
		ui.externalReferee->setChecked(true);
	}
	
	// Is the processing thread running?
	uint64_t curTime = Utils::timestamp();
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
	
	if ((!sim || _processor->externalReferee()) && curTime - ps.lastRefereeTime > 500 * 1000)
	{
		if (_autoExternalReferee && ui.externalReferee->isChecked())
		{
			// Automatically turn off external referee when in simulation
			ui.externalReferee->setChecked(false);
		}
		
		// In simulation, we will often run without a referee, so just make it a warning.
		// There is a separate status for non-simulation with internal referee.
		status("NO REFEREE", Status_Fail);
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
		ui.fieldView->layerVisible[layer] = (item->checkState() == Qt::Checked);
	}
	ui.fieldView->update();
}

////////
// Referee controls

void MainWindow::refCommand(char ch)
{
	QMutexLocker locker(&_processor->loopMutex);
	
	_processor->refereeModule()->command(ch);
}

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
	refCommand(RefereeCommands::Halt);
}

void MainWindow::on_refReady_clicked()
{
	refCommand(RefereeCommands::Ready);
}

void MainWindow::on_refStop_clicked()
{
	refCommand(RefereeCommands::Stop);
}

void MainWindow::on_refForceStart_clicked()
{
	refCommand(RefereeCommands::ForceStart);
}

void MainWindow::on_refFirstHalf_clicked()
{
	refCommand(RefereeCommands::FirstHalf);
}

void MainWindow::on_refOvertime1_clicked()
{
	refCommand(RefereeCommands::Overtime1);
}

void MainWindow::on_refHalftime_clicked()
{
	refCommand(RefereeCommands::Halftime);
}

void MainWindow::on_refOvertime2_clicked()
{
	refCommand(RefereeCommands::Overtime2);
}

void MainWindow::on_refSecondHalf_clicked()
{
	refCommand(RefereeCommands::SecondHalf);
}

void MainWindow::on_refPenaltyShootout_clicked()
{
	refCommand(RefereeCommands::PenaltyShootout);
}

void MainWindow::on_refTimeoutBlue_clicked()
{
	refCommand(RefereeCommands::TimeoutBlue);
}

void MainWindow::on_refTimeoutYellow_clicked()
{
	refCommand(RefereeCommands::TimeoutYellow);
}

void MainWindow::on_refTimeoutEnd_clicked()
{
	refCommand(RefereeCommands::TimeoutEnd);
}

void MainWindow::on_refTimeoutCancel_clicked()
{
	refCommand(RefereeCommands::TimeoutCancel);
}

void MainWindow::on_refKickoffBlue_clicked()
{
	refCommand(RefereeCommands::KickoffBlue);
}

void MainWindow::on_refKickoffYellow_clicked()
{
	refCommand(RefereeCommands::KickoffYellow);
}

void MainWindow::on_refDirectBlue_clicked()
{
	refCommand(RefereeCommands::DirectBlue);
}

void MainWindow::on_refDirectYellow_clicked()
{
	refCommand(RefereeCommands::DirectYellow);
}

void MainWindow::on_refIndirectBlue_clicked()
{
	refCommand(RefereeCommands::IndirectBlue);
}

void MainWindow::on_refIndirectYellow_clicked()
{
	refCommand(RefereeCommands::IndirectYellow);
}

void MainWindow::on_refPenaltyBlue_clicked()
{
	refCommand(RefereeCommands::PenaltyBlue);
}

void MainWindow::on_refPenaltyYellow_clicked()
{
	refCommand(RefereeCommands::PenaltyYellow);
}

void MainWindow::on_refGoalBlue_clicked()
{
	QMutexLocker locker(&_processor->loopMutex);
	
	if (_processor->blueTeam())
	{
		++state()->gameState.ourScore;
	} else {
		++state()->gameState.theirScore;
	}
	
	refCommand(RefereeCommands::GoalBlue);
}

void MainWindow::on_refSubtractGoalBlue_clicked()
{
	QMutexLocker locker(&_processor->loopMutex);
	
	if (_processor->blueTeam())
	{
		if (state()->gameState.ourScore)
		{
			--state()->gameState.ourScore;
		}
	} else {
		if (state()->gameState.theirScore)
		{
			--state()->gameState.theirScore;
		}
	}
	
	refCommand(RefereeCommands::SubtractGoalBlue);
}

void MainWindow::on_refGoalYellow_clicked()
{
	QMutexLocker locker(&_processor->loopMutex);
	
	if (_processor->blueTeam())
	{
		++state()->gameState.theirScore;
	} else {
		++state()->gameState.ourScore;
	}
	
	refCommand(RefereeCommands::GoalYellow);
}

void MainWindow::on_refSubtractGoalYellow_clicked()
{
	QMutexLocker locker(&_processor->loopMutex);
	
	if (_processor->blueTeam())
	{
		if (state()->gameState.theirScore)
		{
			--state()->gameState.theirScore;
		}
	} else {
		if (state()->gameState.ourScore)
		{
			--state()->gameState.ourScore;
		}
	}
	
	refCommand(RefereeCommands::SubtractGoalYellow);
}

void MainWindow::on_refYellowCardBlue_clicked()
{
	refCommand(RefereeCommands::YellowCardBlue);
}

void MainWindow::on_refYellowCardYellow_clicked()
{
	refCommand(RefereeCommands::YellowCardYellow);
}

void MainWindow::on_refRedCardBlue_clicked()
{
	refCommand(RefereeCommands::RedCardBlue);
}

void MainWindow::on_refRedCardYellow_clicked()
{
	refCommand(RefereeCommands::RedCardYellow);
}
