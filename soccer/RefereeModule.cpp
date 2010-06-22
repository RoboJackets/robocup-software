#include "RefereeModule.hpp"
#include "RefereeModule.moc"

using namespace RefereeCommands;

// Distance in meters that the ball must travel for a kick to be detected
static const float KickThreshold = 0.150f;

// How many milliseconds the ball must be more than KickThreshold meters away from
// its position when the referee indicated Ready for us to detect the ball as having been kicked.
static const int KickVerifyTime_ms = 500;

const char *RefereeAddress = "224.5.23.1";

RefereeModule::RefereeModule(SystemState *state):
	Module("Referee")
{
	_state = state;
	_counter = -1;
	_kickDetectState = WaitForReady;
	
	_widget = new QWidget();
	ui.setupUi(_widget);
	
	_useExternal = ui.externalReferee->isChecked();
	
	((QObject*)_widget)->setParent((QObject*)this);
	QMetaObject::connectSlotsByName(this);
	
	_toolbar = new QToolBar("Referee ToolBar", 0);
	_toolbar->addAction(ui.actionHalt);
	_toolbar->addAction(ui.actionStop);
	_toolbar->addAction(ui.actionReady);
	_toolbar->addAction(ui.actionForceStart);
	_toolbar->addAction(ui.actionKickoffYellow);
	_toolbar->addAction(ui.actionKickoffBlue);
	
	ui.actionHalt->connect(ui.refHalt, SIGNAL(clicked()), SLOT(trigger()));
	ui.actionStop->connect(ui.refStop, SIGNAL(clicked()), SLOT(trigger()));
	ui.actionReady->connect(ui.refReady, SIGNAL(clicked()), SLOT(trigger()));
	ui.actionForceStart->connect(ui.refForceStart, SIGNAL(clicked()), SLOT(trigger()));
	ui.actionKickoffYellow->connect(ui.refKickoffYellow, SIGNAL(clicked()), SLOT(trigger()));
	ui.actionKickoffBlue->connect(ui.refKickoffBlue, SIGNAL(clicked()), SLOT(trigger()));
}

void RefereeModule::run()
{
	if (_state->ball.valid)
	{
		// Only run the kick detector when the ball is visible
		switch (_kickDetectState)
		{
			case WaitForReady:
				// Never kicked and not ready for a restart
				break;
				
			case CapturePosition:
				// Do this in the processing thread
				_readyBallPos = _state->ball.pos;
				_kickDetectState = WaitForKick;
				break;
				
			case WaitForKick:
				if (!_state->ball.pos.nearPoint(_readyBallPos, KickThreshold))
				{
					// The ball appears to have moved
					_kickTime = QTime::currentTime();
					_kickDetectState = VerifyKick;
				}
				break;
				
			case VerifyKick:
				if (_state->ball.pos.nearPoint(_readyBallPos, KickThreshold))
				{
					// The ball is back where it was.  There was probably a vision error.
					_kickDetectState = WaitForKick;
				} else if (_kickTime.msecsTo(QTime::currentTime()) >= KickVerifyTime_ms)
				{
					// The ball has been far enough away for enough time, so call this a kick.
					_kickDetectState = Kicked;
				}
				break;
				
			case Kicked:
				// Stay here until the referee puts us back in Ready
				break;
		}
	}
	
	if (_state->gameState.state == GameState::Ready && kicked())
	{
		_state->gameState.state = GameState::Playing;
	}
}

void RefereeModule::packet(const Packet::Referee *packet)
{
	if (!_useExternal)
	{
		return;
	}
	++_state->gameState.numPackets;
	
	int cmd = packet->command;
	int newCounter = packet->counter;
	
	// Update scores and time
	if (_state->team == Blue)
	{
		_state->gameState.ourScore = packet->scoreBlue;
		_state->gameState.theirScore = packet->scoreYellow;
	} else {
		_state->gameState.ourScore = packet->scoreYellow;
		_state->gameState.theirScore = packet->scoreBlue;
	}
	
	_state->gameState.secondsRemaining = packet->timeHigh * 256 + packet->timeLow;

	if (newCounter != _counter || cmd == Halt)
	{
		// Command has changed.
		// We never ignore Halt in case the counter is wrong or state is inconsistent.
		_counter = newCounter;
		command(cmd);
	}
}

void RefereeModule::command(uint8_t command)
{
	// New command
	switch (command)
	{
		// Major states
		case Halt:
			_state->gameState.state = GameState::Halt;
			_state->gameState.ourRestart = false;
			_state->gameState.restart = GameState::None;
			break;
			
		case Stop:
			_state->gameState.state = GameState::Stop;
			_state->gameState.ourRestart = false;
			_state->gameState.restart = GameState::None;
			_kickDetectState = WaitForReady;
			break;
		
		case ForceStart:
			_state->gameState.state = GameState::Playing;
			break;
		
		case Ready:
			if (_state->gameState.state == GameState::Setup)
			{
				ready();
			}
			break;
		
		// Periods
		case FirstHalf:
			_state->gameState.period = GameState::FirstHalf;
			break;
		
		case Halftime:
			_state->gameState.period = GameState::Halftime;
			break;
		
		case SecondHalf:
			_state->gameState.period = GameState::SecondHalf;
			break;
		
		case Overtime1:
			_state->gameState.period = GameState::Overtime1;
			break;
		
		case Overtime2:
			_state->gameState.period = GameState::Overtime2;
			break;
		
		// Timeouts
		case TimeoutYellow:
		case TimeoutBlue:
			_state->gameState.state = GameState::Halt;
			break;
		
		case TimeoutEnd:
			_state->gameState.state = GameState::Stop;
			break;
		
		// Restarts
		case KickoffYellow:
			_state->gameState.state = GameState::Setup;
			_state->gameState.ourRestart = (_state->team == Yellow);
			_state->gameState.restart = GameState::Kickoff;
			break;
		
		case KickoffBlue:
			_state->gameState.state = GameState::Setup;
			_state->gameState.ourRestart = (_state->team == Blue);
			_state->gameState.restart = GameState::Kickoff;
			break;
		
		case PenaltyYellow:
			_state->gameState.state = GameState::Setup;
			_state->gameState.ourRestart = (_state->team == Yellow);
			_state->gameState.restart = GameState::Penalty;
			break;
		
		case PenaltyBlue:
			_state->gameState.state = GameState::Setup;
			_state->gameState.ourRestart = (_state->team == Blue);
			_state->gameState.restart = GameState::Penalty;
			break;
		
		case DirectYellow:
			ready();
			_state->gameState.ourRestart = (_state->team == Yellow);
			_state->gameState.restart = GameState::Direct;
			break;
		
		case DirectBlue:
			ready();
			_state->gameState.ourRestart = (_state->team == Blue);
			_state->gameState.restart = GameState::Direct;
			break;
		
		case IndirectYellow:
			ready();
			_state->gameState.ourRestart = (_state->team == Yellow);
			_state->gameState.restart = GameState::Indirect;
			break;
		
		case IndirectBlue:
			ready();
			_state->gameState.ourRestart = (_state->team == Blue);
			_state->gameState.restart = GameState::Indirect;
			break;
	}
}

void RefereeModule::ready()
{
	_state->gameState.state = GameState::Ready;
	_kickDetectState = CapturePosition;
}

void RefereeModule::on_actionHalt_triggered()
{
	command('H');
}

void RefereeModule::on_actionReady_triggered()
{
	command(' ');
}

void RefereeModule::on_actionStop_triggered()
{
	command('S');
}

void RefereeModule::on_actionForceStart_triggered()
{
	command('s');
}

void RefereeModule::on_refFirstHalf_clicked()
{
	command('1');
}

void RefereeModule::on_refOvertime1_clicked()
{
	command('o');
}

void RefereeModule::on_refHalftime_clicked()
{
	command('h');
}

void RefereeModule::on_refOvertime2_clicked()
{
	command('O');
}

void RefereeModule::on_refSecondHalf_clicked()
{
	command('2');
}

void RefereeModule::on_refPenaltyShootout_clicked()
{
	command('a');
}

void RefereeModule::on_refTimeoutBlue_clicked()
{
	command('T');
}

void RefereeModule::on_refTimeoutYellow_clicked()
{
	command('t');
}

void RefereeModule::on_refTimeoutEnd_clicked()
{
	command('z');
}

void RefereeModule::on_refTimeoutCancel_clicked()
{
	command('c');
}

void RefereeModule::on_actionKickoffBlue_triggered()
{
	command('K');
}

void RefereeModule::on_actionKickoffYellow_triggered()
{
	command('k');
}

void RefereeModule::on_refDirectBlue_clicked()
{
	command('F');
}

void RefereeModule::on_refDirectYellow_clicked()
{
	command('f');
}

void RefereeModule::on_refIndirectBlue_clicked()
{
	command('I');
}

void RefereeModule::on_refIndirectYellow_clicked()
{
	command('i');
}

void RefereeModule::on_refPenaltyBlue_clicked()
{
	command('P');
}

void RefereeModule::on_refPenaltyYellow_clicked()
{
	command('p');
}

void RefereeModule::on_refGoalBlue_clicked()
{
	if (_state->team == Blue)
	{
		++_state->gameState.ourScore;
	} else {
		++_state->gameState.theirScore;
	}
	command('G');
}

void RefereeModule::on_refSubtractGoalBlue_clicked()
{
	if (_state->team == Blue)
	{
		if (_state->gameState.ourScore)
		{
			--_state->gameState.ourScore;
		}
	} else {
		if (_state->gameState.theirScore)
		{
			--_state->gameState.theirScore;
		}
	}
	
	command('D');
}

void RefereeModule::on_refGoalYellow_clicked()
{
	if (_state->team == Blue)
	{
		++_state->gameState.theirScore;
	} else {
		++_state->gameState.ourScore;
	}
	
	command('g');
}

void RefereeModule::on_refSubtractGoalYellow_clicked()
{
	if (_state->team == Blue)
	{
		if (_state->gameState.theirScore)
		{
			--_state->gameState.theirScore;
		}
	} else {
		if (_state->gameState.ourScore)
		{
			--_state->gameState.ourScore;
		}
	}
	
	command('d');
}

void RefereeModule::on_refYellowCardBlue_clicked()
{
	command('Y');
}

void RefereeModule::on_refYellowCardYellow_clicked()
{
	command('y');
}

void RefereeModule::on_refRedCardBlue_clicked()
{
	command('R');
}

void RefereeModule::on_refRedCardYellow_clicked()
{
	command('r');
}

void RefereeModule::on_externalReferee_toggled(bool value)
{
	_useExternal = value;
}
