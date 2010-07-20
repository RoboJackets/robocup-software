#include <RefereeModule.hpp>
#include <framework/SystemState.hpp>

using namespace RefereeCommands;

// Distance in meters that the ball must travel for a kick to be detected
static const float KickThreshold = 0.150f;

// How many milliseconds the ball must be more than KickThreshold meters away from
// its position when the referee indicated Ready for us to detect the ball as having been kicked.
static const int KickVerifyTime_ms = 500;

RefereeModule::RefereeModule(SystemState *state):
	_mutex(QMutex::Recursive)
{
	_state = state;
	_counter = -1;
	_kickDetectState = WaitForReady;
	_blueTeam = false;
}

void RefereeModule::run()
{
	QMutexLocker locker(&_mutex);
	
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

void RefereeModule::packet(const std::string &packet)
{
	QMutexLocker locker(&_mutex);
	
	int cmd = packet[0];
	uint8_t newCounter = packet[1];
	
	// Update scores and time
	int scoreBlue = packet[2];
	int scoreYellow = packet[3];
	
	if (_blueTeam)
	{
		_state->gameState.ourScore = scoreBlue;
		_state->gameState.theirScore = scoreYellow;
	} else {
		_state->gameState.ourScore = scoreYellow;
		_state->gameState.theirScore = scoreBlue;
	}
	
	_state->gameState.secondsRemaining = (uint8_t)packet[4] * 256 + (uint8_t)packet[5];

	if (newCounter != _counter || cmd == Halt)
	{
		// Command has changed.
		// We never ignore Halt in case the counter is wrong or state is inconsistent.
		_counter = newCounter;
		command(cmd);
	}
}

void RefereeModule::command(char command)
{
	QMutexLocker locker(&_mutex);
	
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
			_state->gameState.ourRestart = !_blueTeam;
			_state->gameState.restart = GameState::Kickoff;
			break;
		
		case KickoffBlue:
			_state->gameState.state = GameState::Setup;
			_state->gameState.ourRestart = _blueTeam;
			_state->gameState.restart = GameState::Kickoff;
			break;
		
		case PenaltyYellow:
			_state->gameState.state = GameState::Setup;
			_state->gameState.ourRestart = !_blueTeam;
			_state->gameState.restart = GameState::Penalty;
			break;
		
		case PenaltyBlue:
			_state->gameState.state = GameState::Setup;
			_state->gameState.ourRestart = _blueTeam;
			_state->gameState.restart = GameState::Penalty;
			break;
		
		case DirectYellow:
			ready();
			_state->gameState.ourRestart = !_blueTeam;
			_state->gameState.restart = GameState::Direct;
			break;
		
		case DirectBlue:
			ready();
			_state->gameState.ourRestart = _blueTeam;
			_state->gameState.restart = GameState::Direct;
			break;
		
		case IndirectYellow:
			ready();
			_state->gameState.ourRestart = !_blueTeam;
			_state->gameState.restart = GameState::Indirect;
			break;
		
		case IndirectBlue:
			ready();
			_state->gameState.ourRestart = _blueTeam;
			_state->gameState.restart = GameState::Indirect;
			break;
	}
}

void RefereeModule::ready()
{
	QMutexLocker locker(&_mutex);
	
	_state->gameState.state = GameState::Ready;
	_kickDetectState = CapturePosition;
}
