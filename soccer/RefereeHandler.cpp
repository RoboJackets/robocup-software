#include "RefereeHandler.hpp"

using namespace RefereeCommands;

const char *RefereeAddress = "224.5.23.1";

RefereeHandler::RefereeHandler(SystemState *state)
{
    _counter = -1;
    _state = state;
    _kicked = false;
    _lastBallValid = false;
    
    _state->gameState.period = SystemState::GameState::FirstHalf;
    _state->gameState.state = SystemState::GameState::Stop;
}

void RefereeHandler::run()
{
    if (_state->ball.valid)
    {
        if (_lastBallValid && _state->ball.pos.distTo(_lastBallPos) >= KickThreshold)
        {
            _kicked = true;
        }
        
        if (!_lastBallValid)
        {
            _lastBallValid = true;
            _lastBallPos = _state->ball.pos;
        }
    }
    
    if (_state->gameState.state == SystemState::GameState::Ready && _kicked)
    {
        _state->gameState.state = SystemState::GameState::Playing;
    }
}

void RefereeHandler::packet(const Packet::Referee *packet)
{
    int cmd = packet->command;
    int newCounter = packet->counter;
    
    if (newCounter == _counter && cmd != Halt)
    {
        // Command hasn't changed.
        // We never ignore Halt in case the counter is wrong or state is inconsistent.
        return;
    }
    
    _counter = newCounter;
    
    // Update scores and time
    _state->gameState.scoreBlue = packet->scoreBlue;
    _state->gameState.scoreYellow = packet->scoreYellow;
    _state->gameState.secondsRemaining = packet->timeHigh * 256 + packet->timeLow;

    command(cmd);
}

void RefereeHandler::command(uint8_t command)
{
    // New command
    switch (command)
    {
        // Major states
        case Halt:
            _state->gameState.state = SystemState::GameState::Halt;
            _state->gameState.startingTeam = UnknownTeam;
            _state->gameState.restart = SystemState::GameState::None;
            break;
            
        case Stop:
            _state->gameState.state = SystemState::GameState::Stop;
            _state->gameState.startingTeam = UnknownTeam;
            _state->gameState.restart = SystemState::GameState::None;
            _kicked = false;
            break;
        
        case ForceStart:
            _state->gameState.state = SystemState::GameState::Playing;
            break;
        
        case Ready:
            _state->gameState.state = SystemState::GameState::Ready;
            break;
        
        // Periods
        case FirstHalf:
            _state->gameState.period = SystemState::GameState::FirstHalf;
            break;
        
        case Halftime:
            _state->gameState.period = SystemState::GameState::Halftime;
            break;
        
        case SecondHalf:
            _state->gameState.period = SystemState::GameState::SecondHalf;
            break;
        
        case Overtime1:
            _state->gameState.period = SystemState::GameState::Overtime1;
            break;
        
        case Overtime2:
            _state->gameState.period = SystemState::GameState::Overtime2;
            break;
        
        // Timeouts
        case TimeoutYellow:
        case TimeoutBlue:
            _state->gameState.state = SystemState::GameState::Halt;
            break;
        
        case TimeoutEnd:
            _state->gameState.state = SystemState::GameState::Stop;
            break;
        
        // Restarts
        case KickoffYellow:
            _state->gameState.state = SystemState::GameState::Start;
            _state->gameState.startingTeam = Yellow;
            _state->gameState.restart = SystemState::GameState::Kickoff;
            _lastBallValid = false;
            _kicked = false;
            break;
        
        case KickoffBlue:
            _state->gameState.state = SystemState::GameState::Start;
            _state->gameState.startingTeam = Blue;
            _state->gameState.restart = SystemState::GameState::Kickoff;
            _lastBallValid = false;
            _kicked = false;
            break;
        
        case PenaltyYellow:
            _state->gameState.state = SystemState::GameState::Start;
            _state->gameState.startingTeam = Yellow;
            _state->gameState.restart = SystemState::GameState::Penalty;
            _lastBallValid = false;
            _kicked = false;
            break;
        
        case PenaltyBlue:
            _state->gameState.state = SystemState::GameState::Start;
            _state->gameState.startingTeam = Blue;
            _state->gameState.restart = SystemState::GameState::Penalty;
            _lastBallValid = false;
            _kicked = false;
            break;
        
        case DirectYellow:
            _state->gameState.state = SystemState::GameState::Ready;
            _state->gameState.startingTeam = Yellow;
            _state->gameState.restart = SystemState::GameState::Direct;
            _lastBallValid = false;
            _kicked = false;
            break;
        
        case DirectBlue:
            _state->gameState.state = SystemState::GameState::Ready;
            _state->gameState.startingTeam = Blue;
            _state->gameState.restart = SystemState::GameState::Direct;
            _lastBallValid = false;
            _kicked = false;
            break;
        
        case IndirectYellow:
            _state->gameState.state = SystemState::GameState::Ready;
            _state->gameState.startingTeam = Yellow;
            _state->gameState.restart = SystemState::GameState::Indirect;
            _lastBallValid = false;
            _kicked = false;
            break;
        
        case IndirectBlue:
            _state->gameState.state = SystemState::GameState::Ready;
            _state->gameState.startingTeam = Blue;
            _state->gameState.restart = SystemState::GameState::Indirect;
            _lastBallValid = false;
            _kicked = false;
            break;
    }
}
