#include "Ref.hpp"

#include <netinet/in.h>

using namespace Geometry;

//control commands - cause the game to run or stop
static const char Halt = 'H'; //halt gameplay
static const char Stop = 'S'; //stop gameplay after something happens, 500mm circle
static const char Start = ' '; //restart tean allowed to act
static const char ForceStart = 's'; //everyone allowed to act

//game notifications
static const char Start1stHalf = '1';
static const char HalfTime = 'h';
static const char Start2ndHalf = '2';
static const char Overtime1 = 'o';
static const char Overtime2 = 'O';
static const char PenaltyShootout = 'a';

//game restarts
static const char KickOffYellow = 'k';
static const char PenaltyYellow = 'p';
static const char DirectFreeKickYellow = 'f';
static const char IndirectFreeKickYellow = 'i';

static const char KickOffBlue = 'K';
static const char PenaltyBlue = 'P';
static const char DirectFreeKickBlue = 'F';
static const char IndirectFreeKickBlue = 'I';

//extra - cause other game control type actions
static const char TimeoutYellow = 't';
static const char GoalScoredYellow = 'g';
static const char DecreaseGoalScoreYellow = 'd';
static const char YellowCardYellow = 'y';
static const char RedCardYellow = 'r';

static const char TimeoutBlue = 'T';
static const char GoalScoredBlue = 'G';
static const char DecreaseGoalScoreBlue = 'D';
static const char YellowCardBlue = 'Y';
static const char RedCardBlue = 'R';

static const char TimeoutEnd = 'z';

static const char Cancel = 'c';

Ref::Ref(Team t) :
	_team(t), _sender(t), _lastCounter(-1)
{
	
}

void Ref::refPacketHandler(const RefPacket* data)
{
	_refPacket = *data;
	
	//if counter == 0 and we != 0, ref program restarted
	if (data->counter == 0 && _lastCounter > 0)
	{
		_lastCounter = -1;
		_refState.state = Packet::Ref::Halt;
		_refState.start = Packet::Ref::None;
		_refState.ourStart = false;
	}
}

void Ref::visionHandler(const Packet::VisionData* vd)
{
	//test for ball movement when waiting for opponent to act
	
	_refState.timestamp = vd->timestamp;
	
	//on setup, record ball position
	if (_refState.state == Packet::Ref::Setup && vd->ball.valid)
	{
		_setupBall = vd->ball.pos;
	}
	//if we are waiting for opposing team to touch ball
	//check distance the ball has moved
	else if (_refState.state == Packet::Ref::OppStart && vd->ball.valid && 
		_setupBall.distTo(vd->ball.pos) > .05)
	{
		//move into normal gameplay for everyone
		_refState.state = Packet::Ref::Running;
		
		//clear the state
		_refState.start = Packet::Ref::None;
		
		//NOTE: once the ball has moved, everyone can move how they want
		printState();
	}
}

void Ref::proc()
{
	//if we have a new packet, process the information
	if (_refPacket.counter != _lastCounter)
	{
		_lastCounter = _refPacket.counter;
		_refState.counter = _lastCounter;
		
		//convert the only short to host order
		_refPacket.time = ntohs(_refPacket.time);
		
		//set the goal count
		if (_team == Yellow)
		{
			_refState.goalsSelf = _refPacket.goalsYellow;
			_refState.goalsOpp = _refPacket.goalsBlue;
		}
		else
		{
			_refState.goalsSelf = _refPacket.goalsBlue;
			_refState.goalsOpp = _refPacket.goalsYellow;
		}

		Team startTeam = UnknownTeam;
		switch (_refPacket.cmd)
		{
			// Game state
			case Halt:
				// All robots must stop moving.
				_refState.state = Packet::Ref::Halt;
				_refState.start = Packet::Ref::None;
				_refState.ourStart = false;
				break;
			case Stop:
				// Stop gameplay, all robots must keep away from ball
				_refState.state = Packet::Ref::Stop;
				_refState.start = Packet::Ref::None;
				_refState.ourStart = false;
				break;
			case Start:
				// Start playing.
				if (_refState.ourStart)
				{
					// Our start, immediately run, can touch ball
					_refState.state = Packet::Ref::Running;
				}
				else
				{
					// Their start, wait for ball motion
					// 500 mm circle
					// see the vision handler
					_refState.state = Packet::Ref::OppStart;
				}
				break;
			case ForceStart:
				// Start playing even if nobody kicked the ball.
				_refState.start = Packet::Ref::None;
				_refState.state = Packet::Ref::Running;
				break;
				
			/// Period
			case Start1stHalf:
				_refState.period = Packet::Ref::FirstHalf;
				break;
			case HalfTime:
				_refState.period = Packet::Ref::HalfTime;
				break;
			case Start2ndHalf:
				_refState.period = Packet::Ref::SecondHalf;
				break;
			case Overtime1:
				_refState.period = Packet::Ref::Overtime1;
				break;
			case Overtime2:
				_refState.period = Packet::Ref::Overtime2;
				break;
			case PenaltyShootout:
				_refState.period = Packet::Ref::PenaltyShootout;
				break;
				
				
				// Starts for yellow
			case KickOffYellow:
				_refState.start = Packet::Ref::Kickoff;
				_refState.state = Packet::Ref::Setup;
				startTeam = Yellow;
				break;
			case PenaltyYellow:
				_refState.start = Packet::Ref::Penalty;
				_refState.state = Packet::Ref::Setup;
				startTeam = Yellow;
				break;
			case DirectFreeKickYellow:
				_refState.start = Packet::Ref::Direct;
				_refState.state = Packet::Ref::Running;
				startTeam = Yellow;
				break;
			case IndirectFreeKickYellow:
				_refState.start = Packet::Ref::Indirect;
				_refState.state = Packet::Ref::Running;
				startTeam = Yellow;
				break;
				
				
				// Starts for blue
			case KickOffBlue:
				_refState.start = Packet::Ref::Kickoff;
				_refState.state = Packet::Ref::Setup;
				startTeam = Blue;
				break;
			case PenaltyBlue:
				_refState.start = Packet::Ref::Penalty;
				_refState.state = Packet::Ref::Setup;
				startTeam = Blue;
				break;
			case DirectFreeKickBlue:
				_refState.start = Packet::Ref::Direct;
				_refState.state = Packet::Ref::Running;
				startTeam = Blue;
				break;
			case IndirectFreeKickBlue:
				_refState.start = Packet::Ref::Indirect;
				_refState.state = Packet::Ref::Running;
				startTeam = Blue;
				break;
				
			/// timeouts
			case TimeoutBlue:
			case TimeoutYellow:
				_refState.start = Packet::Ref::None;
				_refState.state = Packet::Ref::Halt;
				printf("!! Timeout\n");
				break;
				
			case TimeoutEnd:
				_refState.start = Packet::Ref::None;
				_refState.state = Packet::Ref::Stop;
				printf("!! Timeout End\n");
				break;
		}

		// If the command indicates a team to start, set ourAction.
		if (startTeam != UnknownTeam)
		{
			if (startTeam == _team)
			{
				_refState.ourStart = true;
			}
			else
			{
				_refState.ourStart = false;
			}
		}
		
		printState();
	}
	
	_sender.send(_refState);
}

void Ref::printState()
{
	const char* state = "Halt";
	const char* start = "None";
	
	switch (_refState.state)
	{
		case Packet::Ref::Stop:
			state = "Stop";
			break;
		case Packet::Ref::Setup:
			state = "Setup";
			break;
		case Packet::Ref::OppStart:
			state = "OppStart";
			break;
		case Packet::Ref::Running:
			state = "Running";
			break;
		default:
			break;
	}
	
	switch (_refState.start)
	{
		case Packet::Ref::Kickoff:
			start = "KickOff";
			break;
		case Packet::Ref::Penalty:
			start = "Penalty";
			break;
		case Packet::Ref::Direct:
			start = "Direct";
			break;
		case Packet::Ref::Indirect:
			start = "Indirect";
			break;
		default:
			break;
	}
	
	printf("-- New Cmd: %s", teamToA(_team));
	printf("\tState: %s", state);
	printf("\tStart: %s", start);
	printf("\tOurStart: %d", _refState.ourStart);
	printf("\n");
}

