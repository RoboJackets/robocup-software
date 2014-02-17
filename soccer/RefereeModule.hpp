#pragma once

#include <Geometry2d/Point.hpp>

#include <QTime>
#include <QMutex>
#include <QMutexLocker>
#include <QUdpSocket>
#include <Network.hpp>

namespace RefereeCommands
{
    ////////////////////////
    /// General gameplay
    
    /// All robots must stop moving immediately.
    static const char Halt = 'H';
    
    /// Stops gameplay.  Robots may still move, but must stay 500mm away from ball.
    static const char Stop = 'S';
    
    /// Switches to normal gameplay.  May follow a Stop or restart command.
    static const char ForceStart = 's';
    
    /// Allows a robot on the starting team to kick the ball.
    static const char Ready = ' ';
    
    ////////////////////////
    /// Timing
    
    static const char FirstHalf = '1';
    static const char Halftime = 'h';
    static const char SecondHalf = '2';
    static const char Overtime1 = 'o';
    static const char Overtime2 = 'O';
    static const char PenaltyShootout = 'a';
    
    ////////////////////////
    /// Timeouts
    ///
    /// A timeout can end with Stop (ended by team) or TimeoutEnd (out of time).
    
    static const char TimeoutYellow = 't';
    static const char TimeoutBlue = 'T';
    
    /// Ran out of time in the timeout.
    static const char TimeoutEnd = 'z';
    
    /// Ends the current timeout and resets the timeout time to its value at the beginning of the timeout.
    static const char TimeoutCancel = 'c';
    
    ////////////////////////
    /// Goals scored
    
    static const char GoalYellow = 'g';
    static const char GoalBlue = 'G';
    
    static const char SubtractGoalYellow = 'd';
    static const char SubtractGoalBlue = 'D';
    
    ////////////////////////
    /// Yellow/red cards
    
    static const char YellowCardYellow = 'y';
    static const char YellowCardBlue = 'Y';
    
    static const char RedCardYellow = 'r';
    static const char RedCardBlue = 'R';
    
    ////////////////////////
    /// Restarts
    
    static const char KickoffYellow = 'k';
    static const char KickoffBlue = 'K';
    
    static const char PenaltyYellow = 'p';
    static const char PenaltyBlue = 'P';
    
    static const char DirectYellow = 'f';
    static const char DirectBlue = 'F';

    static const char IndirectYellow = 'i';
    static const char IndirectBlue = 'I';
};

class SystemState;
/**
 * this is the referee
 */
class RefereeModule
{
	public:
		RefereeModule(SystemState *state);

		~RefereeModule();
		
		/// Called periodically.  Checks vision data for ball movement.
		void run();
		
		/// Handles a packet from the referee.
		void packet(const std::string &data);
		
		/// Handles the command part of a packet.
		/// This is used by the GUI to simulate referee packets.
		void command(char command);

		/// True if the ball has been kicked since the last restart began
		bool kicked() const
		{
			return _kickDetectState == Kicked;
		}
		
		void blueTeam(bool value)
		{
			QMutexLocker locker(&_mutex);
			_blueTeam = value;
		}

		time_t lastPacketTime()
		{
			return _lastPacketTime_t;
		}
		
		QString lastPacketDescription();

		bool UseExternalReferee;
		
	protected:
		QMutex _mutex;
		SystemState *_state;
		bool _blueTeam;
		
		std::string _lastPacket;
		QTime _lastPacketTime;
		/**
		 * @parm cmd the character of the command that the referee issues
		 */
		void refCommand(char cmd);
		
		/// Moves to the ready state and sets up the ball kick detector
		void ready();
		
		/// Last counter value received or -1 if no packets have been processed
		int _counter;
		
		typedef enum
		{
			WaitForReady,
			CapturePosition,
			WaitForKick,
			VerifyKick,
			Kicked
		} KickDetectState;
		KickDetectState _kickDetectState;
		
		Geometry2d::Point _readyBallPos;
		
		// Time the ball was first beyond KickThreshold from its original position
		QTime _kickTime;

	private:
		QUdpSocket *_refereeSocket;
		time_t _lastPacketTime_t;
};
