#pragma once

#include <framework/Module.hpp>
#include <framework/SystemState.hpp>
#include <Team.h>
#include <Referee.hpp>
#include <QTime>

#include "ui_RefereeTab.h"

namespace RefereeCommands
{
    ////////////////////////
    // General gameplay
    
    // All robots must stop moving immediately.
    static const char Halt = 'H';
    
    // Stops gameplay.  Robots may still move, but must stay 500mm away from ball.
    static const char Stop = 'S';
    
    // Switches to normal gameplay.  May follow a Stop or restart command.
    static const char ForceStart = 's';
    
    // Allows a robot on the starting team to kick the ball.
    static const char Ready = ' ';
    
    ////////////////////////
    // Timing
    
    static const char FirstHalf = '1';
    static const char Halftime = 'h';
    static const char SecondHalf = '2';
    static const char Overtime1 = 'o';
    static const char Overtime2 = 'O';
    static const char PenaltyShootout = 'a';
    
    ////////////////////////
    // Timeouts
    //
    // A timeout can end with Stop (ended by team) or TimeoutEnd (out of time).
    
    static const char TimeoutYellow = 't';
    static const char TimeoutBlue = 'T';
    
    // Ran out of time in the timeout.
    static const char TimeoutEnd = 'z';
    
    // Ends the current timeout and resets the timeout time to its value at the beginning of the timeout.
    static const char TimeoutCancel = 'c';
    
    ////////////////////////
    // Goals scored
    
    static const char GoalYellow = 'g';
    static const char GoalBlue = 'G';
    
    static const char SubtractGoalYellow = 'd';
    static const char SubtractGoalBlue = 'D';
    
    ////////////////////////
    // Yellow/red cards
    
    static const char YellowCardYellow = 'y';
    static const char YellowCardBlue = 'Y';
    
    static const char RedCardYellow = 'r';
    static const char RedCardBlue = 'R';
    
    ////////////////////////
    // Restarts
    
    static const char KickoffYellow = 'k';
    static const char KickoffBlue = 'K';
    
    static const char PenaltyYellow = 'p';
    static const char PenaltyBlue = 'P';
    
    static const char DirectYellow = 'f';
    static const char DirectBlue = 'F';

    static const char IndirectYellow = 'i';
    static const char IndirectBlue = 'I';
};

extern const char *RefereeAddress;
static const int RefereePort = 10001;

class RefereeModule: public QObject, public Module
{
    Q_OBJECT;
    
public:
    RefereeModule(SystemState *state);
    
    // Called periodically.  Checks vision data for ball movement.
    void run();
    
    // Handles a packet from the referee.
    // If force == true, the packet's counter value is ignored.
    void packet(const Packet::Referee *data);
    void command(uint8_t command);

    // True if the ball has been kicked since the last restart began
    bool kicked() const
    {
        return _kickDetectState == Kicked;
    }

protected Q_SLOTS:
    void on_actionHalt_triggered();
    void on_actionStop_triggered();
    void on_actionReady_triggered();
    void on_actionForceStart_triggered();
    void on_actionKickoffBlue_triggered();
    void on_actionKickoffYellow_triggered();
    
    void on_refFirstHalf_clicked();
    void on_refOvertime1_clicked();
    void on_refHalftime_clicked();
    void on_refOvertime2_clicked();
    void on_refSecondHalf_clicked();
    void on_refPenaltyShootout_clicked();
    void on_refTimeoutBlue_clicked();
    void on_refTimeoutYellow_clicked();
    void on_refTimeoutEnd_clicked();
    void on_refTimeoutCancel_clicked();
    void on_refDirectBlue_clicked();
    void on_refDirectYellow_clicked();
    void on_refIndirectBlue_clicked();
    void on_refIndirectYellow_clicked();
    void on_refPenaltyBlue_clicked();
    void on_refPenaltyYellow_clicked();
    void on_refGoalBlue_clicked();
    void on_refSubtractGoalBlue_clicked();
    void on_refGoalYellow_clicked();
    void on_refSubtractGoalYellow_clicked();
    void on_refYellowCardBlue_clicked();
    void on_refYellowCardYellow_clicked();
    void on_refRedCardBlue_clicked();
    void on_refRedCardYellow_clicked();
	void on_externalReferee_toggled(bool value);

protected:
    Ui_Referee ui;
	SystemState *_state;
    
    void refCommand(char cmd);
    
    // Moves to the ready state and sets up the ball kick detector
    void ready();
    
    // Last counter value received or -1 if no packets have been processed
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
	bool _useExternal;
    
    // Time the ball was first beyond KickThreshold from its original position
    QTime _kickTime;
};
