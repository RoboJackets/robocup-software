#pragma once

#include <ui_MainRef.h>
#include <QTimer>
#include "GameControl.hpp"

class Referee : public QMainWindow, Ui_MainWindow
{
	Q_OBJECT;

	public:
		Referee(GameControl &gc);
		~Referee();


		/** Variables that describe the game state
		*
		*	0 - Pregame
		*	1 - First Half
		*	2 - Half Time
		*	3 - Second Half
		*	4 - First Overtime
		*	5 - Second Overtime */
		//int currPeriod;
		
		typedef enum
		{
			PreGame,
			FirstHalf,
			HalfTime,
			SecondHalf,
			FirstOvertime,
			SecondOvertime
		} Period;
		
		/** Current game period */
		Period currPeriod;

		/** Current milliseconds */
		int currMilliseconds;

		/** Maximum milliseconds in current period of game play */
		int maxMilliseconds;

		/** Current game state (0 - Halted, 1 - In progress, 2 - timeout) */
		int currGameState;		
		
		/** Variables to hold the amount of goals */
		int blueGoals;
		int yellowGoals;

		/** Variables to hold timeouts for both teams */
		int yellowTimeOuts;
		int blueTimeOuts;

		/** Variable to hold current time out time */
		int currBlueTimeOutTimer;
		int currYellowTimeOutTimer;
	
		
	protected:
		/** Non GUI Elements */
		GameControl gameControl;


	private Q_SLOTS:
		/** send latest ref info */
		void txSend();
		
		/** Send game update info */
		void gameUpdate();

		/** Update timers */
		void blueTimeOutUpdate();
		void yellowTimeOutUpdate();
		
		/** slot for idle processing */
		void idle();
		
		///Generate functions for all our button presses///

		/** Blue team's buttons */
		void on_BlueGoalButton_clicked();
		void on_MinusBlueButton_clicked();
		void on_BlueTimeOutButton_clicked();
		void on_BluePenaltyButton_clicked();
		void on_BlueKickOffButton_clicked();
		void on_BlueFreeKickButton_clicked();
		void on_BlueIndirectButton_clicked();
		void on_BlueYellowCardButton_clicked();
		void on_BlueRedCardButton_clicked();

		/** Yellow team's buttons */
		void on_YellowGoalButton_clicked();
		void on_MinusYellowButton_clicked();
		void on_YellowTimeOutButton_clicked();
		void on_YellowPenaltyButton_clicked();
		void on_YellowKickOffButton_clicked();
		void on_YellowFreeKickButton_clicked();
		void on_YellowIndirectButton_clicked();
		void on_YellowYellowCardButton_clicked();
		void on_YellowRedCardButton_clicked();

		/** Game buttons */
		void on_StartTimeButton_clicked();
		void on_StopTimeButton_clicked();
		void on_StartGameButton_clicked();
		void on_StopGameButton_clicked();
		void on_CancelButton_clicked();

		/** Functions to update handling the game timer */
		void updateTimeElapsed();
		void updateTimeRemaining();

		/** Functions to start and stop game timer */
		void startGameTimer();
		void stopGameTimer();

		/** Functions to change goals */
		void incrementBlueGoals();
		void incrementYellowGoals();
		void decrementBlueGoals();
		void decrementYellowGoals();

		/** Functions to increment current period */
		void incrementCurrentPeriod();

		/** Function to reset game time labels */
		void resetTimeLabels();

		/** Function to change the label of what period it is */
		void updatePeriodLabels();

		/** Function to update the time out time labels */
		void updateBlueTimeOutRemaining();
		void updateYellowTimeOutRemaining();

		/** Function to update times out left label */
		void updateBlueTimeOutLabels();
		void updateYellowTimeOutLabels();

	private:
		QTimer _txTimer;
		QTimer _gameTimer;
		QTimer _blueTimeOutTimer;
		QTimer _yellowTimeOutTimer;
		QTimer _idleTimer;
		
		/** function */
		void setActiveWidgets(const EnableState &es);

		/** Number of referee packets to transmit in 1 second */
		static const int _hz = 10;

		/** Number of game updates to happen in 1 second */
		static const int _gameHz = 10;

		/** Number of MILLISECONDS in pregame interval */
		static const int _preGameMsecs=5*60*1000;
		
		/** Number of MILLISECONDs in first half */
		static const int _firstHalfMsecs=15*60*1000;

		/** Number of MILLISECONDS in second half */
		static const int _secondHalfMsecs=15*60*1000;

		/** Number of MILLISECONDS in half time */
		static const int _halfTimeMsecs=5*60*1000;

		/** Number of MILLISECONDS in first overtime */
		static const int _firstOvertimeMsecs=3*60*1000;

		/** Number of MILLISECONDS in second overtime */
		static const int _secondOvertimeMsecs=3*60*1000;

		/** Number of MILLISECONDS in each time out */
		static const int _timeOutMsecs=10*60*1000;

};
