#ifndef REFEREE_HPP
#define REFEREE_HPP

#include <ui_MainRef.h>
#include <QTimer>

class Referee : public QMainWindow, Ui_MainWindow
{
	Q_OBJECT;

	public:
		Referee();
		~Referee();


		///Variables that describe the game state

		///0 - Pregame
		///1 - First Half
		///2 - Half Time
		///3 - Second Half
		///4 - First Overtime
		///5 - Second Overtime
		int CurrentPeriod;

		///Current milliseconds
		int CurrentMilliseconds;

		///Maximum milliseconds in current period of game play
		int MaxMilliseconds;

		///Current game state (0 - Halted, 1 - In progress, 2 - timeout)
		int CurrentGameState;		
		
		///Variables to hold the amount of goals
		int BlueGoals;
		int YellowGoals;

		///Variables to hold timeouts for both teams
		int YellowTimeOuts;
		int BlueTimeOuts;

		///Variable to hold current time out time
		int CurrentBlueTimeOutTimer;
		int CurrentYellowTimeOutTimer;



	private Q_SLOTS:
		/** send latest ref info */
		void TxSend();
		
		///Send game update info
		void GameUpdate();

		///Update timers
		void BlueTimeOutUpdate();
		void YellowTimeOutUpdate();
		
		///Generate functions for all our button presses.

		///Blue team's buttons
		void on_BlueGoalButton_clicked();
		void on_MinusBlueButton_clicked();
		void on_BlueTimeOutButton_clicked();
		void on_BluePenaltyButton_clicked();
		void on_BlueKickOffButton_clicked();
		void on_BlueFreeKickButton_clicked();
		void on_BlueIndirectButton_clicked();
		void on_BlueYellowCardButton_clicked();
		void on_BlueRedCardButton_clicked();

		///Yellow team's buttons
		void on_YellowGoalButton_clicked();
		void on_MinusYellowButton_clicked();
		void on_YellowTimeOutButton_clicked();
		void on_YellowPenaltyButton_clicked();
		void on_YellowKickOffButton_clicked();
		void on_YellowFreeKickButton_clicked();
		void on_YellowIndirectButton_clicked();
		void on_YellowYellowCardButton_clicked();
		void on_YellowRedCardButton_clicked();

		///Game buttons
		void on_StartTimeButton_clicked();
		void on_StopTimeButton_clicked();
		void on_StartGameButton_clicked();
		void on_StopGameButton_clicked();

		///Functions to update handling the game timer
		void UpdateTimeElapsed();
		void UpdateTimeRemaining();

		///Functions to start and stop game timer.
		void StartGameTimer();
		void StopGameTimer();

		///Functions to change goals
		void IncrementBlueGoals();
		void IncrementYellowGoals();
		void DecrementBlueGoals();
		void DecrementYellowGoals();

		//Functions to increment current period
		void IncrementCurrentPeriod();

		///Function to reset game time labels
		void ResetTimeLabels();

		///Function to change the label of what period it is.
		void UpdatePeriodLabels();

		///Function to update the time out time labels
		void UpdateBlueTimeOutRemaining();
		void UpdateYellowTimeOutRemaining();

		///Function to update times out left label
		void UpdateBlueTimeOutLabels();
		void UpdateYellowTimeOutLabels();

	private:
		QTimer _txTimer;
		QTimer GameTimer;
		QTimer BlueTimeOutTimer;
		QTimer YellowTimeOutTimer;

		///Number of referee packets to transmit in 1 second
		static const int Hz = 10;

		//Number of game updates to happen in 1 second
		static const int GameHz = 10;

		///Number of MILLISECONDS in pregame interval
		static const int PreGameMsecs=5*60*1000;
		
		///Number of MILLISECONDs in first half
		static const int FirstHalfMsecs=15*60*1000;

		///Number of MILLISECONDS in second half
		static const int SecondHalfMsecs=15*60*1000;

		///Number of MILLISECONDS in half time
		static const int HalfTimeMsecs=5*60*1000;

		///Number of MILLISECONDS in first overtime
		static const int FirstOvertimeMsecs=3*60*1000;

		///Number of MILLISECONDS in second overtime
		static const int SecondOvertimeMsecs=3*60*1000;

		///Number of MILLISECONDS in each time out
		static const int TimeOutMsecs=10*60*1000;

	


};


#endif // REFEREE_HPP
