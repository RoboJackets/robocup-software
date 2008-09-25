#include "Referee.hpp"
#include <math.h>

Referee::Referee() :
	QMainWindow()
{
	setupUi(this);

	//Initialize game variables
	Referee::CurrentPeriod=0;
	Referee::CurrentGameState=0;
	Referee::CurrentMilliseconds=0;
	Referee::MaxMilliseconds=Referee::PreGameMsecs;

	///Initialize goals 
	Referee::BlueGoals=0;
	Referee::YellowGoals=0;

	///Initialize current time out timers
	Referee::CurrentBlueTimeOutTimer=0;
	Referee::CurrentYellowTimeOutTimer=0;

	//Initialize amount of time outs left
	Referee::BlueTimeOuts=4;
	Referee::YellowTimeOuts=4;

	///connect to timer for sending ref packets out
	connect(&_txTimer, SIGNAL(timeout()), SLOT(TxSend()));

	///Connect to the timer that keeps track of the game state.
	connect(&GameTimer, SIGNAL(timeout()), SLOT(GameUpdate()));

	///Connect to the timer that controls blue team's time outs
	connect(&YellowTimeOutTimer, SIGNAL(timeout()), SLOT(YellowTimeOutUpdate()));

	///Connect to the timer that controls yellow team's time outs
	connect(&BlueTimeOutTimer, SIGNAL(timeout()), SLOT(BlueTimeOutUpdate()));

	//setup transmit timer
	_txTimer.start(1000/Referee::Hz);

}

Referee::~Referee()
{
	
}

///This is the function that is called when the game timer times out.  This is executed several times per second, based on the value of Hz.
void Referee::TxSend()
{

}

///This function updates the timers for time outs
void Referee::BlueTimeOutUpdate()
{
	///Check to see if current time exceeds max time possible in current period.
	if(Referee::CurrentBlueTimeOutTimer >= Referee::TimeOutMsecs) {

		///Stop game timer, set the gamestate to halted and increment the game period.
		BlueTimeOutTimer.stop();
		Referee::StartGameTimer();
		///Reset the timer if there are still time outs left.
		if(BlueTimeOuts>=1) {
			CurrentBlueTimeOutTimer=0;
			Referee::UpdateBlueTimeOutRemaining();
			Referee::UpdateBlueTimeOutLabels();
		}
	
	} else {
		///Increment the timer if the game if time is still left.
		Referee::CurrentBlueTimeOutTimer+=100;
		//DEBUG
		//printf("Current milliseconds: %d\n", Referee::CurrentBlueTimeOutTimer);

		///Update our labels.
		Referee::UpdateBlueTimeOutRemaining();
		Referee::UpdateBlueTimeOutLabels();
	}


}

///This function updats the timers for time outs
void Referee::YellowTimeOutUpdate()
{

}


///This function updates the labels of the game timers when the game is being played.
void Referee::GameUpdate()
{
	///Check to see if current time exceeds max time possible in current period.
	if(Referee::CurrentMilliseconds >= Referee::MaxMilliseconds) {

		///Stop game timer, set the gamestate to halted and increment the game period.
		Referee::StopGameTimer();
		Referee::IncrementCurrentPeriod();

	} else {
		///Increment the timer if the game if time is still left.
		Referee::CurrentMilliseconds+=100;
		//DEBUG
		//printf("Current milliseconds: %d\n", Referee::CurrentMilliseconds);

		//Update our labels.
		Referee::UpdateTimeElapsed();
		Referee::UpdateTimeRemaining();
	}

}

///Start of blue buttons code
void Referee::on_BlueGoalButton_clicked()
{

printf("Debug.\n");

}

void Referee::on_MinusBlueButton_clicked()
{

printf("Debug.\n");

}

void Referee::on_BlueTimeOutButton_clicked()
{

	///Make sure blue timer is not active
	if(BlueTimeOutTimer.timerId() == -1) {
		///Make sure there are enough time outs, that another time out clock isn't running, and that the game is playing
		if( (BlueTimeOuts >= 1) && (YellowTimeOutTimer.timerId()==-1) && (CurrentGameState==1)) {
			BlueTimeOutTimer.start(100);
			Referee::StopGameTimer();
			Referee::BlueTimeOuts--;
			Referee::UpdateBlueTimeOutLabels();
		}
	}
}

void Referee::on_BluePenaltyButton_clicked()
{

printf("Debug.\n");

}

void Referee::on_BlueKickOffButton_clicked()
{

printf("Debug.\n");

}

void Referee:: on_BlueFreeKickButton_clicked()
{

printf("Debug.\n");

}

void Referee::on_BlueIndirectButton_clicked()
{

printf("Debug.\n");

}

void Referee::on_BlueYellowCardButton_clicked()
{

printf("Debug.\n");

}

void Referee::on_BlueRedCardButton_clicked()
{

printf("Debug.\n");

}

///Start of yellow buttons code

void Referee::on_YellowGoalButton_clicked()
{

printf("Debug.\n");

}

void Referee::on_MinusYellowButton_clicked()
{

printf("Debug.\n");

}

void Referee::on_YellowTimeOutButton_clicked()
{

printf("Debug.\n");

}

void Referee::on_YellowPenaltyButton_clicked()
{

printf("Debug.\n");

}

void Referee::on_YellowKickOffButton_clicked()
{

printf("Debug.\n");

}

void Referee::on_YellowFreeKickButton_clicked()
{

printf("Debug.\n");

}

void Referee::on_YellowIndirectButton_clicked()
{

printf("Debug.\n");

}

void Referee::on_YellowYellowCardButton_clicked()
{

printf("Debug.\n");

}

void Referee::on_YellowRedCardButton_clicked()
{

printf("Debug.\n");

}


///Start of game buttons code
void Referee::on_StartTimeButton_clicked()
{
	Referee::StartGameTimer();

}

void Referee::on_StopTimeButton_clicked()
{
	Referee::StopGameTimer();

}

void Referee::on_StartGameButton_clicked()
{

printf("Debug.\n");

}

void Referee::on_StopGameButton_clicked()
{

printf("Debug.\n");

}

///This function updates the Time Elapsed button inside the GUI.
void Referee::UpdateTimeElapsed()
{
	//Create a buffer string for our sprintf
	char buf[256];
	
	//Convert milliseconds to minutes, seconds and tenths of seconds
	int milliseconds=Referee::CurrentMilliseconds;

	//Find Minutes
	int minutes=floor(milliseconds/(60*1000));
	milliseconds = milliseconds - (minutes*60*1000);

	//Find seconds
	int seconds=floor(milliseconds/(1000));
	milliseconds = milliseconds - (seconds*1000);

	//Find tenths
	int tenths = floor(milliseconds/(100));

	//Format string for use in label
	if(seconds > 10) {
		snprintf(buf, 100, "%d:%d.%d", minutes, seconds, tenths);
	} else {
		snprintf(buf, 100, "%d:0%d.%d", minutes, seconds, tenths);
	}

	//Put string into label
	TimeElapsedText->setText(buf);

	//DEBUG
	//printf("Time Elapsed: %s\n", buf);
		

}

///This function updates the Time Remaining button inside the GUI.
void Referee::UpdateTimeRemaining()
{
	//Create a buffer string for our sprintf
	char buf[256];
	
	//Convert milliseconds to minutes, seconds and tenths of seconds
	int milliseconds=Referee::MaxMilliseconds - Referee::CurrentMilliseconds;

	//Find Minutes
	int minutes=floor(milliseconds/(60*1000));
	milliseconds = milliseconds - (minutes*60*1000);

	//Find seconds
	int seconds=floor(milliseconds/(1000));
	milliseconds = milliseconds - (seconds*1000);

	//Find tenths
	int tenths = floor(milliseconds/(100));
	
	//Format string for use in label.
	if(seconds > 10) {
		snprintf(buf, 100, "%d:%d.%d", minutes, seconds, tenths);
	} else {
		snprintf(buf, 100, "%d:0%d.%d", minutes, seconds, tenths);
	}

	//Put string into label
	TimeRemainingText->setText(buf);

	//DEBUG
	//printf("Time Remaining: %s\n", buf);

}

void Referee::UpdateBlueTimeOutRemaining()
{
	//Create a buffer string for our sprintf
	char buf[256];
	
	//Convert milliseconds to minutes, seconds and tenths of seconds
	int milliseconds=Referee::TimeOutMsecs - Referee::CurrentBlueTimeOutTimer;

	//Find Minutes
	int minutes=floor(milliseconds/(60*1000));
	milliseconds = milliseconds - (minutes*60*1000);

	//Find seconds
	int seconds=floor(milliseconds/(1000));
	milliseconds = milliseconds - (seconds*1000);

	//Find tenths
	int tenths = floor(milliseconds/(100));
	
	//Format string for use in label.
	if(seconds > 10) {
		snprintf(buf, 100, "%d:%d.%d", minutes, seconds, tenths);
	} else {
		snprintf(buf, 100, "%d:0%d.%d", minutes, seconds, tenths);
	}

	//Put string into label
	TimeRemainingBlueText->setText(buf);

	//DEBUG
	//printf("Time Remaining: %s\n", buf);


}

void Referee::UpdateYellowTimeOutRemaining()
{

}

void Referee::StopGameTimer()
{
	///The timer has AIDS.  Stop it from reproducing.
	GameTimer.stop();
	Referee::CurrentGameState=0;
}

void Referee::StartGameTimer()
{
	if(GameTimer.timerId() == -1) {
		//Start the timer if it hasn't been started yet.
		GameTimer.start(100);
		///MAKE SURE YOU CHANGE THIS!!!!
		CurrentGameState=1;
	} else {
		///Re-start the timer if it has already been started.
		GameTimer.start();
	}
}

void Referee::IncrementBlueGoals()
{

}

void Referee::IncrementYellowGoals()
{

}

void Referee::DecrementBlueGoals()
{

}

void Referee::DecrementYellowGoals()
{

}

void Referee::IncrementCurrentPeriod()
{
	switch(Referee::CurrentPeriod) {
		case 0:
			Referee::CurrentPeriod++;
			Referee::CurrentMilliseconds=0;
			Referee::MaxMilliseconds=FirstHalfMsecs;
			Referee::ResetTimeLabels();
			Referee::UpdatePeriodLabels();
			break;
		case 1:
			Referee::CurrentPeriod++;
			Referee::CurrentMilliseconds=0;
			Referee::MaxMilliseconds=HalfTimeMsecs;
			Referee::ResetTimeLabels();
			Referee::UpdatePeriodLabels();
			break;
		case 2:
			Referee::CurrentPeriod++;
			Referee::CurrentMilliseconds=0;
			Referee::MaxMilliseconds=SecondHalfMsecs;
			Referee::ResetTimeLabels();
			Referee::UpdatePeriodLabels();
			break;
		case 3:
			Referee::CurrentPeriod++;
			Referee::CurrentMilliseconds=0;
			Referee::MaxMilliseconds=FirstOvertimeMsecs;
			Referee::ResetTimeLabels();
			Referee::UpdatePeriodLabels();
			break;
		case 4:
			Referee::CurrentPeriod++;
			Referee::CurrentMilliseconds=0;
			Referee::MaxMilliseconds=SecondOvertimeMsecs;
			Referee::ResetTimeLabels();
			Referee::UpdatePeriodLabels();
			break;
		case 5:
			Referee::CurrentPeriod=0;
			Referee::CurrentMilliseconds=0;
			Referee::MaxMilliseconds=PreGameMsecs;
			Referee::ResetTimeLabels();
			Referee::UpdatePeriodLabels();
			break;
		default:
			Referee::CurrentPeriod=0;
			Referee::CurrentMilliseconds=0;
			Referee::MaxMilliseconds=PreGameMsecs;
			Referee::ResetTimeLabels();
			Referee::UpdatePeriodLabels();
			break;
	}
}

void Referee::ResetTimeLabels()
{
	Referee::UpdateTimeElapsed();
	Referee::UpdateTimeRemaining();
	Referee::UpdatePeriodLabels();
}

void Referee::UpdatePeriodLabels()
{
	switch(Referee::CurrentPeriod) {
		case 0:
			PeriodText->setText("Pre-Game");
			break;
		case 1:
			PeriodText->setText("First Half");
			break;
		case 2:
			PeriodText->setText("Half Time");
			break;
		case 3:
			PeriodText->setText("Second Half");
			break;
		case 4:
			PeriodText->setText("First Overtime");
			break;
		case 5:
			PeriodText->setText("Pre-Game");
			break;
		default:
			PeriodText->setText("Pre-Game");
			break;
	}
}

void Referee::UpdateBlueTimeOutLabels()
{
	///Update blue time out labels.
	char buf[16];
	snprintf(buf, 8, "%d", BlueTimeOuts);
	BlueTimeOutsLeftText->setText(buf);
}

void Referee::UpdateYellowTimeOutLabels()
{

}

