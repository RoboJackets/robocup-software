// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

#include <QMainWindow>
#include <QTimer>
#include <QTime>

#include <FieldView.hpp>

#include "Processor.hpp"
#include "ui_MainWindow.h"

class PlayConfigTab;
class TestResultTab;
class StripChart;
class ConfigBool;

class MainWindow : public QMainWindow
{
	Q_OBJECT;
	
	public:
		MainWindow(QWidget *parent = 0);
		
		void configuration(Configuration *config);
		
		void processor(Processor *value);
		
		Processor *processor()
		{
			return _processor;
		}
		
		SystemState *state()
		{
			return _processor->state();
		}
		
		PlayConfigTab *playConfigTab() const
		{
			return _playConfigTab;
		}
		
		TestResultTab *testResultTab() const
		{
			return _testResultTab;
		}

		// Deselects all debug layers
		void allDebugOff();
		
		// Selects all debug layers
		void allDebugOn();
		
		void live(bool value);
		
		int frameNumber() const
		{
			return roundf(_doubleFrameNumber);
		}
		
		void frameNumber(int value)
		{
			_doubleFrameNumber = value;
		}
		
		// Call this to update the status bar when the log file has changed
		void logFileChanged();
		
		QTimer updateTimer;
		
	private Q_SLOTS:
		void updateViews();
		
		void on_fieldView_robotSelected(int shell);
		void on_actionRawBalls_toggled(bool state);
		void on_actionRawRobots_toggled(bool state);
		void on_actionCoords_toggled(bool state);
		void on_actionTeamYellow_triggered();
		void on_actionTeamBlue_triggered();
		void on_manualID_currentIndexChanged(int value);
		
		// Field side
		void on_actionDefendPlusX_triggered();
		void on_actionDefendMinusX_triggered();
		void on_actionUseOurHalf_toggled(bool value);
		void on_actionUseOpponentHalf_toggled(bool value);
		
		// Field rotation
		void on_action0_triggered();
		void on_action90_triggered();
		void on_action180_triggered();
		void on_action270_triggered();
		
		// Simulator commands
		void on_actionCenterBall_triggered();
		void on_actionStopBall_triggered();
		void on_actionResetField_triggered();
		void on_actionStopRobots_triggered();
		
		// Debug menu commands
		void on_actionRestartUpdateTimer_triggered();
		
		// Gameplay menu
		void on_menu_Gameplay_aboutToShow();
		void on_actionSeed_triggered();
		
		// Log controls
		void on_playbackRate_sliderPressed();
		void on_playbackRate_sliderMoved(int value);
		void on_playbackRate_sliderReleased();
		void on_logStop_clicked();
		void on_logFirst_clicked();
		void on_logPrev_clicked();
		void on_logNext_clicked();
		void on_logLive_clicked();
		
		// Debug layers
		void on_debugLayers_itemChanged(QListWidgetItem *item);
		void on_debugLayers_customContextMenuRequested(const QPoint &pos);
		
		// Referee
		void on_externalReferee_toggled(bool value);
		void on_externalReferee_clicked(bool value);
		
		void on_refHalt_clicked();
		void on_refStop_clicked();
		void on_refReady_clicked();
		void on_refForceStart_clicked();
		void on_refKickoffBlue_clicked();
		void on_refKickoffYellow_clicked();
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
		
		// Configuration
		void on_configTree_itemChanged(QTreeWidgetItem *item, int column);
		void on_loadConfig_clicked();
		void on_saveConfig_clicked();
		
	private:
		void updateStatus();
		
		typedef enum
		{
			Status_OK,
			Status_Warning,
			Status_Fail
		} StatusType;
		
		void status(QString text, StatusType status);
		
		// Sets revision-dependent robot configuration
		void updateRobotConfigs();
		
		Ui_MainWindow _ui;
		
		Processor *_processor;
		Configuration *_config;
		
		// Log history, copied from Logger.
		// This is used by other controls to get log data without having to copy it again from the Logger.
		std::vector<boost::shared_ptr<Packet::LogFrame> > _history;
		
		// When true, External Referee is automatically set.
		// This is cleared by manually changing the checkbox or after the
		// first referee packet is seen and the box is automatically checked.
		bool _autoExternalReferee;
		
		PlayConfigTab *_playConfigTab;
		
		TestResultTab *_testResultTab;

		// Tree items that are not in LogFrame
		QTreeWidgetItem *_frameNumberItem;
		QTreeWidgetItem *_elapsedTimeItem;
		
		bool _live;
		
		RobotConfig *_robotConfig2008;
		RobotConfig *_robotConfig2011;
		ConfigBool *_robot2011[Num_Shells];
		
		// This is used to update some status items less frequently than the full field view
		int _updateCount;
		
		// Tracking fractional frames is the easiest way to allow arbitrary playback rates.
		// To keep rounding consistent, only access this with frameNumber().
		double _doubleFrameNumber;
		
		uint64_t _lastUpdateTime;
		
		QLabel *_currentPlay;
		QLabel *_logFile;
		QLabel *_viewFPS;
		QLabel *_procFPS;
		QLabel *_logMemory;
		QLabel *_refereeLabel;

		// True after the processor has chosen a radio channel and the label has been updated.
		bool _haveRadioChannel;
};
