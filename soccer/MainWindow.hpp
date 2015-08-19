#pragma once

#include <QMainWindow>
#include <QTimer>
#include <QTime>

#include <FieldView.hpp>
#include <Configuration.hpp>

#include "Processor.hpp"
#include "ui_MainWindow.h"

class TestResultTab;
class StripChart;
class ConfigBool;
class QuaternionDemo;

enum RadioChannels
{
    MHz_904,
    MHz_906
};

/**
 * main gui thread class
 */
class MainWindow : public QMainWindow
{
	Q_OBJECT;

	public:

		MainWindow(QWidget *parent = nullptr);

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

		/// Deselects all debug layers
		void allDebugOff();

		/// Selects all debug layers
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

        void setRadioChannel(RadioChannels channel);

        QTimer updateTimer;

    void setUseRefChecked(bool use_ref);

	private Q_SLOTS:
		void addLayer(int i, QString name, bool checked);
		void updateViews();

		void on_fieldView_robotSelected(int shell);
		void on_actionRawBalls_toggled(bool state);
		void on_actionRawRobots_toggled(bool state);
		void on_actionCoords_toggled(bool state);
		void on_actionDotPatterns_toggled(bool state);
    void on_actionTeam_Names_toggled(bool state);
		void on_actionTeamYellow_triggered();
		void on_actionTeamBlue_triggered();
		void on_manualID_currentIndexChanged(int value);
		void on_goalieID_currentIndexChanged(int value);

		void on_actionUse_Field_Oriented_Controls_toggled(bool value);

		void on_actionUse_External_Referee_toggled(bool value);

		/// Field side
		void on_actionDefendPlusX_triggered();
		void on_actionDefendMinusX_triggered();
		void on_actionUseOurHalf_toggled(bool value);
		void on_actionUseOpponentHalf_toggled(bool value);

		/// Field rotation
		void on_action0_triggered();
		void on_action90_triggered();
		void on_action180_triggered();
		void on_action270_triggered();

		/// Radio channels
		void on_action904MHz_triggered();
		void on_action906MHz_triggered();

		/// Vision port
		void on_actionVisionPrimary_Half_triggered();
		void on_actionVisionSecondary_Half_triggered();
		void on_actionVisionFull_Field_triggered();

		/// Simulator commands
		void on_actionCenterBall_triggered();
		void on_actionStopBall_triggered();
		void on_actionResetField_triggered();
		void on_actionStopRobots_triggered();

		/// Manual control commands
		void on_actionDampedRotation_toggled(bool value);
		void on_actionDampedTranslation_toggled(bool value);

		/// Debug menu commands
		void on_actionRestartUpdateTimer_triggered();
		void on_actionQuaternion_Demo_toggled(bool value);

		/// Gameplay menu
		void on_menu_Gameplay_aboutToShow();
		void on_actionSeed_triggered();

		/// Log controls
		void on_logHistoryLocation_sliderMoved(int value);
		void on_logPlaybackRewind_clicked();
		void on_logPlaybackPrevFrame_clicked();
		void on_logPlaybackPause_clicked();
		void on_logPlaybackNextFrame_clicked();
		void on_logPlaybackPlay_clicked();
		void on_logPlaybackLive_clicked();

		/// Debug layers
		void on_debugLayers_itemChanged(QListWidgetItem *item);
		void on_debugLayers_customContextMenuRequested(const QPoint &pos);

		/// Configuration
		void on_configTree_itemChanged(QTreeWidgetItem *item, int column);
		void on_loadConfig_clicked();
		void on_saveConfig_clicked();

		// Playbook
		void on_loadPlaybook_clicked();
		void on_savePlaybook_clicked();

		// Fast Ref Buttons
		void on_fastHalt_clicked();
		void on_fastStop_clicked();
		void on_fastReady_clicked();
		void on_fastForceStart_clicked();
		void on_fastKickoffBlue_clicked();
		void on_fastKickoffYellow_clicked();


	signals:
		//	signal used to let widgets that we're viewing a different log frame now
		int historyLocationChanged(int value);


	private:
		void updateStatus();

		typedef enum
		{
			Status_OK,
			Status_Warning,
			Status_Fail
		} StatusType;

        void status(QString text, StatusType status);
        void channel(int n);

		Ui_MainWindow _ui;

		Processor *_processor;
		Configuration *_config;

		QuaternionDemo *_quaternion_demo;

		// Log history, copied from Logger.
		// This is used by other controls to get log data without having to copy it again from the Logger.
		std::vector<std::shared_ptr<Packet::LogFrame> > _history;

		// When true, External Referee is automatically set.
		// This is cleared by manually changing the checkbox or after the
		// first referee packet is seen and the box is automatically checked.
		bool _autoExternalReferee;

		// Tree items that are not in LogFrame
		QTreeWidgetItem *_frameNumberItem;
		QTreeWidgetItem *_elapsedTimeItem;

		bool _live;

		///	playback rate of the viewer - a value of 1 means realtime
		double _playbackRate;

		// This is used to update some status items less frequently than the full field view
		int _updateCount;

		// Tracking fractional frames is the easiest way to allow arbitrary playback rates.
		// To keep rounding consistent, only access this with frameNumber().
		double _doubleFrameNumber;

		Time _lastUpdateTime;

		QLabel *_currentPlay;
		QLabel *_logFile;
		QLabel *_viewFPS;
		QLabel *_procFPS;
		QLabel *_logMemory;

		//	maps robot shell IDs to items in the list
		std::map<int, QListWidgetItem*> _robotStatusItemMap;

		///	the play, pause, ffwd, etc buttons
		std::vector<QPushButton *> _logPlaybackButtons;

		unsigned long long _firstLogTimestamp = -1;
};
