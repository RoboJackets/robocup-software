// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
#ifndef MAINWINDOW_HPP_
#define MAINWINDOW_HPP_

#include <QMainWindow>
#include <QTimer>
#include <QPushButton>
#include <QString>

#include <Team.h>

#include <log/FieldView.hpp>
#include <log/LogFile.hpp>
#include <log/LogControl.hpp>
#include <log/TreeView.hpp>
#include <log/TreeModel.hpp>
#include <config/ConfigFile.hpp>

#include "Processor.hpp"
#include "ui_MainWindow.h"

class MainWindow : public QMainWindow
{
	Q_OBJECT;
	
	public:
		MainWindow(Team t, QString filename);
		~MainWindow();
	
	protected Q_SLOTS:
		void refHalt();
		void refReady();
		void refStop();
		void refForceStart();
		void refFirstHalf();
		void refOvertime1();
		void refHalftime();
		void refOvertime2();
		void refSecondHalf();
		void refPenaltyShootout();
		void refTimeoutBlue();
		void refTimeoutYellow();
		void refTimeoutEnd();
		void refTimeoutCancel();
		void refKickoffBlue();
		void refKickoffYellow();
		void refDirectBlue();
		void refDirectYellow();
		void refIndirectBlue();
		void refIndirectYellow();
		void refPenaltyBlue();
		void refPenaltyYellow();
		void refGoalBlue();
		void refSubtractGoalBlue();
		void refGoalYellow();
		void refSubtractGoalYellow();
		void refYellowCardBlue();
		void refYellowCardYellow();
		void refRedCardBlue();
		void refRedCardYellow();
	
	protected:
		void refCommand(char cmd);

	private:
		Ui_MainWindow ui;
		
		Team _team;
	
		Processor _processor;
	
		Log::TreeModel* _treeModel;
	
		Log::LogFile* _logFile;
	
		/** Currently the configfile is for motion but others can add to it **/
		QString _configFile;
	
		/** Currently the configfile is for motion but others can add to it **/
		//ConfigFile _config;
};

#endif /* MAINWINDOW_HPP_ */
