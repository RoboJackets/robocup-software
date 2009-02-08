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

#include "Processor.hpp"

class MainWindow : public QMainWindow
{
	Q_OBJECT;

	public:
		MainWindow(Team t, QString filename);
		~MainWindow();
		
		void setupModules();
		
	private:
		Team _team;

		Processor _processor;
		
		Log::FieldView* _fieldView;
		Log::LogControl* _logControl;
		Log::TreeView* _treeView;
		Log::TreeModel* _treeModel;

		Log::LogFile* _logFile;

        /** Currently the configfile is for motion but others can add to it **/
        QString _configFile;
};

#endif /* MAINWINDOW_HPP_ */
