#ifndef MAINWINDOW_HPP_
#define MAINWINDOW_HPP_

#include <QMainWindow>
#include <QTimer>
#include <QPushButton>

#include <Team.h>

#include <log/FieldView.hpp>
#include <log/LogFile.hpp>
#include <log/LogControl.hpp>
#include <log/TreeView.hpp>
#include <log/TreeModel.hpp>

#include "TeamHandler.hpp"

class MainWindow : public QMainWindow
{
	Q_OBJECT;
	
	public:
		MainWindow(Team t);
		~MainWindow();
		
		void setupModules();
		
	private:
		Team _team;
		
		TeamHandler _handler;
		
		FieldView* _fieldView;
		LogControl* _logControl;
		TreeView* _treeView;
		TreeModel* _treeModel;
		
		LogFile* _logFile;
};

#endif /* MAINWINDOW_HPP_ */
