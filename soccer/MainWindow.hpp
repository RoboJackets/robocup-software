// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

#include <QMainWindow>
#include <QTimer>

#include <Team.h>

#include <log/FieldView.hpp>
#include <log/TreeView.hpp>
#include <log/TreeModel.hpp>

#include <configuration/ConfigFileTab.hpp>

#include "Processor.hpp"
#include "ui_MainWindow.h"

class PlayConfigTab;

class MainWindow : public QMainWindow
{
	Q_OBJECT;
	
	public:
		MainWindow(Team t, QString filename, bool sim);
		~MainWindow();
		
		Processor *processor()
		{
			return &_processor;
		}
		
		PlayConfigTab *playConfig() const;
	
	public Q_SLOTS:
		void updateTree();
		void flipField(bool value);
		
	private:
		virtual bool event(QEvent *e);
		
		Ui_MainWindow ui;
		QCheckBox* _flipBox;

		Team _team;
	
		Processor _processor;

		Log::TreeModel* _treeModel;
	
		QString _configFile;

		QTimer _treeTimer;
		QTimer _fieldTimer;
		ConfigFileTab* _configFileTab;
		Packet::LogFrame _viewFrame;
};
