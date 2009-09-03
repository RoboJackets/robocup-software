#pragma once

#include "../LogControl.hpp"
#include "../FieldView.hpp"
#include "../TreeView.hpp"
#include "../TreeModel.hpp"
#include "../LogModule.hpp"

#include <QMainWindow>

class MainWindow : public QMainWindow
{
	public:
		MainWindow(Log::LogFile* logfile = 0);
		
	private:
		Log::LogControl* _control;
		Log::FieldView* _view;
		Log::TreeView* _tree;
		
		Log::LogModule* _logModule;
		Log::TreeModel _model;
};

