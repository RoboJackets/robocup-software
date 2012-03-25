#pragma once

#include <QMainWindow>

#include "ui_SimpleSimulatorWindow.h"

class Environment;
class RobotTableModel;
class SimRenderView;

class SimulatorWindow: public QMainWindow
{
	Q_OBJECT;

public:
	SimulatorWindow(Environment *value, QWidget *parent = 0);

private slots:
	void on_dropFrame_clicked();
	void on_ballVisibility_valueChanged(int value);
	
private:
	Ui_SimpleSimulatorWindow _ui;

	Environment *_env;
	RobotTableModel *_model;
//	SimRenderView *_render;
};
