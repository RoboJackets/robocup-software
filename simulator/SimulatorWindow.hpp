#pragma once

#include <QMainWindow>

#include "ui_SimulatorWindow.h"

class Environment;
class RobotTableModel;
class SimRenderView;

class SimulatorWindow: public QMainWindow
{
	Q_OBJECT;

public:
	SimulatorWindow(QWidget *parent = 0);

	void env(Environment *value);

private slots:
	void on_dropFrame_clicked();
	void on_ballVisibility_valueChanged(int value);
	
private:
	Ui_SimulatorWindow _ui;

	Environment *_env;
	RobotTableModel *_model;
	SimRenderView *_render;
};
