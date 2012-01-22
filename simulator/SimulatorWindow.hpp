#pragma once

#include <QMainWindow>

#include "ui_SimulatorWindow.h"

class Env;
class RobotTableModel;

class SimulatorWindow: public QMainWindow
{
	Q_OBJECT;
	
public:
	SimulatorWindow(QWidget *parent = 0);
	
	void env(Env *value);

private slots:
	void on_dropFrame_clicked();
	void on_ballVisibility_valueChanged(int value);
	
private:
	Ui_SimulatorWindow _ui;
	Env *_env;
	RobotTableModel *_model;
};
