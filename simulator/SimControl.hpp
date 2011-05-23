#pragma once

#include <QMainWindow>

#include "ui_SimControl.h"

class Env;
class RobotTableModel;

class SimControl: public QMainWindow
{
	Q_OBJECT;
	
public:
	SimControl(QWidget *parent = 0);
	
	void env(Env *value);

private slots:
	void on_dropFrame_clicked();
	void on_ballVisibility_valueChanged(int value);
	
private:
	Ui_SimControl _ui;
	Env *_env;
	RobotTableModel *_model;
};
