#pragma once

#include <QMainWindow>

#include "ui_SimpleSimulatorWindow.h"


class Environment;
class RobotTableModel;

class SimulatorWindow: public QMainWindow
{
	Q_OBJECT;

public:
	SimulatorWindow(Environment *value, QWidget *parent = nullptr);

private slots:
	// slots from GUI components
	void on_dropFrame_clicked();
	void on_ballVisibility_valueChanged(int value);
	void on_pushButton_singleFieldSize_clicked();
	void on_pushButton_doubleFieldSize_clicked();


signals:
	// Commands to the simulation engine
	void dropframe();
	void setBallVisibility(int value);

private:
	Ui_SimpleSimulatorWindow _ui;

	RobotTableModel *_model;

	Environment *_environment;
};
