#pragma once

#include <QMainWindow>

#include "ui_SimpleSimulatorWindow.h"

class Environment;
class RobotTableModel;
//class SimRenderView;

class SimulatorWindow: public QMainWindow
{
	Q_OBJECT;

public:
	SimulatorWindow(Environment *value, QWidget *parent = 0);

private slots:
	// slots from GUI components
	void on_dropFrame_clicked();
	void on_ballVisibility_valueChanged(int value);
	void on_pushButton_singleFieldSize_clicked();
	void on_pushButton_doubleFieldSize_clicked();
	
public slots:
	// slots to receive information from simulator engine

//	/** add a ball @ pos */
//	void envAddBall(Geometry2d::Point pos);
//
//	/** add a robot with id i to the environment @ pos */
//	void envAddRobot(bool blue, int id, const Geometry2d::Point& pos, Robot::RobotRevision rev);
//
//	/** removes a robot with id i from the environment */
//	void envRemoveRobot(bool blue, int id);

signals:
	// Commands to the simulation engine
	void dropframe();
	void setBallVisibility(int value);

private:
	Ui_SimpleSimulatorWindow _ui;

	RobotTableModel *_model;

	Environment *_environment;

	// Renderer disabled for now
//	SimRenderView *_render;
};
