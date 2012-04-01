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

  void setRobotVisibility(int visibility);
  void setBallSensorWorks(bool works);
  void setChargerWorks(bool works);

private:
	Ui_SimpleSimulatorWindow _ui;

	RobotTableModel *_model;

	// Renderer disabled for now
//	SimRenderView *_render;
};
