#include "SimulatorWindow.hpp"
//#include "rendering/SimRenderView.hpp"
#include "Physics/Environment.hpp"
#include "RobotTableModel.hpp"

#include <boost/foreach.hpp>

////////////////////////////////////
// SimulatorWindow class
////////////////////////////////////
SimulatorWindow::SimulatorWindow(Environment * env, QWidget* parent):
			QMainWindow(parent)
{
	_ui.setupUi(this);

	// renderer setup
	//	_render = _ui.renderViewWidget;

	//	// connect renderer to simulator
	//	connect(_env, SIGNAL(addNewRobot(bool,int)), _render, SLOT(addRobot(bool,int)));
	//	connect(_env, SIGNAL(removeExistingRobot(bool,int)), _render, SLOT(removeRobot(bool,int)));
	//	connect(_env, SIGNAL(setRobotPose(bool,int,QVector3D,qreal,QVector3D)),
	//			    _render, SLOT(setRobotPose(bool,int,QVector3D,qreal,QVector3D)));

	// set up table
	_model = new RobotTableModel(env);
	_ui.robotTable->setModel(_model);
	_ui.robotTable->resizeColumnsToContents();
}

void SimulatorWindow::on_dropFrame_clicked()
{
	emit dropframe();
}

void SimulatorWindow::on_ballVisibility_valueChanged(int value)
{
	emit setBallVisibility(value);
}
