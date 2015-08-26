#include "SimulatorWindow.hpp"
//#include "rendering/SimRenderView.hpp"
#include "physics/Environment.hpp"
#include "RobotTableModel.hpp"
#include "physics/PhysicsConstants.hpp"

////////////////////////////////////
// SimulatorWindow class
////////////////////////////////////
SimulatorWindow::SimulatorWindow(Environment* env, QWidget* parent)
    : QMainWindow(parent), _environment(env) {
    _ui.setupUi(this);

    // set up table
    _model = new RobotTableModel(env);
    _ui.robotTable->setModel(_model);
    _ui.robotTable->resizeColumnsToContents();
}

void SimulatorWindow::on_dropFrame_clicked() { emit dropframe(); }

void SimulatorWindow::on_ballVisibility_valueChanged(int value) {
    emit setBallVisibility(value);
}

void SimulatorWindow::on_pushButton_singleFieldSize_clicked() {
    Field_Dimensions::Current_Dimensions =
        Field_Dimensions::Single_Field_Dimensions * scaling;
    _environment->reshapeFieldBodies();
}

void SimulatorWindow::on_pushButton_doubleFieldSize_clicked() {
    Field_Dimensions::Current_Dimensions =
        Field_Dimensions::Double_Field_Dimensions * scaling;
    _environment->reshapeFieldBodies();
}
