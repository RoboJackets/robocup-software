#include "Test_Window.hpp"
#include "Test_Table_Model.hpp"
#include "Test_Interface.hpp"

Test_Window::Test_Window(QWidget *parent): QDialog(parent)
{
    ui.setupUi(this);
    
    _interface = new Test_Interface();
    _model = new Test_Table_Model(_interface);
    ui.table->setModel(_model);
    
    connect(ui.test_cw, SIGNAL(clicked()), SLOT(test_cw()));
    connect(ui.test_ccw, SIGNAL(clicked()), SLOT(test_ccw()));
}

Test_Window::~Test_Window()
{
    delete _model;
    delete _interface;
}

void Test_Window::test_cw()
{
    run_test();
    _model->ccw = false;
    _model->refresh();
}

void Test_Window::test_ccw()
{
    run_test();
    _model->ccw = true;
    _model->refresh();
}

void Test_Window::run_test()
{
    for (int i = 0; i < 8; ++i)
    {
        _interface->hall(i);
        _interface->motors(_model->motor_level[i], _model->motor_raw[i]);
    }
}
