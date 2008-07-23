#include "Cal_Window.hpp"
#include "Command_Thread.hpp"

Cal_Window::Cal_Window(QWidget *parent): QDialog(parent)
{
    ui.setupUi(this);
}

void Cal_Window::on_send_clicked()
{
    _command_thread->send_cal(ui.robot->value(), ui.coeff_p->value(), ui.coeff_d->value());
}
