#ifndef _CAL_WINDOW_HPP_
#define _CAL_WINDOW_HPP_

#include "ui_cal_window.h"

class Command_Thread;

class Cal_Window: public QDialog
{
	Q_OBJECT;
public:
	Cal_Window(QWidget *parent = 0);

	void command_thread(Command_Thread *ct) { _command_thread = ct; }
    
protected Q_SLOTS:
    void on_send_clicked();

protected:
	Ui_Cal_Window ui;
    Command_Thread *_command_thread;
};

#endif // _CAL_WINDOW_HPP_
