#ifndef _GRAB_WINDOW_HPP_
#define _GRAB_WINDOW_HPP_

#include "ui_grab_window.h"

#include <QTimer>

class Receive_Thread;

class Grab_Window: public QDialog
{
    Q_OBJECT;
    
public:
    Grab_Window(QWidget *parent = 0);

    Receive_Thread *rt;

protected Q_SLOTS:
    void on_grab_clicked();
    void on_clear_clicked();
    void on_periodic_toggled(bool on);
    void on_period_valueChanged(int ms);
    void on_save_clicked();

protected:
    Ui_Grab_Window ui;
    QTimer timer;
};

#endif // _GRAB_WINDOW_HPP_
