#ifndef _TEST_WINDOW_HPP_
#define _TEST_WINDOW_HPP_

#include "ui_test_window.h"

#include <QDialog>

class Test_Table_Model;
class Test_Interface;

class Test_Window: public QDialog
{
    Q_OBJECT;
    
public:
    Test_Window(QWidget *parent = 0);
    ~Test_Window();

protected Q_SLOTS:
    void test_cw();
    void test_ccw();

protected:
    Ui_Test_Window ui;
    
    Test_Table_Model *_model;
    Test_Interface *_interface;
    
    void run_test();
};

#endif // _TEST_WINDOW_HPP_
