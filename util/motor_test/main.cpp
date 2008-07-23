#include <QApplication>

#include "Test_Window.hpp"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    
    Test_Window *win = new Test_Window();
    win->show();
    
    app.exec();
    
    return 0;
}
