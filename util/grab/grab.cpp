#include "Grab_Window.hpp"
#include "Receive_Thread.hpp"

#include <stdio.h>

#include <QApplication>

#include <Packet/PacketReceiver.hpp>
#include <Packet/VisionData.hpp>

void usage()
{
    printf("Usage: grab [-y|-b]\n");
    exit(1);
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        usage();
    }
    
    Team team;
    if (!strcmp(argv[1], "-y"))
    {
        team = Yellow;
    } else if (!strcmp(argv[1], "-b"))
    {
        team = Blue;
    } else {
        usage();
    }
    
    QApplication app(argc, argv);
    
    Receive_Thread rt;
    rt.team = team;
    rt.start();
    
    Grab_Window *win = new Grab_Window();
    win->show();
    win->rt = &rt;
    
    app.exec();
    rt.stop();
    
    return 0;
}
