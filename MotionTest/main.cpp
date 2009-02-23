#include <QApplication>
#include <Team.h>
#include "MainWindow.hpp"
#include <QString>

void usage(const char* prog)
{
    printf("usage: %s <-y|-b> [-c <config file>]\n", prog);
    printf("\t-y: run as the yellow team\n");
    printf("\t-b: run as the blue team\n");
}


int main(int argc,char* argv[])
{
    QApplication app(argc, argv);

    Team t = Yellow;
    QString cfgFile = "";

    for (int i=0 ; i<argc; ++i)
    {
        const char* var = argv[i];

        if (strcmp(var, "-y") == 0)
        {
            t = Yellow;
        }
        else if (strcmp(var, "-b") == 0)
        {
            t = Blue;
        }
        else if(strcmp(var, "-c") == 0)
        {
            cfgFile = argv[i+1];
        }
    }

    if(cfgFile == "")
    {
        usage("MotionTest");
        return 0;
    }

    MainWindow win(t,cfgFile);
    win.show();

    return app.exec();
}
