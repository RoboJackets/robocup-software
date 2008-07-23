#include "Radio.hpp"
#include "Robot_Table.hpp"
#include "Command_Thread.hpp"
#include "Status_Thread.hpp"
#include "Cal_Window.hpp"
#include "main.hpp"

#include <Team.h>

#include <QApplication>
#include <QTableView>
#include <QHeaderView>
#include <QMainWindow>
#include <QVBoxLayout>

bool debug = false;

void usage()
{
	printf("Usage: radio [-b|-y] <serial device>\n");
	exit(1);
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    
    Team team = UnknownTeam;
    char *device = 0;
    bool cal = false;
    for (int i = 1; i < argc; ++i)
    {
    	if (argv[i][0] == '-')
    	{
    		if (!strcmp(argv[i], "-b"))
    		{
    			team = Blue;
    		} else if (!strcmp(argv[i], "-y"))
    		{
    			team = Yellow;
    		} else if (!strcmp(argv[i], "-cal"))
            {
                cal = true;
            } else if (!strcmp(argv[i], "-debug"))
            {
                debug = true;
            } else {
                usage();
            }
    	} else if (!device)
    	{
    		device = argv[i];
    	} else {
    		usage();
    	}
    }
    
    if (!device || team == UnknownTeam)
    {
    	usage();
    }
    
    Radio *radio = new Radio(device);
    radio->command_mode(false);
    radio->command_mode(true);
    radio->set_channels(45, 70);
    radio->command_mode(false);
    
    Command_Thread command_thread(radio, team);
    command_thread.start();
    
    Status_Thread status_thread(&command_thread, team);
    status_thread.start();
    
    QWidget *win = new QWidget();
    QVBoxLayout *layout = new QVBoxLayout(win);
    win->setLayout(layout);
    
    QTableView *view = new QTableView(win);
    Robot_Table_Model *model = new Robot_Table_Model(&command_thread);
    view->setModel(model);
    layout->addWidget(view);
    win->show();
    win->resize(450, 300);
    
    if (cal)
    {
        Cal_Window *cw = new Cal_Window();
        cw->command_thread(&command_thread);
        cw->show();
        cw->move(100, 100);
    }
    
    app.exec();
    
    printf("Shutdown\n");
    
    command_thread.stop();
    status_thread.stop();
    
    delete radio;
    
    return 0;
}
