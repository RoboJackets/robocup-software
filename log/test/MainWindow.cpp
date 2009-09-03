#include "MainWindow.hpp"

#include <QWidget>
#include <QGridLayout>

using namespace Log;

MainWindow::MainWindow(LogFile* logfile)
{
	this->setCentralWidget(new QWidget());
	
	QGridLayout* layout = new QGridLayout();
	this->centralWidget()->setLayout(layout);
	
	_control = new LogControl();
	_control->live(false);
	layout->addWidget(_control, 0, 0);
	
	_view = new FieldView(this); //TODO fixme team
	_view->team(Blue);
	_view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);			
	layout->addWidget(_view, 1, 0);
	
	_tree = new TreeView();
	_tree->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);
	_tree->setModel(&_model);
	layout->addWidget(_tree, 0, 1, 2, 1);
	
	connect(_control, SIGNAL(newFrame(Packet::LogFrame*)), _view, SLOT(frame(Packet::LogFrame*)));
	connect(_control, SIGNAL(newFrame(Packet::LogFrame*)), &_model, SLOT(frame(Packet::LogFrame*)));
	
	_control->setLogFile(logfile);
	
	_logModule = new LogModule();
	
	_view->addModule(_logModule);
}
