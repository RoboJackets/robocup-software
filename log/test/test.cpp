#include <stdio.h>
#include <QApplication>

#include "../LogControl.cpp"
#include "../FieldView.cpp"
#include "../TreeView.hpp"
#include "../TreeModel.hpp"

#include <LogFile.hpp>

#include <QMainWindow>
#include <QWidget>
#include <QGridLayout>

class MainWindow : public QMainWindow
{
	public:
		MainWindow(LogFile* logfile = 0)
		{
			this->setCentralWidget(new QWidget());
			
			QGridLayout* layout = new QGridLayout();
			this->centralWidget()->setLayout(layout);
			
			_control = new LogControl();
			_control->live(false);
			layout->addWidget(_control, 0, 0);
			
			_view = new FieldView(Blue); //TODO fixme team
			_view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);			
			layout->addWidget(_view, 1, 0);
			
			_tree = new TreeView();
			_tree->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);
			_tree->setModel(&_model);
			layout->addWidget(_tree, 0, 1, 2, 1);
			
			connect(_control, SIGNAL(newFrame(Packet::LogFrame*)), _view, SLOT(frame(Packet::LogFrame*)));
			connect(_control, SIGNAL(newFrame(Packet::LogFrame*)), &_model, SLOT(frame(Packet::LogFrame*)));
			
			_control->setLogFile(logfile);
		}
		
	private:
		LogControl* _control;
		FieldView* _view;
		TreeView* _tree;
		
		TreeModel _model;
};

int main(int argc, char* argv[])
{
	QApplication app(argc, argv);
	
	LogFile* logfile = 0;
	if (argc == 2)
	{
		logfile = new LogFile(argv[1]);
	}
	
	MainWindow win(logfile);
	win.setFixedSize(800,600);
	win.show();
	
	int ret = app.exec();
	
	if (logfile)
	{
		delete logfile;
	}
	
	return ret;
}
