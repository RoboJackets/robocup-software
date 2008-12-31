#include "MainWindow.hpp"

#include <QHBoxLayout>
#include <QGridLayout>

#include "module/modules.hpp"

using namespace Log;

MainWindow::MainWindow(Team t, QString filename) :
        QMainWindow(), _team(t), _processor(t), _logFile(0), _configFile(filename)
{
	QWidget* central = new QWidget();
	this->setCentralWidget(central);

	QGridLayout* layout = new QGridLayout();
	central->setLayout(layout);

	_fieldView = new FieldView(_team, this);
	_fieldView->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	_logControl = new LogControl(this);

	_treeView = new TreeView(this);
	_treeView->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);
	_treeModel = new TreeModel();
	_treeView->setModel(_treeModel);

	layout->addWidget(_fieldView, 0, 0);
	layout->addWidget(_logControl, 1, 0);
	layout->addWidget(_treeView, 0, 1, 2, 1);

	//start control thread
	_processor.start();

	_logFile = new LogFile(LogFile::genFilename());

        setupModules();

	_logControl->setLogFile(_logFile);
	connect(_logControl, SIGNAL(newFrame(Packet::LogFrame*)), _fieldView, SLOT(frame(Packet::LogFrame*)));
	connect(_logControl, SIGNAL(newFrame(Packet::LogFrame*)), _treeModel, SLOT(frame(Packet::LogFrame*)));
}

MainWindow::~MainWindow()
{
	//remove the log file from the things that use it
	_logControl->setLogFile(0);

	_processor.terminate();
	_processor.wait();

	//cleanup modules

	if (_logFile)
	{
		delete _logFile;
		_logFile = 0;
	}
}

void MainWindow::setupModules()
{
	WorldModel* wm = new WorldModel();

	Motion::Controller* motion = new Motion::Controller(_configFile);

	Log::LogModule* lm = new Log::LogModule();
	lm->setLogFile(_logFile);

	//add the modules....ORDER MATTERS!!
	_processor.addModule(wm);
	_processor.addModule(motion);
	_processor.addModule(lm);
}
