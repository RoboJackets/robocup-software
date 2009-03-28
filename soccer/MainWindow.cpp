// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
#include "MainWindow.hpp"
#include "MainWindow.moc"
#include <QHBoxLayout>
#include <QGridLayout>

using namespace Log;

MainWindow::MainWindow(Team t, QString filename) :
		QMainWindow(), _team(t), _processor(t, filename), _logFile(0), _configFile(filename)
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
	
	/*
	try
	{
		_config.load();
	}
	catch (std::runtime_error& re)
	{
		printf("Config Load Error: %s\n", re.what());
	}
	*/
	
    _logFile = new LogFile(LogFile::genFilename());
	_logControl->setLogFile(_logFile);
	_processor.setLogFile(_logFile);
	
	//TODO fixme
	connect(_logControl, SIGNAL(newFrame(Packet::LogFrame*)), _fieldView, SLOT(frame(Packet::LogFrame*)));
	//connect(_logControl, SIGNAL(newFrame(Packet::LogFrame*)), _treeModel, SLOT(frame(Packet::LogFrame*)));
	
	//TODO make the log elements parent be the log control?
}

MainWindow::~MainWindow()
{	
	_processor.terminate();
	_processor.wait();
	
	if (_logFile)
	{
		_logControl->setLogFile(0);
		delete _logFile;
		_logFile = 0;
	}
}
