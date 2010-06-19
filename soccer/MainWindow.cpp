// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include "MainWindow.hpp"
#include "MainWindow.moc"
#include <QHBoxLayout>
#include <QGridLayout>

using namespace Log;
using namespace boost;

MainWindow::MainWindow(Team t, QString filename, bool sim) :
	_team(t),
	_processor(t, filename),
	_logControl(new LogControl()),
	_logFile(0),
	_configFile(filename)
{
	ui.setupUi(this);
	statusBar()->hide();
	ui.fieldView->team(t);
	
	_treeModel = new TreeModel();
	ui.splitter->setStretchFactor(0, 10);
	ui.splitter->setStretchFactor(1, 1);
	ui.treeView->setModel(_treeModel);
	ui.treeView->expandAll();

	ui.fieldView->state = &_processor.state();
	Q_FOREACH(shared_ptr<Module> m, _processor.modules())
	{
		ui.fieldView->addModule(m);
		
		if (m->widget())
		{
			ui.tabWidget->addTab(m->widget(), m->widget()->windowTitle());
		}
		
		if (m->toolbar())
		{
			this->addToolBar(m->toolbar());
		}
	}
	
	// Manually add tabs not connected with modules
	_configFileTab = new ConfigFileTab(_processor.configFile());
	ui.tabWidget->addTab(_configFileTab, tr("Configuration"));

	QToolBar* processorBar = new QToolBar("Processor Bar");
	processorBar->setVisible(false);
	
	QCheckBox* flibBox = new QCheckBox("Flip Field", processorBar);
	processorBar->addWidget(flibBox);
	
	connect(flibBox, SIGNAL(toggled(bool)), &_processor, SLOT(flip_field(bool)));
	
	this->addToolBar(processorBar);
	
	if (sim)
	{
		_processor.vision_addr = "224.5.20.2";
	}

	//start control thread
	_processor.start();
	
	_logFile = new LogFile(LogFile::genFilename());
	_logControl->setLogFile(_logFile);
	_processor.setLogFile(_logFile);

	connect(_logControl, SIGNAL(newFrame(Packet::LogFrame*)), ui.fieldView, SLOT(frame(Packet::LogFrame*)));
	connect(_logControl, SIGNAL(newFrame(Packet::LogFrame*)), _treeModel, SLOT(frame(Packet::LogFrame*)));
}

MainWindow::~MainWindow()
{	
	_processor.terminate();
	_processor.wait();
	
	if (_logFile)
	{
		_logControl->setLogFile(0);
	}
	delete _logControl;
}

PlayConfigTab *MainWindow::playConfig() const
{
	return _processor.gameplayModule()->playConfig();
}
