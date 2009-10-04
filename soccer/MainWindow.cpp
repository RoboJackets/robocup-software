// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include "MainWindow.hpp"
#include "MainWindow.moc"
#include <QHBoxLayout>
#include <QGridLayout>
#include <QKeyEvent>

using namespace Log;

MainWindow::MainWindow(Team t, QString filename) :
		_team(t), _processor(t, filename), _logFile(0), _configFile(filename)
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
	Q_FOREACH(Module::shared_ptr m, _processor.modules())
	{
		ui.fieldView->addModule(m);
		
		if (m->widget())
		{
			ui.tabWidget->addTab(m->widget(), m->widget()->objectName());
		}
		
		if (m->toolbar())
		{
			this->addToolBar(m->toolbar());
		}
	}
	
	QToolBar* processorBar = new QToolBar("Processor Bar");
	processorBar->setVisible(false);
	
	QCheckBox* flibBox = new QCheckBox("Flip Field", processorBar);
	processorBar->addWidget(flibBox);
	
	connect(flibBox, SIGNAL(toggled(bool)), &_processor, SLOT(flip_field(bool)));
	
	this->addToolBar(processorBar);
	
	//start control thread
	_processor.start();
	
    _logFile = new LogFile(LogFile::genFilename());
	ui.logControl->setLogFile(_logFile);
	_processor.setLogFile(_logFile);

	connect(ui.logControl, SIGNAL(newFrame(Packet::LogFrame*)), ui.fieldView, SLOT(frame(Packet::LogFrame*)));
	connect(ui.logControl, SIGNAL(newFrame(Packet::LogFrame*)), _treeModel, SLOT(frame(Packet::LogFrame*)));
}

MainWindow::~MainWindow()
{	
	_processor.terminate();
	_processor.wait();
	
	if (_logFile)
	{
		ui.logControl->setLogFile(0);
	}
}

void MainWindow::keyReleaseEvent(QKeyEvent* ke)
{
	if (ke->key() & Qt::Key_Space)
	{
		_processor.on_input_playPauseButton();
	}
}
