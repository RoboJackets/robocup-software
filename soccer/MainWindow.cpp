// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include "MainWindow.hpp"
#include "MainWindow.moc"

using namespace Log;
using namespace boost;

MainWindow::MainWindow(Team t, QString filename, bool sim) :
	_team(t),
	_processor(t, filename, this),
	_configFile(filename)
{
	ui.setupUi(this);
	statusBar()->hide();
	ui.fieldView->team(t);
	
// 	_treeModel = new TreeModel(ui.treeView);
	ui.splitter->setStretchFactor(0, 10);
	ui.splitter->setStretchFactor(1, 1);
// 	ui.treeView->setModel(_treeModel);
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
			addToolBar(m->toolbar());
		}
	}
	
	// Manually add tabs not connected with modules
	_configFileTab = new ConfigFileTab(_processor.configFile());
	ui.tabWidget->addTab(_configFileTab, tr("Configuration"));

	QToolBar* processorBar = new QToolBar("Processor Bar");
	_flipBox = new QCheckBox("Flip Field", processorBar);
	processorBar->addWidget(_flipBox);
	connect(_flipBox, SIGNAL(toggled(bool)), &_processor, SLOT(flipField(bool)));
	
	addToolBar(processorBar);
	
	if (sim)
	{
		_processor.vision_addr = "224.5.20.2";
	}

	//start control thread
	_processor.start();
	
	ui.fieldView->frame(&_viewFrame);
	
	connect(&_treeTimer, SIGNAL(timeout()), this, SLOT(updateTree()));
	_treeTimer.start(150);
}

MainWindow::~MainWindow()
{	
	_processor.terminate();
	_processor.wait();
}

PlayConfigTab *MainWindow::playConfig() const
{
	return _processor.gameplayModule()->playConfig();
}

void MainWindow::updateTree()
{
// 	_treeModel->frame(&_viewFrame);
}

bool MainWindow::event(QEvent* e)
{
	if (e->type() == QEvent::User)
	{
		_viewFrame = _processor.lastFrame();

		return true;
	} else {
		return QMainWindow::event(e);
	}
}

void MainWindow::flipField(bool value)
{
	_flipBox->setChecked(value);
	_processor.flipField(value);
}
