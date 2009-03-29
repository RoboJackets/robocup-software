// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
#include "MainWindow.hpp"
#include "MainWindow.moc"
#include <QHBoxLayout>
#include <QGridLayout>

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

	//start control thread
	_processor.start();
	
    _logFile = new LogFile(LogFile::genFilename());
	ui.logControl->setLogFile(_logFile);
	_processor.setLogFile(_logFile);

	connect(ui.logControl, SIGNAL(newFrame(Packet::LogFrame*)), ui.fieldView, SLOT(frame(Packet::LogFrame*)));
	connect(ui.logControl, SIGNAL(newFrame(Packet::LogFrame*)), _treeModel, SLOT(frame(Packet::LogFrame*)));
	
	// Connect referee buttons
	connect(ui.refHalt, SIGNAL(clicked()), SLOT(refHalt()));
	connect(ui.refReady, SIGNAL(clicked()), SLOT(refReady()));
	connect(ui.refStop, SIGNAL(clicked()), SLOT(refStop()));
	connect(ui.refForceStart, SIGNAL(clicked()), SLOT(refForceStart()));
	connect(ui.refFirstHalf, SIGNAL(clicked()), SLOT(refFirstHalf()));
	connect(ui.refOvertime1, SIGNAL(clicked()), SLOT(refOvertime1()));
	connect(ui.refHalftime, SIGNAL(clicked()), SLOT(refHalftime()));
	connect(ui.refOvertime2, SIGNAL(clicked()), SLOT(refOvertime2()));
	connect(ui.refSecondHalf, SIGNAL(clicked()), SLOT(refSecondHalf()));
	connect(ui.refPenaltyShootout, SIGNAL(clicked()), SLOT(refPenaltyShootout()));
	connect(ui.refTimeoutBlue, SIGNAL(clicked()), SLOT(refTimeoutBlue()));
	connect(ui.refTimeoutYellow, SIGNAL(clicked()), SLOT(refTimeoutYellow()));
	connect(ui.refTimeoutEnd, SIGNAL(clicked()), SLOT(refTimeoutEnd()));
	connect(ui.refTimeoutCancel, SIGNAL(clicked()), SLOT(refTimeoutCancel()));
	connect(ui.refKickoffBlue, SIGNAL(clicked()), SLOT(refKickoffBlue()));
	connect(ui.refKickoffYellow, SIGNAL(clicked()), SLOT(refKickoffYellow()));
	connect(ui.refDirectBlue, SIGNAL(clicked()), SLOT(refDirectBlue()));
	connect(ui.refDirectYellow, SIGNAL(clicked()), SLOT(refDirectYellow()));
	connect(ui.refIndirectBlue, SIGNAL(clicked()), SLOT(refIndirectBlue()));
	connect(ui.refIndirectYellow, SIGNAL(clicked()), SLOT(refIndirectYellow()));
	connect(ui.refPenaltyBlue, SIGNAL(clicked()), SLOT(refPenaltyBlue()));
	connect(ui.refPenaltyYellow, SIGNAL(clicked()), SLOT(refPenaltyYellow()));
	connect(ui.refGoalBlue, SIGNAL(clicked()), SLOT(refGoalBlue()));
	connect(ui.refSubtractGoalBlue, SIGNAL(clicked()), SLOT(refSubtractGoalBlue()));
	connect(ui.refGoalYellow, SIGNAL(clicked()), SLOT(refGoalYellow()));
	connect(ui.refSubtractGoalYellow, SIGNAL(clicked()), SLOT(refSubtractGoalYellow()));
	connect(ui.refYellowCardBlue, SIGNAL(clicked()), SLOT(refYellowCardBlue()));
	connect(ui.refYellowCardYellow, SIGNAL(clicked()), SLOT(refYellowCardYellow()));
	connect(ui.refRedCardBlue, SIGNAL(clicked()), SLOT(refRedCardBlue()));
	connect(ui.refRedCardYellow, SIGNAL(clicked()), SLOT(refRedCardYellow()));
}

MainWindow::~MainWindow()
{	
	_processor.terminate();
	_processor.wait();
	
	if (_logFile)
	{
		ui.logControl->setLogFile(0);
		delete _logFile;
	}
}

void MainWindow::refHalt()
{
	refCommand('H');
}

void MainWindow::refReady()
{
	refCommand(' ');
}

void MainWindow::refStop()
{
	refCommand('S');
}

void MainWindow::refForceStart()
{
	refCommand('s');
}

void MainWindow::refFirstHalf()
{
	refCommand('1');
}

void MainWindow::refOvertime1()
{
	refCommand('o');
}

void MainWindow::refHalftime()
{
	refCommand('h');
}

void MainWindow::refOvertime2()
{
	refCommand('O');
}

void MainWindow::refSecondHalf()
{
	refCommand('2');
}

void MainWindow::refPenaltyShootout()
{
	refCommand('a');
}

void MainWindow::refTimeoutBlue()
{
	refCommand('T');
}

void MainWindow::refTimeoutYellow()
{
	refCommand('t');
}

void MainWindow::refTimeoutEnd()
{
	refCommand('z');
}

void MainWindow::refTimeoutCancel()
{
	refCommand('c');
}

void MainWindow::refKickoffBlue()
{
	refCommand('K');
}

void MainWindow::refKickoffYellow()
{
	refCommand('k');
}

void MainWindow::refDirectBlue()
{
	refCommand('F');
}

void MainWindow::refDirectYellow()
{
	refCommand('f');
}

void MainWindow::refIndirectBlue()
{
	refCommand('I');
}

void MainWindow::refIndirectYellow()
{
	refCommand('i');
}

void MainWindow::refPenaltyBlue()
{
	refCommand('P');
}

void MainWindow::refPenaltyYellow()
{
	refCommand('p');
}

void MainWindow::refGoalBlue()
{
	++_processor.state().gameState.scoreBlue;
	refCommand('G');
}

void MainWindow::refSubtractGoalBlue()
{
	if (_processor.state().gameState.scoreBlue)
	{
		--_processor.state().gameState.scoreBlue;
	}
	
	refCommand('D');
}

void MainWindow::refGoalYellow()
{
	++_processor.state().gameState.scoreBlue;
	refCommand('g');
}

void MainWindow::refSubtractGoalYellow()
{
	if (_processor.state().gameState.scoreBlue)
	{
		--_processor.state().gameState.scoreBlue;
	}
	
	refCommand('d');
}

void MainWindow::refYellowCardBlue()
{
	refCommand('Y');
}

void MainWindow::refYellowCardYellow()
{
	refCommand('y');
}

void MainWindow::refRedCardBlue()
{
	refCommand('R');
}

void MainWindow::refRedCardYellow()
{
	refCommand('r');
}

void MainWindow::refCommand(char cmd)
{
	_processor.refereeHandler().command(cmd);
}
