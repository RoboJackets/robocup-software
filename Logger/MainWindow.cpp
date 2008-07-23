#include "MainWindow.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QToolBox>
#include <QGroupBox>

#include "Log/LiveOnly.hpp"
#include "Tools.hpp"

#include "Log/Packets/VisionData.hpp"

MainWindow::MainWindow(Team team) :
	QMainWindow(), _logger(team), _disp(team)
{
	this->resize(800,600);

	QWidget* central = new QWidget();
	this->setCentralWidget(central);
	this->setWindowTitle("Logger");

	//display and its controls
	QWidget* top = new QWidget();
	//QHBoxLayout* _layout = new QHBoxLayout(top);
	QGridLayout* layout = new QGridLayout(top);
	layout->addWidget(&_disp, 0, 0);

	//configurations for all packet types
	_typeConfigs = new QWidget(top);
	QVBoxLayout* typeLayout = new QVBoxLayout(_typeConfigs);
	layout->addWidget(_typeConfigs, 0, 1);

	layout->setColumnStretch(0,3);
	layout->setColumnStretch(1,1);

	//layout for entire form
	QVBoxLayout* centerLayout = new QVBoxLayout(this->centralWidget());
	centerLayout->addWidget(top);
	centerLayout->addWidget(&_control);

	//create tools toolbox and connect the new location signal to it
	Tools* tools = new Tools(team, _vision, this->centralWidget());
	QObject::connect(&_disp, SIGNAL(newPosition(float, float, float, float, QMouseEvent)), tools,
			SLOT(newPosition(float, float, float, float, QMouseEvent)));

	addDisplay("Tools", tools);

	//populate type configs
	Log::PacketType::TypeList types = Log::PacketType::types();
	for (unsigned int i=0 ; i<types.size() ; ++i)
	{
		QWidget* type = types[i]->information();
		if (type)
		{
			addDisplay(QString(types[i]->name()), type);
		}
	}

	//redisplay timers
	connect(&_timer, SIGNAL(timeout()), SLOT(redraw()));
	_timer.start(30);

	//start the logging of packets
	_logger.start();

	loggable(new Log::LiveOnly());
}

MainWindow::~MainWindow()
{

}

void MainWindow::loggable(Log::Loggable* loggable)
{
	_logger.loggable(loggable);
	_control.loggable(loggable);
}

void MainWindow::redraw()
{
	QList<Log::LogPacket*> packets = _logger.dispPackets();
	QList<Log::LogPacket*> lastPackets = _logger.lastDispPackets();

	Q_FOREACH(Log::LogPacket* packet, lastPackets)
	{
		packet->updateInformationWidget();
		
		//if vision packet, store it
		if (packet->type()->id() == Packet::VisionData::Type)
		{
			Log::VisionDataType::VisionData* vpacket = dynamic_cast<Log::VisionDataType::VisionData*>(packet);
			_vision = vpacket->data();
		}
		
		delete packet;
	}
	
	_disp.setPackets(packets);
	_disp.update();

	_control.update();
}

void MainWindow::addDisplay(QString name, QWidget* w)
{
	QGroupBox* group = new QGroupBox(name , _typeConfigs);
	group->setCheckable(true);
	group->setLayout(new QVBoxLayout());
	
	connect(group, SIGNAL(toggled(bool)), w, SLOT(setVisible(bool)));
	group->setChecked(false);
	
	group->layout()->addWidget(w);
	group->layout()->setContentsMargins(2,2,2,2);
	_typeConfigs->layout()->addWidget(group);
}
