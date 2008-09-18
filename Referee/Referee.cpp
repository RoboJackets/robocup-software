#include "Referee.hpp"

Referee::Referee() :
	QMainWindow()
{
	setupUi(this);

	//connect to timer for sending ref packets out
	connect(&_txTimer, SIGNAL(timeout()), SLOT(send()));

	//setup transmit timer
	_txTimer.start(1000/Referee::Hz);
}

Referee::~Referee()
{
	
}

void Referee::send()
{
	
}
