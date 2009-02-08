#include "Radio.hpp"

#include <Network/Receiver.hpp>
#include <Network/Sender.hpp>
#include <Network/Network.hpp>

Radio::Radio(Team t, Env& env):
	_env(env), _running(true), _team(t)
{
	
}

Radio::~Radio()
{
	_running = false;
	wait();
}

void Radio::run()
{
	//rx/tx are from POV of control software
	//receive tx packets from control -> robots (base station)
	Network::Receiver receiver(Network::Address, Network::addTeamOffset(_team, Network::RadioTx));
	
	//send rx packets from robots (base station) -> control
	Network::Sender sender(Network::Address, Network::addTeamOffset(_team, Network::RadioRx));
	
	while (_running)
	{
		Packet::RadioTx radioTx;
		receiver.receive(radioTx);
		
		//control environment with radio data
		_env.radio(_team, radioTx);
		
		//get latest radio data from env
		Packet::RadioRx radioRx = _env.radio(_team);
		
		sender.send(radioRx);
		
		//TODO delay between tx and rx
	}
}
