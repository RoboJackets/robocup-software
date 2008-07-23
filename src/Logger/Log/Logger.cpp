#include "Logger.hpp"

#include <Packet/PacketReceiver.hpp>

using namespace Log;

Loggable* Logger::_loggable = 0;

Logger::Logger(Team t) :
	_running(true), _team(t)
{
	
}

Logger::~Logger()
{
	_running = false;
	wait();
	
	delete _loggable;
}

void Logger::run()
{
	Packet::PacketReceiver recv(_team);
	
	Log::PacketType::TypeList types = Log::PacketType::types();
	for (unsigned int i=0 ; i<types.size() ; ++i)
	{
		types[i]->listen(recv);
	}
	
	while (_running)
	{
		recv.receive();
	}
}
