#include "LogModule.hpp"

using namespace Log;

LogModule::LogModule() :
	Module("Log Module")
{
	_logFile = 0;
}

void LogModule::setLogFile(LogFile* file)
{
    //TODO this mutex should not be here...instread in code
    //that actually uses this object?? - Roman
	_logFileMutex.lock();
	_logFile = file;
	_logFileMutex.unlock();
}

void LogModule::run()
{
	_logFileMutex.lock();
	if (_logFile && _state)
	{
		//write out log frame part of the state
		_logFile->write(*_state);
	}
	_logFileMutex.unlock();
	
}
