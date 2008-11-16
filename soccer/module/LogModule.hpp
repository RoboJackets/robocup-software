#ifndef LOGMODULE_HPP_
#define LOGMODULE_HPP_

#include <QMutex>

#include <framework/Module.hpp>
#include <log/LogFile.hpp>

/** logs the system state to a file */
class LogModule : public Module
{
	public:
		LogModule();
	
		void setLogFile(LogFile* file);
		
		virtual void run();
		
	private:
		QMutex _logFileMutex;
		LogFile* _logFile;
};

#endif /* LOGMODULE_HPP_ */
