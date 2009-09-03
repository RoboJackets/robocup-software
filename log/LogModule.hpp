// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et
#pragma once

#include <QMutex>

#include <framework/Module.hpp>
#include "LogFile.hpp"

namespace Log
{
	/** logs the system state to a file */
	class LogModule : public Module
	{
		public:
			LogModule();
		
			void setLogFile(LogFile* file);
			
			virtual void run();
			
			virtual void fieldOverlay(QPainter&, Packet::LogFrame&) const;
			
		private:
			QMutex _logFileMutex;
			bool _showVision;
			LogFile* _logFile;
	};
}
