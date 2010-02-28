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
			typedef boost::shared_ptr<LogModule> shared_ptr;
			LogModule(SystemState *state);

			void setLogFile(LogFile* file);

			virtual void run();

			virtual void fieldOverlay(QPainter&, Packet::LogFrame&) const;

		private:
			void drawPoseHistory(QPainter &p, const Packet::LogFrame::Robot &robot) const;

			SystemState *_state;
			QMutex _logFileMutex;
			bool _showVision;
			LogFile* _logFile;
	};
}
