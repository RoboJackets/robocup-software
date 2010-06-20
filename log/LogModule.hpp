// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et
#pragma once

#include <QMutex>

#include <framework/Module.hpp>

namespace Log
{
	/** logs the system state to a file */
	class LogModule : public Module
	{
		public:
			typedef boost::shared_ptr<LogModule> shared_ptr;
			LogModule(SystemState *state);

			virtual void run();

			virtual void fieldOverlay(QPainter&, Packet::LogFrame&) const;

		private:
			SystemState *_state;
			bool _showVision;
	};
}
