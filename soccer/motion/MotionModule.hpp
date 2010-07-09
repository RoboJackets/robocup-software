// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

#include <QObject>
#include <QString>
#include <protobuf/RadioTx.pb.h>

#include "Robot.hpp"
#include <framework/ConfigFile.hpp>
#include "ui_config.h"

namespace Motion
{
	class MotionModule: public QObject
	{
		Q_OBJECT;

		public:
			MotionModule(SystemState *state, const ConfigFile::MotionModule& cfg);
			~MotionModule();

			virtual void run();
			
			virtual QWidget* widget() const;

			virtual void fieldOverlay(QPainter&, Packet::LogFrame&) const;

			virtual void mousePress(QMouseEvent* me, Geometry2d::Point pos);

		private:
			Ui::Motion _ui;

			bool _guiInitialized; /// flag to initialize the indicators on the GUI

			SystemState *_state;
			QWidget* _configWidget;

			/** Robots **/
			Robot* _robots[5];
			
			const ConfigFile::MotionModule& _config;
	};
}
