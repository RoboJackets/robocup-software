// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

#include <QObject>
#include <QString>
#include <framework/Module.hpp>
#include <RadioTx.hpp>

#include "Robot.hpp"
#include <framework/ConfigFile.hpp>
#include "ui_config.h"

namespace Motion
{
	class MotionModule: public QObject, public Module
	{
		Q_OBJECT;

		public:
			MotionModule(SystemState *state, ConfigFile::MotionModule& cfg);
			~MotionModule();

			virtual void run();
			
			virtual void slowRun();

			virtual QWidget* widget() const;

			virtual void fieldOverlay(QPainter&, Packet::LogFrame&) const;

			virtual void mousePress(QMouseEvent* me, Geometry2d::Point pos);

		public Q_SLOTS:
			void on_pos_kp_valueChanged(double value);
			void on_pos_ki_valueChanged(double value);
			void on_pos_kd_valueChanged(double value);
			
			void on_ang_kp_valueChanged(double value);
			void on_ang_ki_valueChanged(double value);
			void on_ang_kd_valueChanged(double value);

		private:
			Ui::Motion ui;

			SystemState *_state;
			QWidget* _configWidget;

			/** Robots **/
			Robot* _robots[5];
			
			ConfigFile::MotionModule& _config;
	};
}
