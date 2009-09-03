// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et
#pragma once

#include <QGLWidget>
#include <QVector>

#include <Team.h>
#include <LogFrame.hpp>
#include <framework/Module.hpp>
#include <Network/Sender.hpp>

namespace Log
{
	/** class that performs drawing of log data onto the field */
	class FieldView : public QGLWidget
	{
		Q_OBJECT;

		public:
			FieldView(QWidget* parent = 0);
			
			void team(Team t);
			
			void addModule(Module* module);

			SystemState *state;
			
		protected:
			void paintEvent(QPaintEvent* pe);
			void resizeEvent(QResizeEvent* re);
			
			void mouseReleaseEvent(QMouseEvent*);
			void mousePressEvent(QMouseEvent*);
			void mouseMoveEvent(QMouseEvent*);
			void mouseDoubleClickEvent(QMouseEvent*);
			
		protected:
			/** convert from screen space to team space */
			Geometry2d::Point toTeamSpace(int x, int y) const;
			
			Geometry2d::Point toWorldSpace(Geometry2d::Point pt, bool translate = true) const;
			
			void drawField(QPainter&);

		public Q_SLOTS:
			void frame(Packet::LogFrame* frame);

		protected:
			/** frame to display */
			Packet::LogFrame* _frame;
			
			//translations for placing robots in team space
			float _tx, _ty, _ta;

			Team _team;
			
			//list of modules for fieldOverlay hook
			QVector<Module*> _modules;
			
			// True while a line is being dragged from the ball
			bool _dragBall;
			Geometry2d::Point _dragTo;
			
			// SimCommand sender
			Network::Sender _sender;
	};
}
