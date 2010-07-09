// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et
#pragma once

#include <QWidget>
#include <QVector>
#include <QTimer>

#include <Team.h>
#include <protobuf/LogFrame.pb.h>
#include <Network/Sender.hpp>

#include <boost/shared_ptr.hpp>

/** class that performs drawing of log data onto the field */
class FieldView : public QWidget
{
	Q_OBJECT;

	public:
		FieldView(QWidget* parent = 0);
		
		void team(Team t);
		
		void addModule(boost::shared_ptr<Module> module);

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
		bool _showVision;
		
		//list of modules for fieldOverlay hook
		QVector<boost::shared_ptr<Module> > _modules;
		
		// True while a line is being dragged from the ball
		bool _dragBall;
		Geometry2d::Point _dragTo;
		
		// SimCommand sender
		Network::Sender _sender;
		
		QTimer _updateTimer;
};
