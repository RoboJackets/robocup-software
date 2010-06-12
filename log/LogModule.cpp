// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include "LogModule.hpp"

#include "drawing/Elements.hpp"

#include <QColor>
#include <GL/gl.h>
#include <boost/foreach.hpp>

using namespace Log;

LogModule::LogModule(SystemState *state) :
	Module("Log Module")
{
	_state = state;
	_logFile = 0;
	_showVision = false;
}

void LogModule::setLogFile(LogFile* file)
{
	//TODO this mutex should not be here...instread in code
	//that actually uses this object?? - Roman
	_logFileMutex.lock();
	_logFile = file;
	_logFileMutex.unlock();
}

void LogModule::fieldOverlay(QPainter& p, Packet::LogFrame& f) const
{
	// draw debug lines
	BOOST_FOREACH(const Packet::LogFrame::DebugLine& seg, f.debugLines)
	{
		p.setPen(QColor(seg.color[0], seg.color[1], seg.color[2]));
		p.drawLine(seg.pt[0].toQPointF(), seg.pt[1].toQPointF());
	}

	// draw debug circles
	BOOST_FOREACH(const Packet::LogFrame::DebugCircle& cir, f.debugCircles)
	{
		p.setPen(QColor(cir.color[0], cir.color[1], cir.color[2]));
		p.drawEllipse(cir.center.toQPointF(), cir.radius(), cir.radius());
	}

	// draw debug text
	BOOST_FOREACH(const Packet::LogFrame::DebugText& tex, f.debugText)
	{
		p.setPen(QColor(tex.color[0], tex.color[1], tex.color[2]));
		QString text = QString::fromStdString(tex.text);
		p.save();
		p.translate(tex.pos.x, tex.pos.y);
		p.rotate((f.team == Blue) ? -90 : 90);
		p.scale(.008, -.008);
		QRect r = p.boundingRect(0, 0, 0, 0, 0, text);
		p.drawText(-r.width() / 2, r.height() / 2 - 4, text);
		p.restore();
	}

	p.setPen(Qt::black);
	// Save GL_BLEND state since QPainter needs it
	glPushAttrib(GL_COLOR_BUFFER_BIT);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	BOOST_FOREACH(const Packet::LogFrame::DebugPolygon &polygon, f.debugPolygons)
	{
		glColor4ub(polygon.color[0], polygon.color[1], polygon.color[2], 64);
		glBegin(GL_POLYGON);
		BOOST_FOREACH(const Geometry2d::Point &pt, polygon.vertices)
		{
			glVertex2f(pt.x, pt.y);
		}
		glEnd();
	}
	glDisable(GL_BLEND);
	glPopAttrib();

	if (_showVision)
	{
		BOOST_FOREACH(const Packet::Vision& vision, f.rawVision)
		{
			if (!vision.sync)
			{
				BOOST_FOREACH(const Packet::Vision::Robot& r, vision.blue)
				{
					drawRobot(p, Blue, r.shell, r.pos, r.angle, f.team);
				}

				BOOST_FOREACH(const Packet::Vision::Robot& r, vision.yellow)
				{
					drawRobot(p, Yellow, r.shell, r.pos, r.angle, f.team);
				}

				BOOST_FOREACH(const Packet::Vision::Ball& b, vision.balls)
				{
					drawBall(p, b.pos);
				}
			}
		}
	} else {
		BOOST_FOREACH(const Packet::LogFrame::Robot &r, f.self)
		{
			if (r.valid)
			{
				drawRobot(p, f.team, r.shell, r.pos, r.angle, f.team, r.haveBall);
			}
		}

		Team opp = opponentTeam(f.team);
		BOOST_FOREACH(const Packet::LogFrame::Robot &r, f.opp)
		{
			if (r.valid)
			{
				drawRobot(p, opp, r.shell, r.pos, r.angle, f.team, r.haveBall);
			}
		}

		if (f.ball.valid)
		{
			drawBall(p, f.ball.pos, f.ball.vel);
		}
	}
}

void LogModule::run()
{
	_logFileMutex.lock();
	if (_logFile && _state)
	{
		//write out log frame part of the state
		_logFile->write(*_state);
		_logFile->setLast(*_state);
	}
	_logFileMutex.unlock();
}
