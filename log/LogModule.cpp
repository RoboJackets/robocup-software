// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include "LogModule.hpp"

#include "drawing/Elements.hpp"

#include <GL/gl.h>
#include <boost/foreach.hpp>

using namespace Log;

LogModule::LogModule() :
	Module("Log Module")
{
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
	p.setPen(Qt::black);
	BOOST_FOREACH(const Geometry2d::Segment &seg, f.debugLines)
	{
		p.drawLine(seg.pt[0].toQPointF(), seg.pt[1].toQPointF());
	}
	
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
				drawRobot(p, f.team, r.shell, r.pos, r.angle, f.team, QString::fromStdString(r.behavior));
			}
		}

		Team opp = opponentTeam(f.team);
		BOOST_FOREACH(const Packet::LogFrame::Robot &r, f.opp)
		{
			if (r.valid)
			{
				drawRobot(p, opp, r.shell, r.pos, r.angle, f.team, QString::fromStdString(r.behavior));
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
	}
	_logFileMutex.unlock();
}
