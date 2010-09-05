#include "Window.hpp"

#include <stdio.h>
#include <Constants.hpp>
#include <framework/SystemState.hpp>
#include <boost/foreach.hpp>

using namespace std;

Gameplay::WindowEvaluator::WindowEvaluator(SystemState *state)
{
	_state = state;
	best = 0;
	_end = 0;
	debug = false;
}

Gameplay::WindowEvaluator::~WindowEvaluator()
{
	clear();
}

void Gameplay::WindowEvaluator::clear()
{
	best = 0;
	BOOST_FOREACH(Window *w, windows)
	{
		delete w;
	}
	windows.clear();
}

void Gameplay::WindowEvaluator::run(Geometry2d::Point origin)
{
	Geometry2d::Point g0(Field_GoalWidth / 2, Field_Length);
	Geometry2d::Point g1(-Field_GoalWidth / 2, Field_Length);
	Geometry2d::Segment goal(g0, g1);
	
	run(origin, goal);
}

void Gameplay::WindowEvaluator::run(Geometry2d::Point origin, const Geometry2d::Point &target)
{
	Geometry2d::Point dir = (target - origin).perpCCW().normalized();
	Geometry2d::Segment seg(target + dir * Robot_Radius,
							target - dir * Robot_Radius);
	
	run(origin, seg);
}

void Gameplay::WindowEvaluator::run(Geometry2d::Point origin, const Geometry2d::Segment &target)
{
	_origin = origin;
	_target = target;
	
	_end = target.delta().magsq();
	
	clear();
	
	if (_end == 0)
	{
		// Degenerate target can't have any windows
		printf("WindowEvaluator: Degenerate target\n");
		return;
	}
	
	Window *all = new Window(0, _end);
	windows.push_back(all);
	
	BOOST_FOREACH(const Robot *robot, _state->opp)
	{
		if (robot->visible)
		{
			bool skip = false;
			BOOST_FOREACH(const Geometry2d::Point &pt, exclude)
			{
				if (pt.nearPoint(robot->pos, Robot_Radius))
				{
					skip = true;
					break;
				}
			}
			
			if (!skip)
			{
				obstacleRobot(robot->pos);
			}
		}
	}
	
	BOOST_FOREACH(const Robot *robot, _state->self)
	{
		if (robot->visible)
		{
			bool skip = false;
			BOOST_FOREACH(const Geometry2d::Point &pt, exclude)
			{
				if (pt.nearPoint(robot->pos, Robot_Radius))
				{
					skip = true;
					break;
				}
			}
			
			if (!skip)
			{
				obstacleRobot(robot->pos);
			}
		}
	}
	
	finish();
}

void Gameplay::WindowEvaluator::finish()
{
	const Geometry2d::Point &p0 = _target.pt[0];
	Geometry2d::Point delta = _target.delta() / _end;
	BOOST_FOREACH(Window *w, windows)
	{
		w->segment.pt[0] = p0 + delta * w->t0;
		w->segment.pt[1] = p0 + delta * w->t1;
		
		w->a0 = (w->segment.pt[0] - _origin).angle() * RadiansToDegrees;
		w->a1 = (w->segment.pt[1] - _origin).angle() * RadiansToDegrees;
	}

	// Find the best (largest) window
	best = 0;
	BOOST_FOREACH(Window *w, windows)
	{
		if (!best || w->segment.delta().magsq() > best->segment.delta().magsq())
		{
			best = w;
		}
	}
	
	if (debug)
	{
		// Debug polygons for windows
		BOOST_FOREACH(Window *w, windows)
		{
			Geometry2d::Point pts[] = {
				_origin,
				w->segment.pt[0],
				w->segment.pt[1]
			};
			QColor color(
				(w == best) ? 255 : 0,
				0,
				(w == best) ? 0 : 255
			);
			_state->drawPolygon(pts, 3, color, "Windows");
		}
	}
}

void Gameplay::WindowEvaluator::obstacleRobot(Geometry2d::Point pos)
{
	Geometry2d::Point n = (pos - _origin).normalized();
	Geometry2d::Point t = n.perpCCW();
	const float r = Robot_Radius + Ball_Radius;
	Geometry2d::Segment seg(pos - n * Robot_Radius + t * r,
							pos - n * Robot_Radius - t * r);
	
	if (debug)
	{
//		_state->debugLines.push_back(seg);
	}
	Geometry2d::Point intersection[2];
	float extent[2] = {0, _end};
	bool valid[2] = {false, false};
	
	for (int i = 0; i < 2; ++i)
	{
		Geometry2d::Line edge(_origin, seg.pt[i]);
		float d = edge.delta().magsq();
		if (edge.intersects(_target, &intersection[i]) && (intersection[i] - _origin).dot(edge.delta()) > d)
		{
			valid[i] = true;
			float t = (intersection[i] - _target.pt[0]).dot(_target.delta());
			
			if (t < 0)
			{
				extent[i] = 0;
			} else if (t > _end)
			{
				extent[i] = _end;
			} else {
				extent[i] = t;
			}
		}
	}
	
	if (!valid[0] || !valid[1])
	{
		// Obstacle has no effect
		return;
	}
	
	obstacleRange(extent[0], extent[1]);
}

void Gameplay::WindowEvaluator::obstacleRange(float t0, float t1)
{
	if (t0 == t1)
	{
		// Ignore degenerate obstacles
		return;
	}
	
	if (t0 > t1)
	{
		swap(t0, t1);
	}
	
	list<Window *>::iterator i = windows.begin();
	while (i != windows.end())
	{
		Window *w = *i;
		list<Window *>::iterator next = i;
		++next;
		
		if (t0 <= w->t0 && t1 >= w->t1)
		{
			// The obstacle fully covers the window.
			// Remove this window.
			delete w;
			windows.erase(i);
		} else if (t0 > w->t0 && t1 < w->t1)
		{
			// The window fully contains the obstacle, so the window must be split.
			Window *w2 = new Window(t1, w->t1);
			windows.insert(next, w2);
			w->t1 = t0;
		} else if (t0 > w->t0 && t0 < w->t1)
		{
			// The obstacle covers the end of the window
			w->t1 = t0;
		} else if (t1 > w->t0 && t1 < w->t1)
		{
			// The obstacle covers the beginning of the window
			w->t0 = t1;
		}
		
		i = next;
	}
}
