#include "Behavior.hpp"

#include <boost/foreach.hpp>

Gameplay::Behavior::Behavior(GameplayModule *gameplay, size_t minRobots)
	: _gameplay(gameplay), _minRobots(minRobots)
{
}

Gameplay::Behavior::~Behavior()
{
}

bool Gameplay::Behavior::run()
{
	return false;
}

bool Gameplay::Behavior::allVisible() const
{
	if (_robots.empty())
	{
		return false;
	}

	BOOST_FOREACH(Robot *r, _robots)
	{
		if (r && !r->visible())
		{
			return false;
		}
	}

	return true;
}

bool Gameplay::Behavior::assign(std::set<Robot *> &available)
{
	_robots.clear();
	takeBest(available);
	return _robots.size() >= _minRobots;
}

bool Gameplay::Behavior::takeAll(std::set<Robot *> &available)
{
	_robots = available;
	available.clear();
	return _robots.size() >= _minRobots;
}

Gameplay::Robot *Gameplay::Behavior::takeBest(std::set<Robot *> &available)
{
	if (available.empty())
	{
		return 0;
	}

	float bestScore = 0;
	Robot *best = 0;
	BOOST_FOREACH(Robot *r, available)
	{
		float s = score(r);
		if (!best)
		{
			bestScore = s;
			best = r;
		} else {
			if (s < bestScore)
			{
				bestScore = s;
				best = r;
			}
		}
	}

	// best is guaranteed not to be null because we already ensured that available is not empty.

	available.erase(best);
	_robots.insert(best);

	return best;
}

float Gameplay::Behavior::score(Robot *r)
{
	return 0;
}

void Gameplay::Behavior::drawText(const std::string& text,
								  const Geometry2d::Point& pt,
								  int r, int g, int b) {
	Packet::LogFrame::DebugText t;
	t.text = text;
	t.pos = pt;
	t.color[0] = r;
	t.color[1] = g;
	t.color[2] = b;

	gameplay()->state()->debugText.push_back(t);
}

void Gameplay::Behavior::drawText(const std::string& text,
								  const Geometry2d::Point& pt,
								  const QColor& color) {
	drawText(text, pt, color.red(), color.green(), color.blue());
}

void Gameplay::Behavior::drawLine(const Geometry2d::Segment& line,
								  int r, int g, int b) {
	Packet::LogFrame::DebugLine ln;
	ln.pt[0] = line.pt[0];
	ln.pt[1] = line.pt[1];
	ln.color[0] = r;
	ln.color[1] = g;
	ln.color[2] = b;
	gameplay()->state()->debugLines.push_back(ln);
}

void Gameplay::Behavior::drawLine(const Geometry2d::Segment& line,
								  const QColor& color) {
	drawLine(line, color.red(), color.green(), color.blue());
}

void Gameplay::Behavior::drawCircle(const Geometry2d::Point& center,
									float radius, int r, int g, int b) {
	Packet::LogFrame::DebugCircle c;
	c.radius(radius);
	c.center = center;
	c.color[0] = r;
	c.color[1] = g;
	c.color[2] = b;
	gameplay()->state()->debugCircles.push_back(c);
}

void Gameplay::Behavior::drawCircle(const Geometry2d::Point& center,
									float radius,
									const QColor& color) {
	drawCircle(center, radius, color.red(), color.green(), color.blue());
}
