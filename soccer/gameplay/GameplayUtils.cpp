#include "GameplayUtils.hpp"
#include "Window.hpp"

#include <boost/foreach.hpp>

float Gameplay::scorePosition(WindowEvaluator &win, Geometry2d::Point pos, Geometry2d::Point ball_pos)
{
	win.run(pos);
	
	if (win.best)
	{
		float goalScore = win.best->segment.delta().magsq();
		
		win.run(ball_pos, pos);
		if (win.best)
		{
			float ballScore = win.best->segment.delta().magsq();
			return goalScore + ballScore;
		}
	}
	
	return 0;
}

Geometry2d::Point Gameplay::bestPassInTree(WindowEvaluator &win, Motion::Tree &tree,
	Geometry2d::Point cur, Geometry2d::Point ball)
{
	// Re-score our current position
	Geometry2d::Point pos = cur;
	float bestScore = scorePosition(win, pos, ball);
	
	BOOST_FOREACH(Motion::Tree::Point *pt, tree.points)
	{
//		if (pt->leaf)
		{
			float score = scorePosition(win, pt->pos, ball);
			if (score > bestScore)
			{
				bestScore = score;
				pos = pt->pos;
			}
		}
	}
	
	return pos;
}
