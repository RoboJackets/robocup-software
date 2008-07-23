#ifndef _GOALIE_HPP_
#define _GOALIE_HPP_

#include "Tactics.hpp"

#include <list>

#include <Geometry/Point2d.hpp>
#include <Geometry/Circle2d.hpp>

namespace Tactics
{
    class Goalie: public Tactics::Base
    {
    	/// Types ///
    	public:
			typedef struct Span
			{
				Span(float amin, float amax) : min(amin), max(amax) {}
				
				float size() const { return fabs(max - min); }
				float posAvg() const { return min + (max - min)/2.0; }	
				
				float min;
				float max;
			} Span;
    	
		public:
			Goalie();
			
			virtual float score(Robot *robot);
			virtual void run();
			
			/** calculate windows, ignoring robot with id ignore */
			static std::list<Goalie::Span> calcWindow(int ignore = -1);
			
			/** remove the shadow from pevious windows */
			static void removeShadow(Goalie::Span s, std::list<Goalie::Span>& spans);
			
			/** see if the robot casts a shadow onto the goal by robot, from point */
			static Goalie::Span shadow(Geometry::Circle2d obstacle, Geometry::Point2d point);
			
		private:
    };
}

#endif // _GOALIE_HPP_
