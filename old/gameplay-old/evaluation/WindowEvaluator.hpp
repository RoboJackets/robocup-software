#pragma once

#include <vector>
#include <list>
#include <set>

#include <Geometry2d/Segment.hpp>

class SystemState;

namespace Gameplay
{
	// A window is a triangle.  WindowEvaluator creates zero or more Windows.
	// One vertex is the origin passed to run().
	// The side opposite this origin is a part of the original target segment.
	class Window
	{
		public:
			Window(float t0, float t1)
			{
				this->t0 = t0;
				this->t1 = t1;
			}

			float t0, t1;

			// Angles (in degrees) from origin along the edges of this window
			float a0, a1;
			
			// Piece of the target segment which represents this window
			Geometry2d::Segment segment;
	};
	
	class WindowEvaluator
	{
		public:
			// bool debug;
			
			// List of all windows
			// std::list<Window *> windows;
			
			// Any robots containing any of these points are ignored as obstacles.
			std::vector<Geometry2d::Point> exclude;
			
			// if enabled, uses min/max range to remove obstacles that can be cleared with chip
			// bool enable_chip;
			// float chip_min_range; // set according externally
			// float chip_max_range;

			const Geometry2d::Segment &target() const { return _target; }
			
			const Geometry2d::Point& origin() const { return _origin; }

			// void clear();
			
			// Calculates the windows to a segment.
			void run(Geometry2d::Point origin, const Geometry2d::Segment &target);
			
			// Calculates the windows to another robot
			void run(Geometry2d::Point origin, const Geometry2d::Point &target);
			
			// Calculates open shot windows into a goal
			void run(const Geometry2d::Segment &target, Geometry2d::Point origin, float dist);

			// Calculates the windows to the opponent's goal
			void run(Geometry2d::Point origin);

			// Window* best() { return _best; }


			// SystemState *state() {
			// 	return _state;
			// }


		protected:
			Geometry2d::Point _origin;
			Geometry2d::Segment _target;
			float _end;
			std::set<float> _edges;
			// SystemState *_state;
			
			// A pointer to the best window.
			// This will be one of the windows in <windows>, or 0 if there are no windows.
			// Window *_best;

			void finish();
			void obstacleRobot(Geometry2d::Point pos);
			void obstacleRange(float t0, float t1);
	};
}
