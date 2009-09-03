#pragma once

#include "Transform.h"

#include <Geometry2d/Point.hpp>

#include <QDomElement>
#include <QMutex>
#include <list>
#include <vector>

namespace Vision
{
	class Colorseg;
	class Distortion;
	
	// Almost-contiguous pixels from one or more spans
	class Group
	{
		public:
			Group();

			int min_x, max_x;
			int min_y, max_y;

			int width() const
			{
				return max_x - min_x + 1;
			}
			int height() const
			{
				return max_y - min_y + 1;
			}
			
			int color;

			// Used later to identify the group
			int id;

			// Center in image space
			Geometry2d::Point raw_center;

			// Center in undistorted coordinates
			Geometry2d::Point center;

			// Number of pixels covered by the group
			int num_pixels;

			// Number of pixels that were the right color (sum of num_pixels in all spans)
			int good_pixels;
	};
	
	
	// A range of horizontally almost-contiguous pixels
	class Span
	{
		public:
			Span(int x1, int x2, int y, int good_pixels)
			{
				this->x1 = x1;
				this->x2 = x2;
				this->y = y;
				this->good_pixels = good_pixels;
				group = 0;
			}
			
			
			// Returns true iff any part of this span's range includes the other span's range.
			bool overlaps(const Span *other) const
			{
				return x2 >= other->x1 && x1 <= other->x2;
			}
			
			
			// Returns true iff this span starts after the other span begins.
			bool after(const Span *other) const
			{
				return x1 > other->x2;
			}
			
			
			// X coordinate range
			int x1, x2;

			int y;

			// Number of pixels of the proper color in this span
			int good_pixels;

			Group *group;

			std::list<Span *> adjacencies;
	};
	
	class Spanner
	{
		public:
			Spanner(Colorseg *colorseg, int color);

			void run();

			void save(QDomElement element);
			void load(QDomElement element);

			int color() const
			{
				return _color;
			}
			
			// Returns a reference to the group list.
			// May only be used in the vision thread or with the mutex locked.
			//
			// The non-const version is needed because Robot_ID needs to set ID's and angles in some groups.
			std::list<Group *> &groups()
			{
				return _groups;
			}
			const std::list<Group *> &groups() const
			{
				return _groups;
			}
			
			// Returns a reference to the span list.
			// May only be used in the vision thread or with the mutex locked.
			const std::list<Span *> spans() const
			{
				return _all_spans;
			}
			
			// Lens distortion parameters
			Distortion *distortion;

			// Transformation for this color
			Transform *transform;

			int max_gap;
			int min_span_pixels;
			int min_group_width;
			int max_group_width;
			int min_group_height;
			int max_group_height;
			float min_group_aspect;
			float max_group_aspect;

			// If this equals _color, debug messages are printed.
			int debug_color;

			// Lock this while using any data from outside the vision thread.
			mutable QMutex mutex;

		protected:
			int _color;

			Colorseg *_colorseg;
			int _width, _height;

			// Finds spans on each row of the colorseg image
			void find_spans();

			// Sets group on the given span and all its adjacencies recursively,
			// and remove those spans (but not the given one) from _all_spans.
			void propagate_group(Span *span);

			// List of spans for each line
			std::vector<std::list<Span *> > _spans;

			// All spans
			std::list<Span *> _all_spans;

			std::list<Group *> _groups;
	};
}
