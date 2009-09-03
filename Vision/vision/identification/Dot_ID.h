#pragma once

#include "../Process.h"
#include "../Spanner.h"
#include "Identifier.hpp"

#include <Geometry2d/Point.hpp>

#include <QMutex>
#include <vector>
#include <list>

static inline float fix_angle(float a)
{
	if (a > 180)
	{
		a -= 360;
	}
	else if (a < -180)
	{
		a += 360;
	}
	
	return a;
}

namespace Vision
{
	typedef std::vector<Color> Color_Sequence;
	
	static inline Color_Sequence Colors(Color c0, Color c1)
	{
		Color_Sequence s(2);
		s[0] = c0;
		s[1] = c1;
		return s;
	}
	
	static inline Color_Sequence Colors(Color c0, Color c1, Color c2)
	{
		Color_Sequence s(3);
		s[0] = c0;
		s[1] = c1;
		s[2] = c2;
		return s;
	}
	
	static inline Color_Sequence Colors(Color c0, Color c1, Color c2, Color c3)
	{
		Color_Sequence s(4);
		s[0] = c0;
		s[1] = c1;
		s[2] = c2;
		s[3] = c3;
		return s;
	}
	
	class Pattern
	{
		public:
			Pattern()
			{
				angle_offset = 0;
			}
			
			Pattern(float a, const Geometry2d::Point &c, 
					const std::vector<Color_Sequence> &s) :
				angle_offset(a), center_offset(c), sequences(s)
			{
			}
			
			float angle_offset;
			Geometry2d::Point center_offset;
			std::vector<Color_Sequence> sequences;
	};
	
	class Dot_ID : public Identifier
	{
		public:
			class Dot
			{
				public:
					Dot(Group *group)
					{
						this->group = group;
					}
					
					Group *group;
					float angle;

					static bool compare_angles(const Dot &a, const Dot &b)
					{
						return a.angle < b.angle;
					}
			};

			Dot_ID(Process *process, Color center_color);

			// Array of Num_Colors pointers to Spanners.
			Spanner **spanner;

			// Populates _dots with the dots around center_group.
			void collect(const Group *center_group);

			// Removes the center dot from _dots.
			void remove_center();

			// Rotates _dots so that the first Dot is the one at the beginning of the
			// largest angular gap.
			//
			// remove_center should be called before this.
			void align_largest_gap();

			// Sets each group's id to its position in _dots.
			void assign_dot_ids();

			// Returns the index in patterns of the first pattern that matches.
			// Uses match_color_sequence() to check each pattern.
			//
			// Returns <0 if no patterns match.
			int find_id(const Pattern &pattern) const;

			// Returns true if the dot colors match the given sequence in order.
			bool match_color_sequence(const Color_Sequence &seq) const;

			void set_report(Group* center_group, const Pattern &pattern,
			        const Geometry2d::Point &center,
			        const Geometry2d::Point &facing);
			void set_report(Group* center_group, const Pattern &pattern, int id,
			        const Geometry2d::Point &center,
			        const Geometry2d::Point &facing);

			const std::vector<Dot> &dots() const
			{
				return _dots;
			}
			const Geometry2d::Point &center() const
			{
				return _center;
			}
			
		protected:
			Process *_process;
			int _center_color;

			// List of colors to check for dots
			std::vector<Color> _dot_colors;

			std::vector<Dot> _dots;

			// Average position of all dots.
			// Set by collect(), so this is unaffected by remove_center().
			Geometry2d::Point _center;
	};
}
