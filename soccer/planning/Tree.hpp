#pragma once

#include <list>

#include <Geometry2d/Segment.hpp>
#include <planning/InterpolatedPath.hpp>

namespace Planning
{
	/** base tree class for rrt trees
	 *  Tree can be grown in different ways */
	class Tree
	{
		public:

			/** base class for a tree point */
			class Point
			{
				public:
					Point(const Geometry2d::Point& pos, Point* parent);

					//field position of the point
					Geometry2d::Point pos;

					// Which obstacles contain this point
					std::set<std::shared_ptr<Geometry2d::Shape>> hit;

					//velocity information (used by dynamic tree)
					Geometry2d::Point vel;

					bool leaf;

					inline Point* parent() const { return _parent; }

					void addEdges(std::list<Geometry2d::Segment>& edges);

				private:
					std::list<Point *> children;
					Point* _parent;
			};

			Tree();
			virtual ~Tree();

			/** cleanup the tree */
			void clear();

			void init(const Geometry2d::Point &start, const Geometry2d::CompositeShape *obstacles);

			/** find the point of the tree closest to @a pt */
			Point* nearest(Geometry2d::Point pt);

			/** grow the tree in the direction of pt
			 *  returns the new tree point.
			 *  If base == 0, then the closest tree point is used */
			virtual Point* extend(Geometry2d::Point pt, Point* base = 0) = 0;

			/** attempt to connect the tree to the point */
			virtual bool connect(const Geometry2d::Point pt) = 0;

			/** make a path from the dest point's root to the dest point
			 *  If rev is true, the path will be from the dest point to its root */
			void addPath(Planning::InterpolatedPath &path, Point* dest, const bool rev = false);

			/** returns the first point or 0 if none */
			Point* start() const;

			/** last point added */
			Point* last() const;

			/** tree step size...interpreted differently for different trees */
			float step;

			std::list<Point*> points;

		protected:
			const Geometry2d::CompositeShape* _obstacles;
	};

	/** tree that grows based on fixed distance step */
	class FixedStepTree : public Tree
	{
		public:
			FixedStepTree() {}

			Tree::Point* extend(Geometry2d::Point pt, Tree::Point* base = 0);
			bool connect(Geometry2d::Point pt);
	};
}
