
#pragma once
#include <planning/Path.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Segment.hpp>
#include <Geometry2d/CompositeShape.hpp>
#include <Configuration.hpp>

namespace Planning
{
	/**
	 * @brief Represents a motion path as a series of {pos, vel} pairs.
	 *
	 * @details The path represents a function of position given time that the robot should follow.  A
	 * line-segment-based path comes from the planner, then we use cubic bezier curves to interpolate
	 * and smooth it out.  This is done via the evaulate() method.
	 */
	class InterpolatedPath: public Path
	{
	public:
		// Set of points in the path - used as waypoints
		std::vector<Geometry2d::Point> points;
		std::vector<Geometry2d::Point> vels;
		std::vector<float> times;

		/** default path is empty */
		InterpolatedPath() {}

		/** constructor with a single point */
		InterpolatedPath(const Geometry2d::Point& p0);

		/** constructor from two points */
		InterpolatedPath(const Geometry2d::Point& p0, const Geometry2d::Point& p1);


		static InterpolatedPath* getStandStillPath(Geometry2d::Point point) {
			InterpolatedPath *empty = new InterpolatedPath();
			empty->addInstant(0, MotionInstant(point, Geometry2d::Point()));
			empty->addInstant(0.01, MotionInstant(point + Geometry2d::Point(0.01, 0.01), Geometry2d::Point()));
		}

		//Adds an instant ot the end of the path for the given time
		//time should not bet less than the last time
		void addInstant(float time, MotionInstant instant) {
			assert(points.size() == vels.size() && vels.size() == times.size());
			if (!times.empty()) {
				assert(time > times.back());
			}
			times.push_back(time);
			points.push_back(instant.pos);
			vels.push_back(instant.vel);
		}

		//Overriden Path Methods
		virtual boost::optional<MotionInstant> destination() const override;
		virtual bool hit(const Geometry2d::CompositeShape &shape, float &hitTime, float startTime) const override;
		virtual std::unique_ptr<Path> subPath(float startTime = 0, float endTime = std::numeric_limits<float>::infinity()) const override;
		virtual void draw(SystemState  * const state, const QColor &color, const QString &layer) const override;
		virtual bool evaluate(float t, MotionInstant &targetMotionInstant) const override;
		virtual float getDuration() const override;
		virtual std::unique_ptr<Path> clone() const override;

		bool empty() const
		{
			return points.empty();
		}

		void clear()
		{
			points.clear();
		}

		/**
		 * Calulates the length of the path
		 *
		 * \param[in] 	start Index of point in path to use at start point.
		 * \returns 	the length of the path starting at point (start).
		 */
		float length(unsigned int start = 0) const;

		/**
		 * Calulates the length of the path
		 *
		 * \param[in] 	start Index of point in path to use at start point.
		 * \param[in] 	end Index of point in path to use at end point.
		 * \returns 	the length of the path starting at point (start) and ending at point [end].
		 */
		float length(unsigned int start, unsigned int end) const;

		/** Returns the length of the path from the closet point found to @a pt */
		float length(const Geometry2d::Point &pt) const;

		/** Returns number of waypoints */
		size_t size() const;

		/** returns true if the path has non-zero size */
		bool valid() const;

		// Returns the index of the point in this path nearest to pt.
		int nearestIndex(const Geometry2d::Point &pt) const;

		/** returns the nearest segement of @a pt to the path */
		Geometry2d::Segment nearestSegment(const Geometry2d::Point &pt) const;

		// Returns the shortest distance from this path to the given point
		float distanceTo(const Geometry2d::Point &pt) const;

		// Returns the start of the path
		boost::optional<Geometry2d::Point> start() const;

		// Returns a new path starting from a given point
		void startFrom(const Geometry2d::Point& pt, Planning::InterpolatedPath& result) const;

		/**
		 * Evaluates the point and velocity of the robot at a given distance in the path.
		 *
		 * @param[in] 	distance A given distance the robot has traveled down the path from the start or t=0
		 * @param[out] 	position The position the robot given distance in the path
		 * @param[out] 	direction The target velocity of the robot at the given distance in the path
		 * @return 		true if the path is valid at the given distance, false if you've gone past the end
		 */
		bool getPoint(float distance ,Geometry2d::Point &position, Geometry2d::Point &direction) const;

		/**
		 * Estimates how long it would take for the robot to get to a certain point in the path using
		 * Trapezoidal motion.
		 *
		 * @param[in] 	index Index of the point on the path
		 * @return 		the estimated time it would take for the robot to a point on the path starting from the start of the path
		 */
		float getTime(int index) const;
	};
}
