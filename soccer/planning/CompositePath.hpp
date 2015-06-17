#pragma once
#include <planning/Path.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Segment.hpp>
#include <Geometry2d/CompositeShape.hpp>
#include <Configuration.hpp>

namespace Planning
{
	/**
	 * @brief Represents a motion path made up of a series of Paths.
	 *
	 * @details The path represents a function of position given time that the robot should follow.
	 * The path is made up of other Paths and can be made up of CompositePaths.
	 */
	class CompositePath: public Path
	{
		private:
			//Vector of Paths
			std::vector<std::unique_ptr<Path>> paths;

			/**
			 * Append the path to the end of the CompositePath
			 * The path passed in should not be refenced anywhere else.
			 */
			void append(Path *path);

			//Saving some variables to speed up computation
			float duration;

		public:
			/** default path is empty */
			CompositePath() {}

			/** constructors with one path */
			CompositePath(Path *path);
			CompositePath(std::unique_ptr<Path> path);

			/** 
			 * Append the path to the end of the CompositePath
			 */
			void append(std::unique_ptr<Path> path);

			virtual bool evaluate(float t, Geometry2d::Point &targetPosOut, Geometry2d::Point &targetVelOut) const override;
			virtual bool hit(const Geometry2d::CompositeShape &shape, float startTime = 0) const override;
			virtual void draw(SystemState * const state, const QColor &color = Qt::black, const QString &layer = "Motion") const override;
			virtual float getDuration() const override;
			virtual std::unique_ptr<Path> subPath(float startTime = 0, float endTime = std::numeric_limits<float>::infinity()) const override;
			virtual boost::optional<Geometry2d::Point> destination() const override;
			virtual std::unique_ptr<Path> clone() const override;
	};
}
