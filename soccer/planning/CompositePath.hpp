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

			/**
			 * A path describes the position and velocity a robot should be at for a
			 * particular time interval.  This method evalates the path at a given time and
			 * returns the target position and velocity of the robot.
			 *
			 * @param[in] 	t Time (in seconds) since the robot started the path
			 * @param[out] 	targetPosOut The position the robot would ideally be at at the given time
			 * @param[out] 	targetVelOut The target velocity of the robot at the given time
			 * @return 		true if the path is valid at time @t, false if you've gone past the end
			 */
			virtual bool evaluate(float t, Geometry2d::Point &targetPosOut, Geometry2d::Point &targetVelOut) const override;

			/**
			 * Returns true if the path hits an obstacle
			 *
			 * @param[in]	shape The obstacles on the field
			 * @param[in] 	startTime The time on the path to start checking from
			 * @return 		true if it hits an obstacle, otherwise false
			 */
			virtual bool hit(const Geometry2d::CompositeShape &shape, float startTime = 0) const override;

			/**
			 * Draws the path
			 *
			 * @param[in]	state The SystemState to draw the path on
			 * @param[in] 	color The color the path should be drawn
			 * @param[in] 	layer The layer to draw the path on
			 */
			virtual void draw(SystemState * const state, const QColor &color = Qt::black, const QString &layer = "Motion") const override;

			/**
			 * Returns how long it would take for the entire path to be traversed
			 *
			 * @return 	The time from start to path completion or infinity if it never stops
			 */
			virtual float getDuration() const override;

			/**
			 * Returns a subPath
			 *
			 * @param[in]	startTime The startTime for from which the subPath should be taken.
			 * @param[in] 	endTime The endTime from which the subPath should be taken. If it is greater than the duration fo the path,
								 it should go to the end of the path.
			 * @return 	A unique_ptr to the new subPath
			 */
			virtual std::unique_ptr<Path> subPath(float startTime = 0, float endTime = std::numeric_limits<float>::infinity()) const override;

			/**
			 * Returns the destination point of the path if it has one
			 */
			virtual boost::optional<Geometry2d::Point> destination() const override;

			virtual std::unique_ptr<Path> clone() const override;
	};
}
