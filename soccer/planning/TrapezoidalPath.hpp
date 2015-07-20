#pragma once
#include <planning/Path.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Segment.hpp>
#include <Geometry2d/CompositeShape.hpp>
#include <Configuration.hpp>
#include "MotionInstant.hpp"
#include "../../common/Geometry2d/Segment.hpp"
#include "motion/TrapezoidalMotion.hpp"
#include "MotionConstraints.hpp"


namespace Planning
{
	/**
	 * @brief Represents a direct Trapezoidal Path
	 *
	 * @details The path represents a function of position given time that the robot should follow.
	 * The path is made up of other Paths and can be made up of CompositePaths.
	 */
	class TrapezoidalPath: public Path
	{

	private:
		const Geometry2d::Point startPos, endPos;
		const Geometry2d::Point pathDirection;
		const float startSpeed, endSpeed;

		const float pathLength;
		const float maxAcc;
		const float maxSpeed;

	public:
		TrapezoidalPath(Geometry2d::Point startPos, float startSpeed, Geometry2d::Point endPos, float endSpeed, float maxAcc, float maxSpeed) :
				startPos(startPos), startSpeed(startSpeed), endPos(endPos), endSpeed(endSpeed),
				pathLength((startPos - endPos).mag()), maxAcc(maxAcc), maxSpeed(maxSpeed),
				pathDirection((endPos - startPos).normalized()){
				}

		/** default path is empty */
		TrapezoidalPath(Geometry2d::Point startPos, float startSpeed, Geometry2d::Point endPos, float endSpeed, const MotionConstraints& constraints) :
				startPos(startPos), startSpeed(startSpeed), endPos(endPos), endSpeed(endSpeed),
				pathLength((startPos - endPos).mag()), maxAcc(constraints.maxAcceleration), maxSpeed(constraints.maxSpeed),
				pathDirection((endPos - startPos).normalized()){
					float minSpeed = maxSpeed;
					if (startSpeed<minSpeed) {
						startSpeed = minSpeed;
					}
				}

		virtual bool evaluate(float time, MotionInstant &targetMotionInstant) const override {
			float distance;
			float speedOut;
			bool valid = TrapezoidalMotion(
					pathLength, 	//PathLength
					maxSpeed,		//maxSpeed
					maxAcc,		//maxAcc
					time,	//time
					startSpeed,	//startSpeed
					endSpeed,	//endSpeed
					distance,		//posOut
					speedOut);	//speedOut
			targetMotionInstant.pos = pathDirection * distance + startPos;
			targetMotionInstant.vel = pathDirection * speedOut;
			return valid;
		}

		virtual bool hit(const Geometry2d::CompositeShape &shape, float &hitTime, float startTime = 0) const override {
			throw "This function is not implemented";
		}

		virtual void draw(SystemState * const state, const QColor &color = Qt::black, const QString &layer = "Motion") const override {
			Packet::DebugPath *dbg = state->logFrame->add_debug_paths();
			dbg->set_layer(state->findDebugLayer(layer));
			*dbg->add_points() = startPos;
			*dbg->add_points() = endPos;
		}

		virtual float getDuration() const override {
			static float duration = Trapezoidal::getTime(
					pathLength, //distance
					pathLength, //pathLength
					maxSpeed,
					maxAcc,
					startSpeed,
					endSpeed);

			return duration;
		}

		virtual std::unique_ptr<Path> subPath(float startTime = 0, float endTime = std::numeric_limits<float>::infinity()) const override {
			throw "This function is not implemented";
		}

		virtual boost::optional<MotionInstant> destination() const override {
			static MotionInstant destination = MotionInstant(endPos, pathDirection*endSpeed);
			return destination;
		}
		virtual std::unique_ptr<Path> clone() const override {
			throw "This function is not implemented";
			return std::unique_ptr<Path>(new TrapezoidalPath(startPos,startSpeed,endPos, endSpeed, maxAcc, maxSpeed));
		}
	};
}
