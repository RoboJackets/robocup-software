/*
 * @file passOptimization.hpp
 * @brief General typedefs necessary for pass Optimization
 * @author Alex Cunningham
 */

#pragma once

#include <map>
#include <boost/shared_ptr.hpp>
#include <gtsam/Point2.h>
#include <gtsam/TupleConfig.h>
#include <gtsam/LieConfig.h>
#include <gtsam/Key.h>
#include <gtsam/NonlinearFactorGraph.h>
#include <gtsam/NonlinearConstraint.h>
#include <gtsam/NonlinearOptimizer.h>
#include <gtsam/NonlinearEquality.h>
#include <common/Geometry2d/Point.hpp>

namespace Gameplay
{
	namespace Optimization {

		/**
		 * Keys for the robots
		 * Only one pose for each opponent,
		 * Indices of self are encoded such that the hundreds digit specifies robot number
		 * and lower digits are the number in the sequence for the robot
		 */
		typedef gtsam::TypedSymbol<gtsam::Point2, 'o'> OppKey;
		typedef gtsam::TypedSymbol<gtsam::Point2, 's'> SelfKey;
		typedef gtsam::TypedSymbol<gtsam::Point2, 'b'> BallKey;

		// Configs
		typedef gtsam::LieConfig<OppKey, gtsam::Point2> OppConfig;
		typedef gtsam::LieConfig<SelfKey, gtsam::Point2> SelfConfig;
		typedef gtsam::LieConfig<BallKey, gtsam::Point2> BallConfig;
		typedef gtsam::TupleConfig3<OppConfig, SelfConfig, BallConfig> Config;

		// Graph
		typedef gtsam::NonlinearFactorGraph<Config> Graph;

		// basic constraints
		typedef gtsam::NonlinearEquality<Config, OppKey, gtsam::Point2> OppConstraint;
		typedef gtsam::NonlinearEquality<Config, SelfKey, gtsam::Point2> SelfConstraint;
		typedef gtsam::NonlinearEquality<Config, BallKey, gtsam::Point2> FixedBallConstraint;

		// Optimizer
		typedef gtsam::NonlinearOptimizer<Graph, Config> Optimizer;

		// shared pointer versions
		typedef boost::shared_ptr<Graph> shared_graph;
		typedef boost::shared_ptr<Config> shared_config;

		// Conversion functions
		gtsam::Point2 rc2gt_Point2(const Geometry2d::Point& pt);
		Geometry2d::Point gt2rc_Point2(const gtsam::Point2& pt);

		/** Encoding constant - sets max number of frames for an element */
		const size_t maxFrames = 100;

		/** Encodes a frame and robot number as a single number */
		size_t encodeID(uint8_t robotNum, size_t frame_num);

		/** Decodes the id number to get the robot */
		uint8_t decodeRobot(size_t id);

		/** Decodes the id number to get the frame */
		size_t decodeFrame(size_t id);

		/** Basic constraints for initial positions */
		class RobotSelfConstraint : public SelfConstraint {
		public:
			RobotSelfConstraint(uint8_t robotID, size_t frame_num, const Geometry2d::Point& pt)
				: SelfConstraint(SelfKey(encodeID(robotID, frame_num)), rc2gt_Point2(pt)) {}
		};

		class RobotOppConstraint : public OppConstraint {
		public:
			RobotOppConstraint(uint8_t robotID, size_t frame_num, const Geometry2d::Point& pt)
				: OppConstraint(OppKey(encodeID(robotID, frame_num)), rc2gt_Point2(pt)) {}
		};

		class BallConstraint : public FixedBallConstraint {
		public:
			BallConstraint(size_t frame_num, const Geometry2d::Point& pt)
				: FixedBallConstraint(BallKey(frame_num), rc2gt_Point2(pt)) {}
		};

	} // \Optimization
} // \Gameplay

