/*
 * @file passOptimization.hpp
 * @brief General typedefs necessary for pass Optimization
 * @author Alex Cunningham
 */

#pragma once

#include <map>
#include <boost/shared_ptr.hpp>
#include <gtsam/Pose2.h>
#include <gtsam/TupleConfig.h>
#include <gtsam/LieConfig.h>
#include <gtsam/Key.h>
#include <gtsam/NonlinearFactorGraph.h>
#include <gtsam/NonlinearConstraint.h>
#include <gtsam/NonlinearOptimizer.h>
#include <gtsam/NonlinearEquality.h>
#include <gtsam/PriorFactor.h>
#include <gameplay/optimizer/LagrangePriorFactor.h>
#include <Geometry2d/Point.hpp>

namespace Gameplay
{
	namespace Optimization {

		/** Datatypes for robots
		 *  Pose2s for self, Point2 for Opp (easier)
		 *  FIXME: this should include times
		 */
		typedef gtsam::Point2 Opp_t;
		typedef gtsam::Pose2 Self_t;

		/**
		 * Keys for the robots
		 * Only one pose for each opponent,
		 * Indices of self are encoded such that the hundreds digit specifies robot number
		 * and lower digits are the number in the sequence for the robot
		 */
		typedef gtsam::TypedSymbol<Opp_t, 'o'> OppKey;
		typedef gtsam::TypedSymbol<Self_t, 's'> SelfKey;

		// Configs
		typedef gtsam::LieConfig<OppKey, Opp_t> OppConfig;
		typedef gtsam::LieConfig<SelfKey, Self_t> SelfConfig;
		typedef gtsam::LieConfig<gtsam::LagrangeKey, Vector> LagrangeConfig;
		typedef gtsam::TupleConfig3<OppConfig, SelfConfig, LagrangeConfig> Config;

		// Graph
		typedef gtsam::NonlinearFactorGraph<Config> Graph;

		// basic constraints
		typedef gtsam::NonlinearEquality<Config, OppKey, Opp_t> OppConstraint;
		typedef gtsam::NonlinearEquality<Config, SelfKey, Self_t> SelfConstraint;

		// basic priors
		typedef gtsam::PriorFactor<Config, SelfKey, Self_t> SelfPrior;
		typedef gtsam::PriorFactor<Config, OppKey, Opp_t> OppPrior;
		typedef gtsam::LagrangePriorFactor<Config, gtsam::LagrangeKey> LagrangePrior;

		// Optimizer
		typedef gtsam::NonlinearOptimizer<Graph, Config> Optimizer;

		// shared pointer versions
		typedef boost::shared_ptr<Graph> shared_graph;
		typedef boost::shared_ptr<Config> shared_config;

		// Conversion functions
		gtsam::Point2 rc2gt_Point2(const Geometry2d::Point& pt);
		Geometry2d::Point gt2rc_Point2(const gtsam::Point2& pt);

		gtsam::Pose2 rc2gt_Pose2(const Geometry2d::Point& pt, float angle);
		std::pair<Geometry2d::Point, float> gt2rc_Pose2(const gtsam::Pose2& pt);

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
			RobotSelfConstraint(uint8_t robotID, size_t frame_num, const Geometry2d::Point& pt, float angle)
				: SelfConstraint(SelfKey(encodeID(robotID, frame_num)), rc2gt_Pose2(pt, angle)) {}
			RobotSelfConstraint(uint8_t robotID, size_t frame_num, const Self_t& pose)
							: SelfConstraint(SelfKey(encodeID(robotID, frame_num)), pose) {}
		};

		class RobotOppConstraint : public OppConstraint {
		public:
			RobotOppConstraint(uint8_t robotID, size_t frame_num, const Geometry2d::Point& pt)
				: OppConstraint(OppKey(encodeID(robotID, frame_num)), rc2gt_Point2(pt)) {}
		};

		/** Point to Segment distances */
		double pointSegmentDist(const gtsam::Point2& start, const gtsam::Point2& end, const gtsam::Point2& pt,
				boost::optional<Matrix&> Dstart = boost::none,
				boost::optional<Matrix&> Dend = boost::none,
				boost::optional<Matrix&> Dpt = boost::none);

	} // \Optimization
} // \Gameplay

