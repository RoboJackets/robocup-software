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
		typedef gtsam::TypedSymbol<gtsam::Point2, 'i'> InitialKey;
		typedef gtsam::TypedSymbol<gtsam::Point2, 'f'> FinalKey;
		typedef gtsam::TypedSymbol<gtsam::Point2, 'b'> BallKey;

		// Configs
		typedef gtsam::LieConfig<OppKey, gtsam::Point2> OppConfig;
		typedef gtsam::LieConfig<InitialKey, gtsam::Point2> InitialConfig;
		typedef gtsam::LieConfig<FinalKey, gtsam::Point2> FinalConfig;
		typedef gtsam::LieConfig<BallKey, gtsam::Point2> BallConfig;
		typedef gtsam::TupleConfig4<OppConfig, InitialConfig, FinalConfig, BallConfig> Config;

		// Graph
		typedef gtsam::NonlinearFactorGraph<Config> Graph;

		// basic constraints
		typedef gtsam::NonlinearEquality<Config, OppKey, gtsam::Point2> OppConstraint;
		typedef gtsam::NonlinearEquality<Config, InitialKey, gtsam::Point2> InitConstraint;
		typedef gtsam::NonlinearEquality<Config, FinalKey, gtsam::Point2> FinalConstraint;
		typedef gtsam::NonlinearEquality<Config, BallKey, gtsam::Point2> BallConstraint;

		// Optimizer
		typedef gtsam::NonlinearOptimizer<Graph, Config> Optimizer;

		// shared pointer versions
		typedef boost::shared_ptr<Graph> shared_graph;
		typedef boost::shared_ptr<Config> shared_config;

		// Conversion functions
		gtsam::Point2 gt2rc_Point2(const Geometry2d::Point& pt);
		Geometry2d::Point rc2gt_Point2(const gtsam::Point2& pt);

	} // \Optimization
} // \Gameplay

