/*
 * OptimizerConfig.hpp
 *
 *  Created on: Dec 9, 2009
 *      Author: alexgc
 */

#pragma once

#include <vector>
//#include <gtsam/VectorConfig.h>
#include <GameplayModule.hpp>
#include <PassState.hpp>

namespace Gameplay {

//	namespace Optimization {
//
//		class OptimizerConfig : public gtsam::VectorConfig {
//			// number of states
//			int states_;
//
//		public:
//			OptimizerConfig();
//			virtual ~OptimizerConfig();
//
//			/** gets various parts */
//			Geometry2d::Point getInit(int idx) const;
//			Geometry2d::Point getFinal(int idx) const;
//			double getTime(int idx) const;
//
//			/**
//			 * Initializes a robot's initial and final positions
//			 *  Must be done in sequence
//			 */
//			void initRobot(int idx, const Geometry2d::Point& init, const Geometry2d::Point& end);
//
//			/**
//			 * Initializes the times for the robot
//			 *  Must be done in sequence
//			 */
//			void initTime(int idx, const PassState& s);
//
//			/** standard print function */
//			virtual void print(const std::string& name) const;
//
//			/**
//			 * equality up to tolerance
//			 */
//			virtual bool equals(const OptimizerConfig& expected, double tol) const;
//
//		    /**
//		     * Add a delta config, needed for use in SQPOptimizer
//		     * For VectorConfig, this is just addition.
//		     */
//		    OptimizerConfig exmap(const VectorConfig & delta) const;
//		};
//
//	}

}

