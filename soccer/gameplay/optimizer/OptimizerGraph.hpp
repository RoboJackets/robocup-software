/*
 * OptimizerGraph.hpp
 *
 *  Created on: Dec 9, 2009
 *      Author: alexgc
 */

#pragma once

#include <gtsam/Testable.h>
#include <gtsam/NonlinearFactorGraph.h>
#include "OptimizerConfig.hpp"

// implementations
#include <gtsam/FactorGraph-inl.h>

namespace Gameplay {

	namespace Optimization {

		class OptimizerGraph :
			public gtsam::NonlinearFactorGraph<OptimizerConfig> {
		public:
			OptimizerGraph();
			virtual ~OptimizerGraph();

			/** standard print function */
			void print(const std::string& name) const;

			/**
			 * equality up to tolerance
			 */
			bool equals(const OptimizerGraph& expected, double tol) const;

			// Functions to create the graph

			/** Forces the final pos of a robot to intercept a ball */
			void addBallIntercept(int robot_num, const Packet::LogFrame::Ball& ball);

			/** Fixes the initial pos of a robot to a point */
			void addRobotInitConstraint(int robot_num, const Geometry2d::Point& pos);

			/** Connects the init and final pos using the robot's motion model */
			void addRobotMotion(int robot_num, Robot* r);

			/** Keeps the final position on the field */
			void addRobotFieldBound(int robot_num);

			/** Creates a pass between two robots */
			void addPass(int passer, int receiver);

			/** Creates a shot on goal from a robot */
			void addShot(int robot_num);

			};

	}

}
