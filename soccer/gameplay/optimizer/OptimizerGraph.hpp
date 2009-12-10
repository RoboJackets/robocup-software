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
		};

	}

}
