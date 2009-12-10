/*
 * OptimizerConfig.hpp
 *
 *  Created on: Dec 9, 2009
 *      Author: alexgc
 */

#pragma once

namespace Gameplay {

	namespace Optimization {

		class OptimizerConfig {
		public:
			OptimizerConfig();
			virtual ~OptimizerConfig();

			/** standard print function */
			virtual void print(const std::string& name) const;

			/**
			 * equality up to tolerance
			 */
			virtual bool equals(const OptimizerConfig& expected, double tol) const;
		};

	}

}

