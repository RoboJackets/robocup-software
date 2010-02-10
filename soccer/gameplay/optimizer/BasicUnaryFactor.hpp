#pragma once

#include <NonlinearFactor.h>
#include <OptimizerConfig.hpp>

namespace Gameplay {
	namespace Optimization {
		/**
		 * a Gaussian nonlinear factor that takes 1 parameter
		 * Note: cannot be serialized as contains function pointers
		 * Specialized derived classes could do this
		 */
		class BasicUnaryFactor : public gtsam::NonlinearFactor<OptimizerConfig> {
		private:

			std::string key_;

		public:

			Vector (*h_)(const Vector&);
			Matrix (*H_)(const Vector&);

			/** Constructor */
			BasicUnaryFactor(const Vector& z,		  // measurement
					const double sigma,	  			  // variance
					Vector (*h)(const Vector&),       // measurement function
					const std::string& key1,          // key of the variable
					Matrix (*H)(const Vector&));      // derivative of the measurement function

			void print(const std::string& s = "") const;

			/** Check if two factors are equal */
			bool equals(const BasicUnaryFactor& f, double tol=1e-9) const;

			/** error function */
			inline Vector error_vector(const OptimizerConfig& c) const {
				return z_ - h_(c[key_]);
			}

			/** linearize a non-linearFactor1 to get a linearFactor1 */
			boost::shared_ptr<gtsam::GaussianFactor> linearize(const OptimizerConfig& c) const;
		};
	}
}
