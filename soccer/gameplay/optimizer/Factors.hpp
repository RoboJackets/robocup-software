/*
 * Factors.hpp
 *
 *  Created on: Dec 10, 2009
 *      Author: alexgc
 */

#pragma once

#include <Vector.h>
#include <Factor.h>
#include <NonlinearFactor.h>
#include <GaussianFactor.h>
#include <Matrix.h>
#include <OptimizerConfig.hpp>

namespace Gameplay {

	namespace Optimization {
		/** Parts we need in order to build simple soft factors */
		Vector unary (const Vector& x);
		Matrix Dunary(const Vector& x);

		/** Constraint components for field bounds */
		namespace field_bound {
			/** p = 1, g(x,y) = [abs(x)-0.5*width abs(y-0.5*length)-0.5*length] */
			Vector g_func(const float min, const float max, int idx,
					const OptimizerConfig& config, const std::list<std::string>& keys);

			/** just one in the direction towards the field */
			Matrix grad_g(float middle, int idx,
					const OptimizerConfig& config, const std::list<std::string>& keys);
		}

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

		/**
		 * a Gaussian nonlinear factor that shortens the distance between two points
		 */
		class ShorteningFactor : public gtsam::NonlinearFactor<OptimizerConfig> {

		private:

			std::string key1_;
			std::string key2_;

		public:

			/** Constructor */
			ShorteningFactor(
					const double sigma,	   // the weight
					const std::string& key1,  // key of the first variable
					const std::string& key2); // key of the second variable

			/** Print */
			void print(const std::string& s = "") const;

			/** Check if two factors are equal */
			bool equals(const ShorteningFactor& f, double tol=1e-9) const;

			/** error function */
			inline Vector error_vector(const OptimizerConfig& c) const {
				return c[key1_] - c[key2_];
			}

			/** Linearize a non-linearFactor2 to get a linearFactor2 */
			boost::shared_ptr<gtsam::GaussianFactor> linearize(const OptimizerConfig& c) const;
		};
	}
}
