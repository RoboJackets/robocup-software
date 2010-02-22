/**
 *  @file  PriorFactor.h
 *  @authors Frank Dellaert
 **/
#pragma once

#include <ostream>
#include <gtsam/NoiseModel.h>
#include <gtsam/NonlinearFactor.h>
#include <gtsam/Pose2.h>

namespace gtsam {

	/**
	 * A class for a soft prior on any Lie type
	 * T is the Lie group type, Config where the T's are gotten from
	 */
	template<class Config, class Key>
	class LagrangePriorFactor: public NonlinearFactor1<Config, Key, Vector> {

	private:

		typedef NonlinearFactor1<Config, Key, Vector> Base;

		Vector prior_; /** The measurement */

	public:

		// shorthand for a smart pointer to a factor
		typedef typename boost::shared_ptr<LagrangePriorFactor> shared_ptr;

		/** Constructor */
		LagrangePriorFactor(const Key& key, const Vector& prior,
				const SharedGaussian& model) :
			Base(model, key), prior_(prior) {
		}

		/** implement functions needed for Testable */

		/** print */
		void print(const std::string& s) const {
			Base::print(s);
			gtsam::print(prior_, "prior");
		}

		/** equals */
		bool equals(const NonlinearFactor<Config>& expected, double tol) const {
			const LagrangePriorFactor<Config, Key> *e = dynamic_cast<const LagrangePriorFactor<
					Config, Key>*> (&expected);
			return e != NULL && Base::equals(expected) && this->prior_.equals(
					e->prior_, tol);
		}

		/** implement functions needed to derive from Factor */

		/** vector of errors */
		Vector evaluateError(const Vector& p, boost::optional<Matrix&> H = boost::none) const {
			if (H) (*H) = eye(dim(p));
			// manifold equivalent of h(x)-z -> log(z,h(x))
			return logmap(prior_, p);
		}
	};

} /// namespace gtsam
