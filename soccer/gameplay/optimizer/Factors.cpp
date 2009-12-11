/*
 * Factors.cpp
 *
 *  Created on: Dec 10, 2009
 *      Author: alexgc
 */

#include "Factors.hpp"


using namespace gtsam;
using namespace std;

/** Functions for Unary Factors */
Vector Gameplay::Optimization::unary(const Vector& x) {
	return x;
}
Matrix Gameplay::Optimization::Dunary(const Vector& x) {
	return eye(x.size());
}

using namespace Gameplay;
using namespace Optimization;

/* ************************************************************************* */
// Implementation of a slightly specified basic factor
/* ************************************************************************* */
BasicUnaryFactor::BasicUnaryFactor(const Vector& z,
		const double sigma,
		Vector (*h)(const Vector&),
		const string& key1,
		Matrix (*H)(const Vector&))
: NonlinearFactor<OptimizerConfig>(z, sigma), key_(key1),h_(h),  H_(H)
  {
	keys_.push_front(key1);
  }

/* ************************************************************************* */
void BasicUnaryFactor::print(const string& s) const {
	cout << "BasicUnaryFactor " << s << endl;
	cout << "h  : " << (void*)h_ << endl;
	cout << "key: " << key_      << endl;
	cout << "H  : " << (void*)H_ << endl;
	NonlinearFactor<OptimizerConfig>::print("parent");
}

/* ************************************************************************* */
GaussianFactor::shared_ptr BasicUnaryFactor::linearize(const OptimizerConfig& c) const {
	// get argument 1 from config
	Vector x1 = c[key_];

	// Jacobian A = H(x1)/sigma
	Matrix A = H_(x1);

	// Right-hand-side b = error(c) = (z - h(x1))/sigma
	Vector b = (z_ - h_(x1));

	GaussianFactor::shared_ptr p(new GaussianFactor(key_, A, b, sigma_));
	return p;
}

/* ************************************************************************* */
bool BasicUnaryFactor::equals(const BasicUnaryFactor& f, double tol) const {
	const BasicUnaryFactor* p = dynamic_cast<const BasicUnaryFactor*> (&f);
	if (p == NULL) return false;
	return NonlinearFactor<OptimizerConfig>::equals(*p, tol)
			&& (h_   == p->h_)
			&& (key_ == p->key_)
			&& (H_   == p->H_);
}

/* ************************************************************************* */
// Factor to shorten the distance between points
/* ************************************************************************* */
ShorteningFactor::ShorteningFactor(
		const double sigma,	   // the weight
		const std::string& key1,  // key of the first variable
		const std::string& key2)  // key of the second variable
: NonlinearFactor<OptimizerConfig>(zero(2), sigma), key1_(key1), key2_(key2)
  {
	keys_.push_front(key1);
	keys_.push_front(key2);
  }

/* ************************************************************************* */
void ShorteningFactor::print(const string& s) const {
	cout << "ShorteningFactor " << s << endl;
	cout << "key1: " << key1_      << endl;
	cout << "key2: " << key2_      << endl;
	NonlinearFactor<OptimizerConfig>::print("parent");
}

/* ************************************************************************* */
GaussianFactor::shared_ptr ShorteningFactor::linearize(const OptimizerConfig& c) const {
	// get arguments from config
	Vector x1 = c[key1_];
	Vector x2 = c[key2_];

	// Jacobian A = H(x)/sigma
	Matrix A1 = eye(2);
	Matrix A2 = -1*eye(2);

	// Right-hand-side b = (z - h(x))/sigma
	Vector b = -(x1-x2);

	GaussianFactor::shared_ptr p(new GaussianFactor(key1_, A1, key2_, A2, b, sigma_));
	return p;
}

/* ************************************************************************* */
bool ShorteningFactor::equals(const ShorteningFactor& f, double tol) const {
	const ShorteningFactor* p = dynamic_cast<const ShorteningFactor*> (&f);
	if (p == NULL) return false;
	return NonlinearFactor<OptimizerConfig>::equals(*p, tol)
    		&& (key1_ == p->key1_)
    		&& (key2_ == p->key2_);
}

/* ************************************************************************* */
// Constraint components for field bounds
/* ************************************************************************* */

/** p = 1, g(x) = -1 if good, 1 if bad */
Vector field_bound::g_func(const float min, const float max, int idx,
		const OptimizerConfig& config, const list<string>& keys) {
	Vector pt = config.get(keys.front());
	if (pt(idx) < max && pt(idx) > min)
		return Vector_(1, -1.0);
	else
		return Vector_(1, 1.0);
}

/** just one in the direction towards the field */
Matrix field_bound::grad_g(float middle, int idx, const OptimizerConfig& config, const list<string>& keys) {
	Vector pt = config.get(keys.front());
	double dx, x=pt(idx);
	if (x > middle)
		dx = -1.0;
	else
		dx = 1.0;
	return Matrix_(1,1,dx);
}

