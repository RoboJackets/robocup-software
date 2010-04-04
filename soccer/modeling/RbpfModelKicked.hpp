/*
 * RbpfModelKicked.hpp
 *
 *  Kicked model, implements update() to check for possible kicks
 *
 *  Because all models require a constant state size, the state size for
 *  all models is defined at the top of RbpfModel.hpp, and must be constant
 *  across all models.  You may mix model sizes (i.e., 4D, 6D, etc.) in this
 *  file, but all the models instantiated at runtime must have sizes that
 *  correspond to those defined in RbpfModel.hpp.
 *
 *  Author: Philip Rogers, Nov 19th 2009
 */

#ifndef RBPFMODELKICKED_HPP_
#define RBPFMODELKICKED_HPP_

#include <iostream>
#include <assert.h>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include "RbpfModel.hpp"

// kicked ball with high noise, ignores control input
// Note: could be written to accept control input from our robots
// state: X (6 x 1) = {x, y, vx, vy, ax, ay}
class RbpfModelKicked : public RbpfModel {
public:
	typedef boost::numeric::ublas::vector<double> Vector;
	typedef boost::numeric::ublas::matrix<double> Matrix;
	RbpfModelKicked(Modeling::RobotModel::RobotMap *_robotMap);
	~RbpfModelKicked();
protected:
	void transitionModel(Vector &X, Vector &U, double dt);
	void computeTransitionJacobian(double dt);
	void observationModel(Vector &X, Vector &out);
	void computeObservationJacobian(double dt);
	void update(Vector &X, Matrix &P, Vector &Z, double dt);
};

#endif /* RBPFMODELKICKED_HPP_ */
