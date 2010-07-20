/*
 * RbpfModelRolling.hpp
 *
 *  Rolling model, assuming no friction.
 *
 *  Because all models require a constant state size, the state size for
 *  all models is defined at the top of RbpfModel.hpp, and must be constant
 *  across all models.  You may mix model sizes (i.e., 4D, 6D, etc.) in this
 *  file, but all the models instantiated at runtime must have sizes that
 *  correspond to those defined in RbpfModel.hpp.
 *
 *  Author: Philip Rogers, Nov 19th 2009
 */

#ifndef RBPFMODELROLLING_HPP_
#define RBPFMODELROLLING_HPP_

#include <iostream>
#include <assert.h>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include "RbpfModel.hpp"

// free rolling ball, ignores control input
// state: X (6 x 1) = {x, y, vx, vy, ax, ay}
class RbpfModelRolling : public RbpfModel {
public:
	typedef boost::numeric::ublas::vector<double> Vector;
	typedef boost::numeric::ublas::matrix<double> Matrix;
	RbpfModelRolling(Modeling::RobotModel::RobotMap *_robotMap, ConfigFile::shared_worldmodel& cfg);
	~RbpfModelRolling();

	// reinitialize the parameters from the config files - should be called each frame
	void initParams();

protected:
	void transitionModel(Vector &X, Vector &U, double dt);
	void computeTransitionJacobian(double dt);
	void observationModel(Vector &X, Vector &out);
	void computeObservationJacobian(double dt);
	ConfigFile::shared_worldmodel _config;

	// initialization functions to pull from config file
	virtual void initializeQ();
	virtual void initializeR();
};

#endif /* RBPFMODELROLLING_HPP_ */
