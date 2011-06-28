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
#include <LinearAlgebra.hpp>
#include "RbpfModel.hpp"

#include <Configuration.hpp>

// free rolling ball, ignores control input
// state: X (6 x 1) = {x, y, vx, vy, ax, ay}
class RbpfModelRolling : public RbpfModel {
public:
	RbpfModelRolling(Modeling::RobotModel::RobotMap *_robotMap, Configuration *config);
	virtual ~RbpfModelRolling();

	// reinitialize the parameters from the config files - should be called each frame
	void initParams();

protected:
	ConfigDouble::shared_ptr _processNoiseSqrdPos;
	ConfigDouble::shared_ptr _processNoiseSqrdVel;
	ConfigDouble::shared_ptr _processNoiseSqrdAcc;
	ConfigDouble::shared_ptr _measurementNoiseSqrd;
	
	void transitionModel(rbpf::VectorNd &X, const rbpf::VectorMd &U, double dt) const;
	void computeTransitionJacobian(double dt);
	void observationModel(const rbpf::VectorNd &X, rbpf::VectorSd &out) const;
	void computeObservationJacobian(double dt);

	// initialization functions to pull from config file
	virtual void initializeQ();
	virtual void initializeR();
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif /* RBPFMODELROLLING_HPP_ */
