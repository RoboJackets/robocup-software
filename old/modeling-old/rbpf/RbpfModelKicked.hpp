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

#include "RbpfModel.hpp"
#include <LinearAlgebra.hpp>

#include <Configuration.hpp>

// kicked ball with high noise, ignores control input
// Note: could be written to accept control input from our robots
// state: X (6 x 1) = {x, y, vx, vy, ax, ay}
class RbpfModelKicked : public RbpfModel {
public:
	RbpfModelKicked(Modeling::RobotModel::RobotMap *_robotMap, Configuration *config);
	virtual ~RbpfModelKicked();

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

	void update(rbpf::VectorNd &X, rbpf::MatrixNNd &P, const rbpf::VectorSd &Z, double dt);

	// initialization functions to pull from config file
	virtual void initializeQ();
	virtual void initializeR();
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif /* RBPFMODELKICKED_HPP_ */
