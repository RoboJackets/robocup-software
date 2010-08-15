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

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include "RbpfModel.hpp"

#include <Configuration.hpp>

// kicked ball with high noise, ignores control input
// Note: could be written to accept control input from our robots
// state: X (6 x 1) = {x, y, vx, vy, ax, ay}
class RbpfModelKicked : public RbpfModel {
public:
	typedef boost::numeric::ublas::vector<double> Vector;
	typedef boost::numeric::ublas::matrix<double> Matrix;
	RbpfModelKicked(Modeling::RobotModel::RobotMap *_robotMap, Configuration *config);
	~RbpfModelKicked();

	// reinitialize the parameters from the config files - should be called each frame
	void initParams();
protected:
	ConfigDouble _processNoiseSqrdPos;
	ConfigDouble _processNoiseSqrdVel;
	ConfigDouble _processNoiseSqrdAcc;
	ConfigDouble _measurementNoiseSqrd;
	
	void transitionModel(Vector &X, Vector &U, double dt);
	void computeTransitionJacobian(double dt);
	void observationModel(Vector &X, Vector &out);
	void computeObservationJacobian(double dt);
	void update(Vector &X, Matrix &P, Vector &Z, double dt);

	// initialization functions to pull from config file
	virtual void initializeQ();
	virtual void initializeR();
};

#endif /* RBPFMODELKICKED_HPP_ */
