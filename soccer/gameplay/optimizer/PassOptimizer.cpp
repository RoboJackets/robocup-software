 /*
 * PassOptimizer.cpp
 *
 *  Created on: Dec 9, 2009
 *      Author: alexgc
 */

#include <boost/foreach.hpp>
#include <gtsam/Ordering.h>
#include <PassOptimizer.hpp>
#include <passOptimization.hpp>

// factors
#include <DrivingFactors.hpp>
#include <ShootingFactors.hpp>
#include <PassingFactors.hpp>

using namespace std;
using namespace boost;
using namespace gtsam;
using namespace Gameplay;
using namespace Optimization;
using namespace Geometry2d;

Gameplay::Optimization::PassOptimizer::PassOptimizer(GameplayModule* gameplay)
: gameplay_(gameplay)
{
	// initialize the parameters
	fetchSigma = 1.0;
	passRecSigma = 1.5;
	reaimSigma = 0.5;
	shotLengthSigma = 3.0;
	passLengthSigma = 2.0;
}

Gameplay::Optimization::PassOptimizer::~PassOptimizer() {}

PassConfig Gameplay::Optimization::PassOptimizer::optimizePlan(
		const PassConfig& init, bool verbose) const
{
	cout << "Optimizing plan!" << endl;

	// create graph and config
	shared_config config(new Config());
	shared_graph graph(new Graph());

	// initialize current position for opp robot
	BOOST_FOREACH(Gameplay::Robot * robot, gameplay_->opp) {
		if (robot->visible()) {
			int id = robot->id();
			Point pos = robot->pos();
			graph->add(RobotOppConstraint(id, 1, pos));
			config->insert(OppKey(encodeID(id, 1)), rc2gt_Point2(pos));
		}
	}

	// DEBUGGING: a model for priors
	SharedDiagonal prior_model = noiseModel::Isotropic::Sigma(3, 0.2);


	// store shells for robots so we don't copy them in multiple times
	set<int> self_shells;

	// go through initial passconfig and initialize the graph and the config
	size_t curFrame = 1;
	BOOST_FOREACH(PassState s, init.passStateVector) {
		// get the robots
		Robot * r1 = s.robot1, * r2 = s.robot2;

		// get robot id's
		int r1id = s.robot1->id(),
		    r2id = s.robot2->id();

		// extract robot poses and convert
		Pose2 r1pos = rc2gt_Pose2(s.robot1Pos, s.robot1Rot),
			  r2pos = rc2gt_Pose2(s.robot2Pos, s.robot2Rot);

		switch (s.stateType) {
		case PassState::INTERMEDIATE :
			// only for initialization
			if (curFrame == 1) {
				graph->add(RobotSelfConstraint(r1id, 1, r1->pos(), r1->angle()));
				graph->add(RobotSelfConstraint(r2id, 1, r2->pos(), r2->angle()));
				config->insert(SelfKey(encodeID(r1id, 1)), rc2gt_Pose2(r1->pos(), r1->angle()));
				config->insert(SelfKey(encodeID(r2id, 1)), rc2gt_Pose2(r2->pos(), r2->angle()));

				// remember the shells in use
				self_shells.insert(r1id);
				self_shells.insert(r2id);
			}

			break;
		case PassState::KICKPASS :
			// initialize the fetch state for robot 1
			config->insert(SelfKey(encodeID(r1id, 2)), r1pos);

			// add a prior to avoid large jumps
			graph->add(SelfPrior(SelfKey(encodeID(r1id, 1)), r1pos, prior_model));

			// add driving factors from initial state to final state
			graph->add(PathShorteningFactor(SelfKey(encodeID(r1id, 1)),
											SelfKey(encodeID(r1id, 2)), fetchSigma));
			break;
		case PassState::RECEIVEPASS :
			// initialize with receive state for robot 2
			config->insert(SelfKey(encodeID(r2id, 2)), r2pos);

			// PLACEHOLDER: add a prior to the new pos
			graph->add(SelfPrior(SelfKey(encodeID(r2id, 2)), r2pos, prior_model));

			// add driving factors from initial state to final state
			graph->add(PathShorteningFactor(SelfKey(encodeID(r2id, 1)),
											SelfKey(encodeID(r2id, 2)), passRecSigma));

			// add pass factors
			graph->add(PassShorteningFactor(SelfKey(encodeID(r1id, 2)),
											SelfKey(encodeID(r2id, 2)), passLengthSigma));
			break;
		case PassState::KICKGOAL :
			// initialize reaim state for robot 2
			config->insert(SelfKey(encodeID(r2id, 3)), r2pos);

			// PLACEHOLDER: add a prior to the new pos
			graph->add(SelfPrior(SelfKey(encodeID(r2id, 3)), r2pos, prior_model));

			// add aiming factor from previous state
			graph->add(ReaimFactor(SelfKey(encodeID(r2id, 2)),
								   SelfKey(encodeID(r2id, 3)), reaimSigma));

			// add shooting factor on goal
			graph->add(ShotShorteningFactor(SelfKey(encodeID(r2id, 3)), shotLengthSigma));
			break;
		}
		++curFrame;
	}

	// initialize current positions of robots not involved
	BOOST_FOREACH(Gameplay::Robot * robot, gameplay_->self) {
		int id = robot->id();
		if (robot->visible() && self_shells.find(id) == self_shells.end()) {
			Point pos = robot->pos();
			float angle = robot->angle();
			graph->add(RobotSelfConstraint(id, 1, pos, angle));
			config->insert(SelfKey(encodeID(id, 1)), rc2gt_Pose2(pos, angle));
		}
	}

	// optimize
	shared_ptr<Ordering> ordering(new Ordering(graph->getOrdering()));
	Optimizer::shared_solver solver(new Optimizer::solver(ordering));
	Optimizer optimizer(graph, config, solver);
	double relThresh = 1e-3;
	double absThresh = 1e-3;
	size_t maxIt = 10;

	// execute optimization
	Optimizer result = optimizer.levenbergMarquardt(relThresh, absThresh, Optimizer::SILENT, maxIt);

	// reconstruct a passconfig by starting with initial config
	Point2 defBallPos(Constants::Robot::Radius+Constants::Ball::Radius, 0.0f);
	PassConfig optConfig(init);
	BOOST_FOREACH(PassState &s, optConfig.passStateVector) {
		// get robot id's
		int r1id = s.robot1->id(),
			r2id = s.robot2->id();

		Point r1t, r2t; float r1r, r2r; Pose2 r1pose, r2pose;
		switch (s.stateType) {
		case PassState::INTERMEDIATE :
			// Just provided initial cases anyway, so no change
			break;

		case PassState::KICKPASS :
			// first state for robot 1
			r1pose = result.config()->at(SelfKey(encodeID(r1id, 2)));
			boost::tie(r1t, r1r) = gt2rc_Pose2(r1pose);
			s.robot1Pos = r1t;
			s.robot1Rot = r1r;
			// FIXME: setting the ball pose, likely wrong
			s.ballPos = gt2rc_Point2(r1pose * defBallPos);
			break;

		case PassState::RECEIVEPASS :
			// receive state for robot 2
			r2pose = result.config()->at(SelfKey(encodeID(r2id, 2)));
			boost::tie(r2t, r2r) = gt2rc_Pose2(r2pose);
			s.robot2Pos = r2t;
			s.robot2Rot = r2r;
			// FIXME: setting the ball pose, likely wrong
			s.ballPos = gt2rc_Point2(r2pose * defBallPos);
			break;

		case PassState::KICKGOAL :
			// reaim state for robot 2
			r2pose = result.config()->at(SelfKey(encodeID(r2id, 3)));
			boost::tie(r2t, r2r) = gt2rc_Pose2(r2pose);
			s.robot2Pos = r2t;
			s.robot2Rot = r2r;
			// FIXME: setting the ball pose, likely wrong
			s.ballPos = gt2rc_Point2(r2pose * defBallPos);
			break;
		}
	}
	cout << "   Finished optimizing plan" << endl;

	return optConfig;
}
