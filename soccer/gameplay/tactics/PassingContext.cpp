
#include "PassingContext.hpp"
#include <gameplay/evaluation/ReceivePointEvaluator.hpp>


using namespace std;
using namespace Geometry2d;
using namespace Gameplay;


namespace Gameplay {
	REGISTER_CONFIGURABLE(PassingContext);
}



ConfigDouble *Gameplay::PassingContext::_receiverChoiceHysteresis;

void Gameplay::PassingContext::createConfiguration(Configuration *cfg) {
	_receiverChoiceHysteresis = new ConfigDouble(cfg, "PassingContext/Receiver Choice Hysteresis", .2);	//	TODO: ?
}


Gameplay::PassingContext::PassingContext(GameplayModule *gameplay, StablePass *passer) : _pdt(gameplay, passer) {
	setPasser(passer);
	_passDone = false;
	_gameplay = gameplay;
	_chosenReceiver = NULL;
}


bool Gameplay::PassingContext::run() {

	if ( !_passDone ) {
		//	see if we're done
		vector<DumbReceive *>::iterator itr;
		for ( itr = _receivers.begin(); itr != _receivers.end(); itr++ ) {
			DumbReceive *rcvr = *itr;
			if ( rcvr->isDone() ) _passDone = true;
		}
	}


	if ( !_passDone ) {
		chooseReceiver();	//	update receiver choice if necessary

		//	disable avoid ball if we're allowed to kick
		const GameState &gs = _gameplay->state()->gameState;
		if ( gs.canKick() && _passer && _passer->robot ) {
			_passer->robot->disableAvoidBall();
		}


		//	run the passer
		_pdt.backoff.robots.clear();
		if ( _passer && _passer->robot ) {
			_pdt.backoff.robots.insert(_passer->robot);
			_pdt.run();
		}


		//	run all the receivers
		vector<DumbReceive *>::iterator itr;
		for ( itr = _receivers.begin(); itr != _receivers.end(); itr++ ) {
			DumbReceive *rcvr = *itr;
			rcvr->run();
		}

		_setAllReceiverDribblers(60);
		_unkickAllReceivers();
	}
	

	return !_passDone && _pdt.keepRunning();
}

void Gameplay::PassingContext::_unkickAllReceivers() {
	vector<DumbReceive *>::iterator itr;
	for ( itr = _receivers.begin(); itr != _receivers.end(); itr++ ) {
		DumbReceive *rcvr = *itr;
		if ( rcvr->robot ) {
			rcvr->robot->unkick();
		}
	}
}

void Gameplay::PassingContext::_setAllReceiverDribblers(uint8_t dSpeed) {
	vector<DumbReceive *>::iterator itr;
	for ( itr = _receivers.begin(); itr != _receivers.end(); itr++ ) {
		DumbReceive *rcvr = *itr;
		if ( rcvr->robot ) {
			rcvr->robot->dribble(dSpeed);
		}
	}
}

bool Gameplay::PassingContext::done() {
	return _passDone;
}



void Gameplay::PassingContext::setReceivePointForReceiver(DumbReceive *rcvr, Point &rcvPt) {
	rcvr->actionTarget = rcvPt;

	if ( _chosenReceiver == rcvr && _passer ) {
		_passer->actionTarget = rcvPt;
	}
}

void Gameplay::PassingContext::chooseReceivePointForReceiverAlongSegment(DumbReceive *receiver, Segment &segment) {
	float score = -1;
	Point pt = segment.center();
	if ( receiver->robot ) pt = ReceivePointEvaluator::FindReceivingPoint(_gameplay->state(), receiver->robot->pos, _gameplay->state()->ball.pos, segment, &score);

	setReceivePointForReceiver(receiver, pt);
	setReceiverScore(receiver, score);
}


void Gameplay::PassingContext::setReceiverScore(DumbReceive *receiver, float score) {
	_scoresByReceiver[receiver] = score;
}

void Gameplay::PassingContext::updateReceiverScoresBasedOnChannelWidth() {
	if ( !_passer || !_passer->robot ) return;

	vector<DumbReceive *>::iterator itr;
	for ( itr = _receivers.begin(); itr != _receivers.end(); itr++ ) {
		OurRobot *rcvBot = (*itr)->robot;
		if ( rcvBot ) {
			Segment passChannel((*itr)->actionTarget, _gameplay->state()->ball.pos);
			float channelWidth = ReceivePointEvaluator::ComputePassChannelWidth(_gameplay->state(), passChannel, rcvBot, _passer->robot);
			_scoresByReceiver[*itr] = channelWidth;		
		} else {
			_scoresByReceiver[*itr] = -1;
		}
	}
}


//	performs the actual assignment
void Gameplay::PassingContext::_chooseReceiver(DumbReceive *rcvr) {
	_chosenReceiver = rcvr;

	_updatePairing();
}

void Gameplay::PassingContext::chooseReceiver(DumbReceive *rcvr) {
	if ( rcvr ) {
		_chooseReceiver(rcvr);
	} else {
		float score;
		DumbReceive *currentBest = currentBestReceiver(&score);

		if ( _chosenReceiver ) {
			float prevScore = _scoresByReceiver[_chosenReceiver];
			if ( score - prevScore > *_receiverChoiceHysteresis ) _chooseReceiver(rcvr);
		} else {
			_chooseReceiver(currentBest);
		}
	}
}


DumbReceive *Gameplay::PassingContext::currentBestReceiver(float *out_score) {
	DumbReceive *bestRcvr = NULL;
	float maxScore = -1;

	map<DumbReceive *, float>::iterator itr;
	for ( itr = _scoresByReceiver.begin(); itr != _scoresByReceiver.end(); itr++ ) {
		DumbReceive *rcvr = itr->first;
		float score = itr->second;

		if ( score >= maxScore ) {
			bestRcvr = rcvr;
			maxScore = score;
		}
	}

	if ( out_score ) *out_score = maxScore;

	return bestRcvr;
}

DumbReceive *Gameplay::PassingContext::currentChosenReceiver() {
	return _chosenReceiver;
}



void Gameplay::PassingContext::setPasser(StablePass *passer) {
	_passer = passer;

	_updatePairing();
}

void Gameplay::PassingContext::addReceiver(DumbReceive *rcvr) {
	_receivers.push_back(rcvr);
}

void Gameplay::PassingContext::removeReceiver(DumbReceive *rcvr) {
	_receivers.erase( find(_receivers.begin(), _receivers.end(), rcvr) );

	if ( _chosenReceiver == rcvr ) {
		_chosenReceiver = NULL;
		_updatePairing();
	}
}


void Gameplay::PassingContext::_updatePairing() {
	
	//	clear partner for each receiver that's not the chosen one
	vector<DumbReceive *>::iterator itr;
	for ( itr = _receivers.begin(); itr != _receivers.end(); itr++ ) {
		DumbReceive *rcvr = *itr;
		if ( rcvr != _chosenReceiver ) rcvr->partner = NULL;
	}

	//	TODO: if ( _receiver2.robot ) _receiver2.robot->addText("Dummy");

	if ( _chosenReceiver ) {
		_chosenReceiver->partner = _passer;
	}


	if ( _passer ) {
		_passer->partner = _chosenReceiver;

		//	if we have a receiver, target it
		if ( _chosenReceiver ) {
			_passer->actionTarget = _chosenReceiver->actionTarget;
		}
	}
}
