/*
 * PassState.cpp
 *
 *  Created on: Nov 29, 2009
 *      Author: Philip Rogers
 */

#include <gameplay/plays/test_plays/PassState.hpp>

PassState::PassState(const Geometry2d::Point* _bP) : ballPos(_bP->x,_bP->y) {

}

PassState::PassState(const Geometry2d::Point* _bP, Robot* _cR) : ballPos(_bP->x,_bP->y), controllingRobot(_cR) {

}


PassState::~PassState() {
	// TODO Auto-generated destructor stub
}
