#ifndef PREDICATES_HPP_
#define PREDICATES_HPP_

#include "Predicate.hpp"

/** global predicates */

extern Predicate offense;
extern Predicate defense;

/// states ///
/** waiting for opponent to start */
extern Predicate waiting;
/** free running */
extern Predicate running;
/** setup, cannot touch ball, can approach */
extern Predicate setup;
/** 500 mm ball zone */
extern Predicate stopped;
/** no action */
extern Predicate halt;

/// start actions ///
extern Predicate kickoff;
extern Predicate penalty;

extern Predicate direct;
extern Predicate indirect;

/** true if we are responsible for the start action */
extern Predicate our_action;

extern Predicate winning;
extern Predicate losing;

#endif /* PREDICATES_HPP_ */
