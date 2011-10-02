/*
 * Copyright (c) 2008, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \mainpage Kalman
 */

/** \file kalman_doc.h
 *  \author  Jon S. Olson
 */


#ifndef DIFFERENCE_KALMAN_H_
#define DIFFERENCE_KALMAN_H_

#include "../BLASWrap/blaswrap.h"

#ifdef __cplusplus


/** Discrete time or Kalman Filter with fixed timestep
 */
class DifferenceKalmanFilter {

 public:
  /** Constructor.  Note that this method WILL MALLOC.  That means you
      probably shouldn't go constructing these objects willy-nilly in
      realtime code.

      Dimensions: n is the number of states, m is the number of
      inputs, s in the number of measurements.

      \param _F state transition matrix
      \param _B control transformation matrix
      \param _x initial system state (if known) or an empty n-element column vector
      \param _P initial error covariance and storage for computed error covariance, \f${\bf P} \in \Re^{n\times n}\f$
      \param _Q process noise covariance, \f${\bf Q} \in \Re^{n\times n}\f$
      \param _R measurement noise covariance, \f${\bf R} \in \Re^{s\times s}\f$
      \param _H observation model: z = H x, \f${\bf H} \in \Re^{s\times n}\f$
   */
  DifferenceKalmanFilter( DMatrix *_F, DMatrix *_B, DVector *_x,
                DMatrix *_P, DMatrix *_Q, DMatrix *_R, DMatrix *_H );
  /** Destructor.

      Frees all tmps that were malloc'ed by the constructor.  Does not
      free the matrices and vectors passed to the constructor.
   */
  ~DifferenceKalmanFilter();


  /** Linear forward prediction
   */
  void predict(const DVector *u);
  /** Kalman correction step.
   */
  int correct(const DVector *z);

  /**Returns pointer to the state vector.*/
  DVector *state() { return x; }

 protected:

  /// estimated state (after update)
  DVector *x;
  /// error covariance (after update)
  DMatrix *P;
  /// process noise covariance
  DMatrix *Q;
  /// measurement noise covariance
  DMatrix *R;

  /// observation model (z = H x)
  DMatrix *H;

  /// System transition model
  DMatrix *F;
  /// Control transition model
  DMatrix *B;

  /// computed kalman gain
  DMatrix *K;

  /// predicted state (prior to update)
  DVector *xPrime;

  /// predicted estimate covariance (prior to update)
  DMatrix *PPrime;

  /// Measurement innovation (i.e., error)
  DVector *y;

  /// Covariance innovation
  DMatrix *S;

  // tmps

  DVector *tmp_n_1; ///< tmp var
  DVector *tmp_n_2; ///< tmp var
  DVector  *tmp_s_1; ///< tmp var

  DMatrix *tmp_sn_1;  ///< tmp var
  DMatrix *tmp_ns_1;  ///< tmp var
  DMatrix  *tmp_ss_1;  ///< tmp var
  DMatrix  *tmp_nn_1;  ///< tmp var
  DMatrix  *tmp_nn_2;  ///< tmp var
  DMatrix  *tmp_nm_1;  ///< tmp var

  int *ipiv; ///< tmp var for pivots
  unsigned int lwork; ///< length of ipiv
  double *work; ///< tmp var for inversion work

  /// Identity matrix
  DMatrix *I_nn;

  /// length of state vector, x
  unsigned int n;
  /// length of input vector, u
  unsigned int m;
  /// length of measurement vector, z
  unsigned int s;
};

#endif

#endif
