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
 *  \author  Neil T. Dantam
 */


#pragma once

#include <LinearAlgebra.hpp>

/** Discrete time or Euler Approximated Kalman Filter 
 */
class KalmanFilter {

 public:
  /** Constructor.  Note that this method WILL MALLOC.  That means you
      probably shouldn't go constructing these objects willy-nilly in
      realtime code.

      Dimensions: n is the number of states, m is the number of
      inputs, s in the number of measurements.

      \param _A dynamic system A matrix, \f${\bf A} \in \Re^{n\times n}\f$
      \param _B dynamic system B matrix, \f${\bf B} \in \Re^{n\times m}\f$
      \param _x initial system state and storage for future system state, \f${\bf x} \in \Re^{n}\f$
      \param _P initial error covariance and storage for computed error covariance, \f${\bf P} \in \Re^{n\times n}\f$
      \param _Q process noise covariance, \f${\bf Q} \in \Re^{n\times n}\f$
      \param _R measurement noise covariance, \f${\bf R} \in \Re^{s\times s}\f$
      \param _H observation model: z = H x, \f${\bf H} \in \Re^{s\times n}\f$
   */
  KalmanFilter( LinAlg::Matrix _A, LinAlg::Matrix _B, LinAlg::Vector _x,
                LinAlg::Matrix _P, LinAlg::Matrix _Q, LinAlg::Matrix _R, LinAlg::Matrix _H );
  /** Destructor.

      Frees all tmps that were malloc'ed by the constructor.  Does not
      free the matrices and vectors passed to the constructor.
   */
  ~KalmanFilter();

 
  /** Euler approximated prediction
   */
  void predict(const LinAlg::Vectoru, double dt);
  /** Kalman correction step.
   */
  int correct(const LinAlg::Vectorz);

  /**Returns pointer to the state vector.*/
  LinAlg::Vector state() { return x; }
  
 protected:
 
  /// estimated state
  LinAlg::Vectorx;
  /// error covariance
  LinAlg::Matrix P;
  /// process noise covariance
  LinAlg::Matrix Q;
  /// measurement noise covariance
  LinAlg::Matrix R;

  /// observation model (z = H x)
  LinAlg::Matrix H;

  /// System A matrix
  LinAlg::Matrix A;
  /// System B matrix
  LinAlg::Matrix B;

  /// computed kalman gain
  LinAlg::Matrix K;
  
  // tmps
  
  LinAlg::Vector tmp_n_1; ///< tmp var
  LinAlg::Vector tmp_n_2; ///< tmp var
  LinAlg::Vector tmp_s_1; ///< tmp var

  LinAlg::Matrix tmp_sn_1;  ///< tmp var
  LinAlg::Matrix tmp_ns_1;  ///< tmp var
  LinAlg::Matrix tmp_ss_1;  ///< tmp var
  LinAlg::Matrix tmp_nn_1;  ///< tmp var
  LinAlg::Matrix tmp_nm_1;  ///< tmp var

  int ipiv; ///< tmp var for pivots
  int lwork; ///< length of ipiv
  double work; ///< tmp var for inversion work

  /// Identity matrix
  LinAlg::Matrix I_nn;

  /// length of state vector, x
  int n; 
  /// length of input vector, u
  int m;
  /// length of measurement vector, z
  int s;
};
