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


/* Authors:
 *   Neil T. Dantam
 */

#include <stdlib.h>
#include <cstdio>
#include <cblas.h>
#include <BLASWrap/blaswrap.h>
#include <assert.h>
#include "kalman.h"

KalmanFilter::KalmanFilter( DMatrix *_A, DMatrix *_B,
                            DVector *_x, DMatrix *_P, DMatrix *_Q, 
                            DMatrix *_R, DMatrix *_H ) :
  A(_A),
  B(_B),
  x(_x),
  P(_P),
  Q(_Q),
  R(_R),
  H(_H)
{
  n = x->size();
  m = B->cols();
  s = H->rows();

  assert( A->rows() == n );
  assert( A->cols() == n );
  assert( B->rows() == n );
  assert( B->cols() == m );
  assert( H->rows() == s );
  assert( H->cols() == n );
  assert( P->rows() == n );
  assert( P->cols() == n );
  assert( Q->rows() == n );
  assert( Q->cols() == n );
  assert( R->rows() == s );
  assert( R->cols() == s );

  I_nn = DMatrix::identity( P->rows() );

  K = new DMatrix(n,s);

  tmp_n_1 = new DVector( n );
  tmp_n_2 = new DVector( n );
  tmp_s_1 = new DVector( s );

  tmp_sn_1 = new DMatrix( s, n );
  tmp_ns_1 = new DMatrix( n, s );
  tmp_ss_1 = new DMatrix( s, s );
  tmp_nn_1 = new DMatrix( n, n );
  tmp_nm_1 = new DMatrix( n, m );

  ipiv = (int*)BW_ALLOC( sizeof(int) * s );
  lwork = tmp_ss_1->invert_size();
  work = (double*)BW_ALLOC(sizeof(double) * lwork);
}

KalmanFilter::~KalmanFilter() {
  delete I_nn;
  delete tmp_n_1;
  delete tmp_n_2;
  delete tmp_s_1;
  delete tmp_sn_1;
  delete tmp_ns_1;
  delete tmp_ss_1;
  delete tmp_nn_1;
  delete tmp_nm_1;
  delete K;

  BW_FREE( ipiv );
  BW_FREE( work );
}

void KalmanFilter::predict(const DVector *u, double dt) {
  assert( u->size()  == m );
  
  // calculate new A
  tmp_nn_1->add( dt, A, I_nn ); // dt*A + I
  // calculate new B
  tmp_nm_1->scale( B, dt); // dt*B

  // project state
  // x_{k-} = A x_{k-1} + B u_{k-1|
  // really: dx/dt(t) = A x(t-dt) + B u(t-dt) and x(t) = x(t-dt) + dx/dt(t) dt
  x->lsim( A, B, u, dt );
  
  // project error covariance
  // P(k) = A P(k-1)A^T + Q
  P->mult( tmp_nn_1, P ); // A P(k-1)
  P->mult_1t( P, tmp_nn_1 ); // (A P(k-1)) A^T
  *P += *Q; // ( (A P(k-1)) A^T ) + Q
}

int KalmanFilter::correct(const DVector *z) {
  assert( z->size()  == s );
  DMatrix *S = tmp_ss_1;

  // compute kalman gain
  tmp_sn_1->mult( H, P ); // H P
  *S = *R;
  S->gemm(CblasNoTrans, CblasTrans, 1, tmp_sn_1, H, 1); // (HP) H^T + R

  assert( S->invert_size() == lwork );
  int inv = S->invert(work, lwork, ipiv, s);
  if( inv ) {
    return inv; // couldn't invert, skip it
  }

  tmp_ns_1->mult_1t(P, H); // P H^T
  K->mult( tmp_ns_1, S );

  // update estimate with measurement
  *tmp_s_1 = *z;
  tmp_s_1->gemv( CblasNoTrans, -1, H, x, 1 ); // -Hx + z
  x->gemv( CblasNoTrans, 1, K, tmp_s_1, 1 ); // K * (-Hx + z) + x
 

  // update measurement covariance
  *tmp_nn_1 = *I_nn;
  tmp_nn_1->gemm(CblasNoTrans, CblasNoTrans, -1, K, H, 1); //I-KH
  P->mult( tmp_nn_1, P ); // P = (I-KH) P

  return 0;
}
