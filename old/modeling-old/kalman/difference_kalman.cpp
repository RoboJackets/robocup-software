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
 *   Jon S. Olson
 */

#include <stdlib.h>
#include <cstdio>
#include <cblas.h>
#include "../BLASWrap/blaswrap.h"
#include <assert.h>
#include "difference_kalman.hpp"

DifferenceKalmanFilter::DifferenceKalmanFilter(DMatrix* _F, DMatrix* _B,
                                               DVector* _x, DMatrix* _P,
                                               DMatrix* _Q, DMatrix* _R,
                                               DMatrix* _H)
    : x(_x), P(_P), Q(_Q), R(_R), H(_H), F(_F), B(_B) {
    n = x->size();
    m = B->cols();
    s = H->rows();

    assert(F->rows() == n);
    assert(F->cols() == n);
    assert(B->rows() == n);
    assert(B->cols() == m);
    assert(H->rows() == s);
    assert(H->cols() == n);
    assert(P->rows() == n);
    assert(P->cols() == n);
    assert(Q->rows() == n);
    assert(Q->cols() == n);
    assert(R->rows() == s);
    assert(R->cols() == s);

    I_nn = DMatrix::identity(P->rows());

    K = new DMatrix(n, s);

    tmp_n_1 =
        new DVector(n);  // Used in calculating intermediate values of xPrime
    tmp_nn_1 =
        new DMatrix(n, n);  // Used in calculating intermediate values of PPrime
    tmp_nn_2 =
        new DMatrix(n, n);  // Used in calculating intermediate values of PPrime

    tmp_n_2 = new DVector(n);
    tmp_s_1 = new DVector(s);

    tmp_sn_1 = new DMatrix(s, n);
    tmp_ns_1 = new DMatrix(n, s);
    tmp_ss_1 = new DMatrix(s, s);
    tmp_nm_1 = new DMatrix(n, m);

    ipiv = (int*)BW_ALLOC(sizeof(int) * s);
    lwork = tmp_ss_1->invert_size();
    work = (double*)BW_ALLOC(sizeof(double) * lwork);
}

DifferenceKalmanFilter::~DifferenceKalmanFilter() {
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

    BW_FREE(ipiv);
    BW_FREE(work);
}

void DifferenceKalmanFilter::predict(const DVector* u) {
    assert(u->size() == m);

    /* Projection consists of two steps. First we project the new state, then
    * we project covariances for our state vector.
    *
    * We might still be able to do this with the lsim method, I don't know.
    * BLASWrap could use some friendlier docs and a more complete wrapper.
    */
    // project state
    // x_{k-} = F x_{k-1} + B u_{k-1}
    tmp_n_1->mult(F, x);  // F dot x_{k-1}
    x->mult(B, u);        // B dot u_{k-1}
    *x += *tmp_n_1;       // Add to two

    /* Ripped this right from Neil's code. */
    // project error covariance
    // P(k) = F P(k-1)F^T + Q
    *tmp_nn_1 = *F;
    tmp_nn_2->mult(tmp_nn_1, P);     // A P(k-1)
    P->mult_1t(tmp_nn_2, tmp_nn_1);  // (A P(k-1)) A^T
    *P += *Q;                        // ( (A P(k-1)) A^T ) + Q
}

// This is unchanged from the differential version
// Should probably just inherit from the original.
// I would like to do some variable renaming, though, to make it more clear
// what the hell is going on in here.
int DifferenceKalmanFilter::correct(const DVector* z) {
    assert(z->size() == s);
    DMatrix* S = tmp_ss_1;  // Covariance innovation
    DVector* y = tmp_s_1;   // Observation innovation

    // update estimate with measurement
    *y = *z;
    y->gemv(CblasNoTrans, -1, H, x, 1);  // -Hx + z

    // compute kalman gain
    *S = *R;               // Copy the contents of R into S
    tmp_sn_1->mult(H, P);  // H P
    S->gemm(CblasNoTrans, CblasTrans, 1, tmp_sn_1, H, 1);  // (HP) H^T + R
    // C->gemm(transpose_a, transpose_b, alpha, A, B, beta)
    // C = alpha*A*B + beta*C
    // Soooo...
    // S = H*P*H' + R

    assert(S->invert_size() == lwork);
    int inv = S->invert(work, lwork, ipiv, s);
    if (inv) {
        return inv;  // couldn't invert, skip it
    }

    tmp_ns_1->mult_1t(P, H);  // P H^T
    K->mult(tmp_ns_1, S);

    // Update state
    x->gemv(CblasNoTrans, 1, K, y, 1);  // K * (-Hx + z) + x

    // update measurement covariance
    *tmp_nn_1 = *I_nn;
    tmp_nn_1->gemm(CblasNoTrans, CblasNoTrans, -1, K, H, 1);  // I-KH
    tmp_nn_2->mult(tmp_nn_1, P);                              // P = (I-KH) P
    *P = *tmp_nn_2;

    return 0;
}
