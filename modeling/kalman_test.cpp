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
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * <copyright holder> BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */


/* kalman filter test program for spring mass system.
 *
 * Authors:
 *   Neil T. Dantam
 */

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <cblas.h>
#include <BLASWrap/blaswrap.h>
#include <assert.h>
#include "kalman.h"
#include "difference_kalman.hpp"
#include <math.h>

double dt = 0.001;
double t_0 = 0;
double t_1 = 30;


DMatrix A(4,4);
DMatrix B(4,4);

DVector x_0(4);
DVector x_a(4);
DMatrix P(4,4);
DMatrix Q(4,4);
DMatrix R(4,4);
DMatrix H(4,4);

DVector z(4);
DVector u(4);
DVector e(4);

DifferenceKalmanFilter kf( &A, &B, &x_0, &P, &Q, &R, &H );
//KalmanFilter kf( &A, &B, &x_k, &P, &Q, &R, &H );

int main( int argc, char **argv ) {
  FILE *f_sys = fopen("sys.csv", "w");
  assert(f_sys);
  DMatrix &I = * DMatrix::identity(4);

  A(0,0) = 1;    A(0,1) = dt;    A(0,2) = 0;    A(0,3) = 0;
  A(1,0) = 0;    A(1,1) = 1;     A(1,2) = 0;    A(1,3) = 0;
  A(2,0) = 0;    A(2,1) = 0;     A(2,2) = 1;    A(2,3) = dt;
  A(3,0) = 0;    A(3,1) = 0;     A(3,2) = 0;    A(3,3) = 1;

  B(0,0) = 0;    B(0,1) = dt;    B(0,2) = 0;    B(0,3) = 0;
  B(1,0) = 0;    B(1,1) = 1;     B(1,2) = 0;    B(1,3) = 0;
  B(2,0) = 0;    B(2,1) = 0;     B(2,2) = 0;    B(2,3) = dt;
  B(3,0) = 0;    B(3,1) = 0;     B(3,2) = 0;    B(3,3) = 1;

  x_a(0) = 0;
  x_a(1) = 0;
  x_a(2) = 0;
  x_a(3) = 0;

  H(0,0) = 1;    H(0,1) = 0;     H(0,2) = 0;    H(0,3) = 0;
  H(1,0) = 0;    H(1,1) = 1;     H(1,2) = 0;    H(1,3) = 0;
  H(2,0) = 0;    H(2,1) = 0;     H(2,2) = 1;    H(2,3) = 0;
  H(3,0) = 0;    H(3,1) = 0;     H(3,2) = 0;    H(3,3) = 1;

  x_0(0) = 0;
  x_0(1) = 0;
  x_0(2) = 0;
  x_0(3) = 0;

  Q(0,0) = 0.1;    Q(0,1) = 0;     Q(0,2) = 0;    Q(0,3) = 0;
  Q(1,0) = 0;    Q(1,1) = 0.1;     Q(1,2) = 0;    Q(1,3) = 0;
  Q(2,0) = 0;    Q(2,1) = 0;     Q(2,2) = 0.1;    Q(2,3) = 0;
  Q(3,0) = 0;    Q(3,1) = 0;     Q(3,2) = 0;    Q(3,3) = 0.1;

  R(0,0) = 1;   R(0,1) = 0;    R(0,2) = 0;    R(0,3) = 0;
  R(1,0) = 0;    R(1,1) = 1;   R(1,2) = 0;    R(1,3) = 0;
  R(2,0) = 0;    R(2,1) = 0;    R(2,2) = 1;   R(2,3) = 0;
  R(3,0) = 0;    R(3,1) = 0;    R(3,2) = 0;    R(3,3) = 1;

  double t = t_0;
  DVector sys_row(13); // t, X, z, k
  int a = 1;
  int phi = 2*M_PI*0.1;

  printf("Initialized\n");
  srand(time(NULL));

  while( t < t_1 ) {

    //Velocity command for motion on a circle with radius a
    //u(1) = a*cos(2*M_PI*t);
    //u(3) = -a*sin(2*M_PI*t);

    e = x_a - u;

    kf.predict(&u);

    //Position for motion on a circle with radius a
//     z(0) = a*sin(2*M_PI*t + phi);
//     z(2) = a*cos(2*M_PI*t + phi);
    z(0) = 0;
    z(2) = 1;

    // inject measurement noise
    double r1 = rand() / (double)(RAND_MAX);
    double r2 = rand() / (double)(RAND_MAX);
    z(0) += .8*z(0)*(r1-.2);
    z(2) += .8*z(2)*(r2-.2);

    kf.correct(&z);

    // write output
    sys_row(0) = t;
    sys_row(1) = x_a(0);
    sys_row(2) = x_a(1);
    sys_row(3) = x_a(2);
    sys_row(4) = x_a(3);
    sys_row(5) = z(0);
    sys_row(6) = z(1);
    sys_row(7) = z(2);
    sys_row(8) = z(3);
    sys_row(9) = kf.state()->elt(0);
    sys_row(10) = kf.state()->elt(1);
    sys_row(11) = kf.state()->elt(2);
    sys_row(12) = kf.state()->elt(3);
    sys_row.write_csv(f_sys);

    t += dt;
  }
  fflush(f_sys);
  fclose(f_sys);
  delete &I;

  return 0;
}
