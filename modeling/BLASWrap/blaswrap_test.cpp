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

//#include <stdlib.h>
#include <cstdlib>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <cblas.h>

#include "BLASWrap/blaswrap.h"

void dump_mat( DMatrix *M ) {
  size_t r, c;
  for( r = 0; r< M->rows(); r++ ) {
    for( c = 0; c< M->cols(); c++ )
      printf("%.2f ", (*M)(r,c));
    printf("\n");
  }
}


void dump_vec( DVector *V ) {
  size_t i;
  for(i = 0; i < V->size(); i++ ) {
    printf("%.2f ", V->elt(i) );
  }
  printf("\n");

}

int main( int argc, char **argv ) {
  {
    DMatrix A(2, 3);
    DMatrix B(2, 3);
    DVector X(3);
    DMatrix Ap(3, 2);

    {DVector X(3);}
    A(0,0) = 1;
    A(0,1) = 2;
    A(0,2) = 3;
    A(1,0) = 4;
    A(1,1) = 5;
    A(1,2) = 6;

    B(0,0) = 10;
    B(0,1) = 20;
    B(0,2) = 30;
    B(1,0) = 40;
    B(1,1) = 50;
    B(1,2) = 60;

    Ap.transpose(&A);

    printf("A\n");
    dump_mat(&A);
    printf("\nB\n");
    dump_mat(&B);
    printf("\nA'\n");
    dump_mat(&Ap);

    {
      DMatrix C = A + B;
      printf("\nA+B\n");
      dump_mat(&C);
    }
    {
      DMatrix C = B - A;
      printf("\nB-A\n");
      dump_mat(&C);
    }
    {
      DMatrix C = A*Ap;
      printf("\nA*A'\n");
      dump_mat(&C);
    }
    {
      DMatrix C = Ap*A;
      printf("\nA'*A\n");
      dump_mat(&C);
    }
    {
      DMatrix C = A;
      C+=5;
      printf("\nA + 5\n");
      dump_mat(&C);
    }{
      DMatrix C = A;
      C*=5;
      printf("\nA * 5\n");
      dump_mat(&C);
    }
  }

  {
    DVector A(5), B(5);
    int i = 5;
    while(i--) { A(i) = i+1; B(i) = (i+1)*10; }
    printf("\nA\n");
    dump_vec(&A);
    printf("\nB\n");
    dump_vec(&B);
    {
      DVector C = A;
      C *= 5;
      printf("\nA*5\n");
      dump_vec(&C);
    }
    {
      DVector C = A * 5;
      printf("\nA*5\n");
      dump_vec(&C);
    }
  }


  { // inversion
    printf("Inversion\n");
    printf("---------\n");
    DMatrix A(3, 3);
    A(0,0) = 1;
    A(0,1) = 2;
    A(0,2) = 2;
    A(1,0) = 1;
    A(1,1) = 0;
    A(1,2) = 2;
    A(2,0) = 0;
    A(2,1) = 1;
    A(2,2) = 1;
    A.write_csv(stdout);
    printf("inv work size: %d\n", A.invert_size() );
        int s = A.invert_size();
        double *w =(double*) malloc(s * sizeof(double));
    int ipiv[3];
    A.invert( w, s, ipiv, 3);
    printf("inverse:\n");
    free(w);
    dump_mat(&A);
  }
  //{DMatrix X(3,1);}

  { //read tsv
    DMatrix A(3,2);
    FILE *f = fopen("3x2.tsv","r");
    assert(f);
    int r = A.read_tsv(f);
    fclose(f);
    printf("Reading TSV\n");
    printf("-----------\n");
    A.write_csv(stdout);
    assert( 0 == r );
  }

  { // vector view
    double a[4];
    DVectorView v(a,4);
    v(0) = 1;
    v(1) = 2;
    v(2) = 3;
    v(3) = 4;
    assert(a[2] == 3 );
  }
}
