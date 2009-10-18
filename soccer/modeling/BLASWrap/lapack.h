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


/* \file lapack.h
 * \brief C prototypes to various fortran lapack routines.
 *
 * Since there is no official c binding to lapack is there is with
 * blas, the only reasonable way to interface with lapack from C is to
 * call the fortran methods directly.
 *
 * Authors:
 *   Neil T. Dantam
 */

#ifndef LAPACK_H_
#define LAPACK_H_

#ifdef __cplusplus
extern "C" {
#endif

/** Inverse of matrix using LU factorization by *getrf.

    You must call *getrf before you call *getri.

    \param n Order of the matrix A
    \param A on entry the L and U factors from *getrf, 
      on exit the inverse of the original A
    \param lda number of rows in A
    \param ipiv pivot indices from sgetrf
    \param work workspace array
    \param lwork length of work, optimally > n*nb where nb is the
      optimal blocksize return by ilaenv_
    \param info output.  info==0 for success, info<zero for illegal
      argument, info > 0 for singular matrix
 */
void sgetri_( const int *n, float *A, const int *lda, 
              const int *ipiv, float *work, const int *lwork, int *info );
/** Inverse of matrix using LU factorization by dgetrf.
    \sa sgetri_
*/
void dgetri_( const int *n, double *A, const int *lda, 
              const int *ipiv, double *work, const int *lwork, int *info );


/** Compute an LU factorization.
    \param m number of rows of matrix A
    \param n number of columns of matrix A
    \param A matrix in column-major order, on exit the L and U factors
    \param lda leading dimesion of A, probably just rows in A
    \param array of length min(m,n), on exit the pivot indices
    \param info on success: info==0
 */
void sgetrf_( const int *m, const int *n, float *A, int *lda,
              int *ipiv, int *info );

/** Compute an LU factorization.
 */
void dgetrf_( const int *m, const int *n, double *A, int *lda,
              int *ipiv, int *info );

#ifdef __cplusplus
}
#endif
#endif
