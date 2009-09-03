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


#include "paste.h"

#if defined DOUBLE_TYPE

#define TYPENAME double
#define TYPE_PREFIX d
#define VECTOR DVectorView
#define VECTORDYN DVector
#define MATRIX DMatrixView
#define MATRIXDYN DMatrix
#define CBLAS_PREFIX cblas_d
#undef DOUBLE_TYPE

#elif defined FLOAT_TYPE

#define TYPENAME float
#define TYPE_PREFIX s
#define VECTOR SVectorView
#define VECTORDYN SVector
#define MATRIX SMatrixView
#define MATRIXDYN SMatrix
#define CBLAS_PREFIX cblas_s
#undef FLOAT_TYPE

/*
#elif defined COMPLEX_TYPE

#define TYPENAME complex double
#define VECTOR CVector
#define GSL_V_TYPE gsl_vector_complex
#undef COMPLEX_TYPE

#elif defined COMPLEX_FLOAT_TYPE

#define TYPENAME complex float
#define VECTOR SCVector
#define GSL_V_TYPE gsl_vector_complex_float
#undef COMPLEX_FLOAT_TYPE
*/

#else

#error "Data type note defined"

#endif

#define VECTORTMP PASTE(VECTORDYN,Tmp)
#define MATRIXTMP PASTE(MATRIXDYN,Tmp)

/*============*/
/*= BLAS OPS =*/
/*============*/

#define CBLAS_FUN(x) (PASTE(cblas_,PASTE(TYPE_PREFIX,x)))
#define LAPACK_FUN(x) (PASTE(TYPE_PREFIX,PASTE(x,_)))
//#define LAPACK_FUN(x) (PASTE(TYPE_PREFIX,x))
