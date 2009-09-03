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

#ifndef BLASWRAP_BASE_
#define BLASWRAP_BASE_

/*
 *  This file will be preprocessed then run through doxygen to
 *  generate the API docs.
 */

/** \file blaswrap_doc.h
 *  \author Neil T. Dantam
 */

/** \mainpage BLASWrap
 *
 *  BLASWrap proves a set of C++ classes that encapsulate the BLAS
 *  library.  It aims to provide a more natural interface to BLAS with
 *  minimal overhead.
 *
 *  \section Overview
 *
 *  This library provides the following classes which you may find useful:
 *
 *    - DVector: A linear algebra vector of type double (64-bit real floating point)
 *    - SVector: A linear algebra vector of type float (32-bit real floating point)
 *    - DMatrix: A linear algebra matrix of type double (64-bit real floating point)
 *    - SMatrix: A linear algebra matrix of type float (32-bit real floating point)
 *
 *  \section copy Copyright and License
 *
 *   This software is made available under the BSD license as follows:
 *
 *   Copyright (c) 2008, Georgia Tech Research Corporation,
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *     - Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     - Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     - Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION
 *   ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 *   NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 *   FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 *   SHALL GEORGIA TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT,
 *   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *   GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *   THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  \section mm Memory Management
 *
 *  The arrays for vectors and matrices are heap allocated and managed
 *  by the constructors and destructors of *Matrix and *Vector; this
 *  is fairly uninteresting.
 *
 *  The test program was given a quick run through valgrind and came
 *  out clean, so it looks like we probably don't have any memory
 *  leaks.
 *
 *  \subsection tmps Tmp Classes
 *
 *  Memory management relating to overloaded binary operators is
 *  slightly more complicated.  These operators must allocate some
 *  kind of intermediate to store their results.  The *MatrixTmp and
 *  *VectorTmp are used for this purpose.  These Tmp classes are
 *  special in that the arithmetic and assignment operators have been
 *  overloaded such that whey given an instance of a Tmp class, they
 *  will reuse the internal array field of the Tmp instance and then
 *  deallocate that Tmp instance.  This saves the need for a copy when
 *  using these binary operators.  This price you pay for the
 *  convenice of the binary operator is a malloc/free to manage the
 *  wrapper *Tmp instance.  If you are doing calculations in an inner
 *  loop, it would be better to preallocate any needed instances and
 *  use the add, mult, scale, etc. methods.
 *
 *  \subsection leak Potential Memory Leaks
 *
 *  Because the binary operators allocate a new object, you must be
 *  careful that this object is properly cleaned up.  The following operators will safely handle the result of these binary operators:
 *
 *    - further +,-,* operations
 *    - +=, -=, *= operations
 *    - assignment, =, operations
 *
 *  If you send the vector/matrix result of a binary operator
 *  elsewhere, you MUST clean it up yourself.  If this seems too
 *  awkard, then use a garbage collector; that's generally a good idea
 *  anway.
 *
 *  \section Performance
 *
 *  \subsection binop-perf Avoid Binary Operators
 *
 *  To maximize performance, avoid the overloaded binary operators in
 *  inner loops.  These must allocate a temporary object to store
 *  their return value.  Even though the array may get reused, you
 *  still pay to allocate and destroy the wrapper object.  Instead,
 *  pre-allocate a single instance and used methods of that instance
 *  which will store the results therein.
 *
 *  \subsection useblas Use A Single BLAS Methods
 *
 *  When you are able, call a single wrapped blas method to perform
 *  you operation.  Many BLAS methods actually perform several
 *  operations at once, ie scale, transpose, multiply.  It will be
 *  more efficient to do this in a single BLAS call than in multiple
 *  separate wrapper calls.
 *  
 *  \bug GCC does not vectorize loops in places it obviously should.
 *     This could be worked around by using GCC's vector extensions to
 *     C, but that limits our portability.
 *
 *  \bug Need to add complex support
 *
 *  \bug Many BLAS and LAPACK operations not yet supported.
 *
 *  \author Neil T. Dantam
 *
 */


/// performs column-major indexing as done in fortran
#define FORTRAN_INDEX( ar, rows, row, col )\
  ((ar) + ((rows)*(col)) + (row))


#define DOUBLE_TYPE
#include "def.h"
#include "blaswrap_source.h"
#include "undef.h"

#define FLOAT_TYPE
#include "def.h"
#include "blaswrap_source.h"
#include "undef.h"



#endif
