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


/* There's some preprocessor trickery going on here to handle the four
 * blas/lapack types.  This file will be included four times with
 * various macros defined for appropriate types.
 */

class MATRIXTMP;
class MATRIX;
class MATRIXDYN;
class VECTORTMP;
class VECTORDYN;



/** Wrapper class for vectors.
 */
class VECTOR {
 public:
  /*================*/
  /*= CONSTRUCTORS =*/
  /*================*/
  virtual ~VECTOR();
  /// creates empty VECTOR
  VECTOR();
  /// creates a view of the underlying array
  VECTOR(TYPENAME *array, size_t len);

  /*================*/
  /*= STORAGE MGMT =*/
  /*================*/
  /** NULLs the array but doesn't free() */
  void clear();
  /** Returns total number of elements in array */
  size_t size() const;
  /** makes all elements zero*/
  void zero();

  /*==========*/
  /*= ACCESS =*/
  /*==========*/
  /** Element Access */
  TYPENAME &elt( size_t i ) const;

  /** Element Access */
  TYPENAME &operator()( size_t i ) const;

  /** Element Access */
  TYPENAME &operator[]( size_t i ) const;

  /// returns pointer to internal array
  TYPENAME *array() const;

  /** copies elements from src into this*/
  void copy( const VECTOR *src, size_t start, size_t nelt);
  /** copies elements from src into this*/
  void copy( size_t dst_start, 
             const VECTOR *src, size_t src_start, size_t nelt);

  /*=======*/
  /*= I/O =*/
  /*=======*/

  /** Writes this vector as a CSV row to fout
   */
  int write_csv( FILE *fout ) const;


  /*============*/
  /*= Controls =*/
  /*============*/

  /** Euler-approximate simulation of a linear system, accumulating result into this.
      \f[ {\bf this} \leftarrow {\bf this} + dt({\bf A}\,{\bf this} + {\bf B}\,{\bf u}) \f]
   */
  void lsim( const MATRIX *A, const MATRIX *B, const VECTOR *u, TYPENAME dt );


  /** Euler-approximate simulation of a linear system, accumulating result into this.
      Special case for zero input.
      \f[ {\bf this} \leftarrow {\bf this} + dt{\bf A}\,{\bf this} \f]
   */
  void lsim( const MATRIX *A, TYPENAME dt );


  /*========*/
  /*= BLAS =*/
  /*========*/
  /** direct access to blas gemv routine.
      \f[ {\bf this} \leftarrow \alpha\,\bf A\,\bf x + \beta\,{\bf this} \f]
      or
      \f[ {\bf this} \leftarrow \alpha\,\bf A^T\,\bf x + \beta\,{\bf this} \f]
 
      \param TransA Whether to transpose matrix \f$ \bf A \f$.  One of
      enum CblasNoTrans, CBlasTrans.

      \param M Rows in matrix \f$ \bf A\f$
    
      \param N Columns in matrix \f$ \bf A\f$
    
      \param alpha scalar factor for \f$\alpha\,op(\bf A)\,\bf x\f$
      \param beta scalar factor for \f${\bf this}\f$

      \param A matrix \f$ \bf A \f$
      \param X vector \f$ \bf x \f$
   */
  void gemv( const enum CBLAS_TRANSPOSE TransA, 
             const int M, const int N,
             const TYPENAME alpha, const TYPENAME *A, 
             const TYPENAME *X, const TYPENAME beta );

  /** Friendlier wrapper over gemv.
      \f[ {\bf this} \leftarrow \alpha\,\bf A\,\bf x + \beta\,{\bf this} \f]
      or
      \f[ {\bf this} \leftarrow \alpha\,\bf A^T\,\bf x + \beta\,{\bf this} \f]
   */
  void gemv( const enum CBLAS_TRANSPOSE TransA, const TYPENAME alpha, const MATRIX *A, 
             const VECTOR *X, const TYPENAME beta );
  /*=============*/
  /*= OPERATORS =*/
  /*=============*/

  /*--- ARITHMETIC ---*/


  
  /** Multiplies A and B elementwise, storing result in this.
   \f[ {\bf this} \leftarrow \bf A\,\bf B \f]
  */
  void mult(const VECTOR *A, const VECTOR *B );
  /** Adds A and B, storing result in this.
      \f[ {\bf this} \leftarrow \bf A+\bf B \f]
  */
  void add(const VECTOR *A, const VECTOR *B );



  /** Adds a*A and B, storing result in this.
      \f[ {\bf this} \leftarrow a \bf A+\bf B \f]
  */
  void add(const TYPENAME a, const VECTOR *A, 
           const VECTOR *B );

  /** Adds a*A and b*B, storing result in this.
      \f[ {\bf this} \leftarrow a \bf A+a\bf B \f]
  */
  void add(const TYPENAME a, const VECTOR *A, 
           const TYPENAME b, const VECTOR *B );

  /** Subtracts B from A, storing result in this.
      \f[ {\bf this} \leftarrow \bf A-\bf B \f]
  */
  void sub(const VECTOR *A, const VECTOR *B );

  /** Multiplies each element of this by -1.
      \f[ {\bf this} \leftarrow -{\bf this} \f]
  */
  void negate();

  
  /** Multiplies alpha by A by b, storing result in this.
      \f[{\bf this} \leftarrow \alpha\,{\bf A}\,{\bf b} \f]
   */
  void mult(const TYPENAME alpha, const MATRIX *A, const VECTOR *b );

  /** Multiplies A by b, storing result in this */
  void mult(  const MATRIX *A, const VECTOR *b );

  /** Multiplies A by b, storing result in this 
      \f[{\bf this} \leftarrow {\bf A}^T\,{\bf b} \f]
  */
  void mult_t(  const MATRIX *A, const VECTOR *b );

  /** Multiplies A by this, storing result in this */
  void mult(  const MATRIX *A );

  /** Scales this by a, storing result in this.
      \f[ {\bf this} \leftarrow a\,{\bf this} \f]
  */
  void scale(  TYPENAME a );

  /** Scales A by a, storing result in this.
      \f[ {\bf this} \leftarrow a\,\bf A \f]
  */
  void scale( const VECTOR *A, TYPENAME a );

  /** Adds A and B, storing result in this.
      \f[ {\bf this} \leftarrow \bf A+b \f]
  */
  void add(const VECTOR *A, TYPENAME b );

  /** Dot product of this and x
      \returns \f$ {\bf this}^T\,\bf x \f$
  */
  TYPENAME dot( const VECTOR *x );

  /** Cross Product
      \f$ this \leftarrow \bf A \times \bf B \f$
  */
  void cross( const VECTOR *A, const VECTOR *B);

  /*--- ASSIGNMENT ---*/
  /** Deep Copy */
  VECTOR &operator=(const VECTOR &other);


  /// add other into this
  VECTOR &operator+=( VECTORTMP &other );
  /// add other into this
  VECTOR &operator+=(const VECTOR &other );
  /// subtract other from this
  VECTOR &operator-=( VECTORTMP &other );
  /// subtract other from this
  VECTOR &operator-=(const VECTOR &other );
  /// multiply other into this
  VECTOR &operator*=( VECTORTMP &other );
  /// multiply other into this
  VECTOR &operator*=(const VECTOR &other );

  /// scale vector by a
  VECTOR &operator*=( TYPENAME a );
  /// add a elemennwise to matrix
  VECTOR &operator+=( TYPENAME a );
  /// subtract a elementwise from matrix
  VECTOR &operator-=( TYPENAME a );

 protected:
  /// pointer to vector data
  TYPENAME *my_array;
  /// number of elements of vector data
  size_t my_size;
};

/** Wrapper class for matrices.
 */
class MATRIX {
 public:

  /*================*/
  /*= CONSTRUCTORS =*/
  /*================*/
  virtual ~MATRIX();
  /** Constructs an empty MATRIX */
  MATRIX();
  /** Creates a matrix view array.

      Note that array will be interpreted as a FORTRAN matrix.  If it
      is a C matrix, you must transpose it.
   */
  MATRIX(TYPENAME *array, size_t fortran_rows, size_t fortran_cols);
  

  /*================*/
  /*= STORAGE MGMT =*/
  /*================*/
  /** NULLs the array but doesn't free() */
  void clear();
  /** returns row count*/
  size_t rows() const;
  /** returns column count*/
  size_t cols() const;
  /** Returns total number of elements in array */
  size_t size() const;
  /** Returns total number of elements in array */
  size_t order() const;
  /** makes all elements zero*/
  void zero();

  /*=======*/
  /*= I/O =*/
  /*=======*/

  /** Writes this matrix as a CSV to fout
   */
  int write_csv( FILE *fout ) const;

  /** Reads a csv file into this matrix
   */
  int read_csv( FILE *fin );

  /** Reads a tsv file into this matrix

      This function should work with the single variable ascii format
      used by matlab.
   */
  int read_tsv( FILE *fin );

  /*==========*/
  /*= ACCESS =*/
  /*==========*/
  /** Element Access */
  TYPENAME &elt( size_t row, size_t col ) const;

  /** Element Access */
  TYPENAME &operator()( size_t row, size_t col ) const;

  /// returns pointer to internal array
  TYPENAME *array() const;

  /*========*/
  /*= BLAS =*/
  /*========*/
  /** Direct access to blas *gemm function.
      \f[ 
      {\bf this} \leftarrow \alpha\,{\mathit op}(\bf A)\,{\mathit op}(\bf B) + \beta\,\bf C
      \f]
     
      Note that we don't expose the CBLAS_ORDER parameter because we
      are storing all of our matrices in Column-Major (Fortran) style.
  */
  void gemm( enum CBLAS_TRANSPOSE transA, enum CBLAS_TRANSPOSE transB, 
             TYPENAME alpha, const MATRIX *A, const MATRIX *B, TYPENAME beta );

  /*=============*/
  /*= OPERATORS =*/
  /*=============*/

  /*--- ASSIGNMENT ---*/
  /** Deep Copy */
  MATRIX &operator=(const MATRIX &other);
  
  /// add other into this
  MATRIX &operator+=( MATRIXTMP &other );
  /// add other into this
  MATRIX &operator+=(const MATRIX &other );
  /// subtract other from this
  MATRIX &operator-=( MATRIXTMP &other );
  /// subtract other from this
  MATRIX &operator-=(const MATRIX &other );
  /// multiply other into this
  MATRIX &operator*=( MATRIXTMP &other );
  /// multiply other into this
  MATRIX &operator*=(const MATRIX &other );

  /// scale matrix by a
  MATRIX &operator*=( TYPENAME a );
  /// add a elemennwise to matrix
  MATRIX &operator+=( TYPENAME a );
  /// subtract a elementwise from matrix
  MATRIX &operator-=( TYPENAME a );

  

  /** Inverts this in place 
      \param work Work array
      \param lwork length of work array
      \param ipiv temp array to store pivots
      \param lipiv size of ipiv, should be >= maximum dimension of this
      \return 0 on success, nonzero if non invertible
   */
  int invert( TYPENAME *work, int lwork, int *ipiv, int lipiv );
  /** returns optimal size of work array for inversion. */
  size_t invert_size();

  /*--- ARITHMETIC ---*/
  /** Multiplies A and B, storing result in this.
      \f[ {\bf this} \leftarrow \bf A\,\bf B \f]
  */
  void mult(const MATRIX *A, const MATRIX *B );
  /** Multiplies A and B transpose, storing result in this.
      \f[ {\bf this} \leftarrow \bf A\,\bf B^T \f]
  */
  void mult_1t(const MATRIX *A, const MATRIX *B );
  /** Multiplies A transpose and B, storing result in this.
      \f[ {\bf this} \leftarrow \bf A^T\,\bf B \f]
  */
  void mult_t1(const MATRIX *A, const MATRIX *B );
  /** Multiplies A transpose and B transpose, storing result in this.
      \f[ {\bf this} \leftarrow \bf A\,\bf B \f]
  */
  void mult_tt(const MATRIX *A, const MATRIX *B );
 
  /** Adds A and B, storing result in this.
      \f[ {\bf this} \leftarrow \bf A+\bf B \f]
  */
  void add(const MATRIX *A, const MATRIX *B );
  /** Adds a*A and B, storing result in this.
      \f[ {\bf this} \leftarrow a \bf A+\bf B \f]
  */
  void add(const TYPENAME a, const MATRIX *A, 
           const MATRIX *B );
  /** Adds a*A and b*B, storing result in this.
      \f[ {\bf this} \leftarrow a \bf A+a\bf B \f]
  */
  void add(const TYPENAME a, const MATRIX *A, 
           const TYPENAME b, const MATRIX *B );


  /** Subtracts B from A, storing result in this.
      \f[ {\bf this} \leftarrow \bf A-\bf B \f]
  */
  void sub(const MATRIX *A, const MATRIX *B );

  /** Multiplies each element of this by -1.
      \f[ {\bf this} \leftarrow -{\bf this} \f]
  */
  void negate();


  /** Scales this by a, storing result in this.
      \f[ {\bf this} \leftarrow a\,{\bf this} \f]
  */
  void scale( TYPENAME a );
  /** Scales A by a, storing result in this.
      \f[ {\bf this} \leftarrow a\,\bf A \f]
  */
  void scale(const MATRIX *A, TYPENAME a );
  /** Adds A and B, storing result in this.
      \f[ {\bf this} \leftarrow {\bf A} + b \f]
      \pre A->cols() == this->cols() && A->rows() == this->rows()
  */
  void add(const MATRIX *A, TYPENAME b );

  /** This gets the transpose of A.
      \f[ {\bf this} \leftarrow \bf A^T \f]
      \pre A->cols() == this->rows() && A->rows() == this->cols()
  */
  void transpose(const MATRIX *A);
 protected:
  /// The actual matrix data
  TYPENAME *my_array;
  /// number of rows in matrix
  size_t my_rows;
  /// number of columns in matrix
  size_t my_cols;
};


/** Heap allocated Vector
 */
class VECTORDYN : public VECTOR {
 public:
  /// creates deep copy of v
  VECTORDYN( const VECTOR &v);
  /// creates deep copy of v
  VECTORDYN( const VECTORDYN &v);
  /// creates copy by stealing v's data
  VECTORDYN(VECTORTMP &v);
  /// creates a deep copy of an array v, of size l
  VECTORDYN( const TYPENAME *v, const size_t l);
  /// creates new vector with size elements, zero initialized
  VECTORDYN( size_t size ) ;
  /// destructor
  ~VECTORDYN();

  /** Allocates new array */
  void alloc(size_t i);
  /** free()s the array */
  void del();
  /** free()s the array and NULLs */
  void release();
  /** Takes data array from m and free's m*/
  void steal(VECTORTMP *m);

  /** Steals other's array */
  VECTORDYN &operator=( VECTORTMP &other);

};

/** Used to store intermediate values in operations.
    
    You should probably not be making any instances of this class on
    your own.
 */
class VECTORTMP : public VECTORDYN {
 public:
  ~VECTORTMP();
  /// creates new vector with size elements
  VECTORTMP( size_t size );
  /// Deep copy of m
  VECTORTMP( const VECTOR &m );
  /** Returns a pointer to this objects array and clears the class
      field pointing to that array.

      \pre this.array is a usable, allocated array
      \post this.array is NULL
      \returns The value held by this.array before it was NULLed
   */
  TYPENAME *forfeit();


};

/** Heap allocated Matrix
 */
class MATRIXDYN : public MATRIX {
 public:
  /** Steals internal array of m */
  MATRIXDYN(MATRIXTMP &m);
  /** Steals creates properly sized MATRIX */
  MATRIXDYN(size_t rows, size_t cols);
  /** Deep copy */
  MATRIXDYN(const MATRIX &m );
  /** Deep copy */
  MATRIXDYN(const MATRIXDYN &m );
  /** Destructor */
  ~MATRIXDYN();

  /** Allocates new array */
  void alloc(size_t rows, size_t cols);
  /** free()s the array */
  void del();
  /** free()s the array and NULLs */
  void release();
  /** returns n * n identity matrix */
  static MATRIXDYN *identity(size_t n);

  /** Takes data array from m */
  void steal(MATRIXTMP *m);

  /** Steals other's array */
  MATRIXDYN &operator=( MATRIXTMP &other);
};

/** Used to store intermediate values in operations.
 */
class MATRIXTMP : public MATRIXDYN {
 public:
  /// Tmp matrix of certain size
  MATRIXTMP( size_t rows, size_t cols );
  /// Deep copy of m
  MATRIXTMP( const MATRIX &m );
  ~MATRIXTMP();
  /** Returns a pointer to this objects array and clears the class
      field pointing to that array.

      \pre this.array is a usable, allocated array
      \post this.array is NULL
      \returns The value held by this.array before it was NULLed
   */
  TYPENAME *forfeit();

};



/*=============*/
/*= OPERATORS =*/
/*=============*/

/// Elementwise add A and B
MATRIXTMP &operator+( MATRIXTMP &A, MATRIXTMP &B ); 
/// Elementwise add A and B
MATRIXTMP &operator+( const MATRIX &A, MATRIXTMP &B ); 
/// Elementwise add A and B
MATRIXTMP &operator+( MATRIXTMP &A, const MATRIX &B ); 
/// Elementwise add A and B
MATRIXTMP &operator+( const MATRIX &A, const MATRIX &B );


/// Elementwise subtract A by B
MATRIXTMP &operator-( MATRIXTMP &A, MATRIXTMP &B ); 
/// Elementwise subtract A by B
MATRIXTMP &operator-( const MATRIX &A, MATRIXTMP &B ); 
/// Elementwise subtract A by B
MATRIXTMP &operator-( MATRIXTMP &A, const MATRIX &B ); 
/// Elementwise subtract A by B
MATRIXTMP &operator-( const MATRIX &A, const MATRIX &B );


/// Multiply A and B
MATRIXTMP &operator*( MATRIXTMP &A, MATRIXTMP &B ); 
/// Multiply A and B
MATRIXTMP &operator*( const MATRIX &A, MATRIXTMP &B ); 
/// Multiply A and B
MATRIXTMP &operator*( MATRIXTMP &A, const MATRIX &B ); 
/// Multiply A and B
MATRIXTMP &operator*( const MATRIX &A, const MATRIX &B );

/// scale A by b
MATRIXTMP &operator*( MATRIXTMP &A, TYPENAME b ); 
/// scale B by a
MATRIXTMP &operator*( TYPENAME a, MATRIXTMP &B ); 
/// scale A by b
MATRIXTMP &operator*( const MATRIX &A, TYPENAME b ); 
/// scale B by a
MATRIXTMP &operator*( TYPENAME a, const MATRIX &B ); 

/// Add A and b
MATRIXTMP &operator+( MATRIXTMP &A, TYPENAME b ); 
/// Add a and B
MATRIXTMP &operator+( TYPENAME a, MATRIXTMP &B ); 
/// Add A and b
MATRIXTMP &operator+( const MATRIX &A, TYPENAME b ); 
/// Add a and B
MATRIXTMP &operator+( TYPENAME a, const MATRIX &B ); 




/// Elementwise add A and B
VECTORTMP &operator+( VECTORTMP &A, VECTORTMP &B ); 
/// Elementwise add A and B
VECTORTMP &operator+( const VECTOR &A, VECTORTMP &B ); 
/// Elementwise add A and B
VECTORTMP &operator+( VECTORTMP &A, const VECTOR &B ); 
/// Elementwise add A and B
VECTORTMP &operator+( const VECTOR &A, const VECTOR &B );


/// Elementwise subtract A by B
VECTORTMP &operator-( VECTORTMP &A, VECTORTMP &B ); 
/// Elementwise subtract A by B
VECTORTMP &operator-( const VECTOR &A, VECTORTMP &B ); 
/// Elementwise subtract A by B
VECTORTMP &operator-( VECTORTMP &A, const VECTOR &B ); 
/// Elementwise subtract A by B
VECTORTMP &operator-( const VECTOR &A, const VECTOR &B );


/// Multiply A and B
VECTORTMP &operator*( VECTORTMP &A, VECTORTMP &B ); 
/// Multiply A and B
VECTORTMP &operator*( const VECTOR &A, VECTORTMP &B ); 
/// Multiply A and B
VECTORTMP &operator*( VECTORTMP &A, const VECTOR &B ); 
/// Multiply A and B
VECTORTMP &operator*( const VECTOR &A, const VECTOR &B );

/// scale A by b
VECTORTMP &operator*( VECTORTMP &A, TYPENAME b ); 
/// scale B by a
VECTORTMP &operator*( TYPENAME a, VECTORTMP &B ); 
/// scale A by b
VECTORTMP &operator*( const VECTOR &A, TYPENAME b ); 
/// scale B by a
VECTORTMP &operator*( TYPENAME a, const VECTOR &B ); 

/// Add A and b
VECTORTMP &operator+( VECTORTMP &A, TYPENAME b ); 
/// Add a and B
VECTORTMP &operator+( TYPENAME a, VECTORTMP &B ); 
/// Add A and b
VECTORTMP &operator+( const VECTOR &A, TYPENAME b ); 
/// Add a and B
VECTORTMP &operator+( TYPENAME a, const VECTOR &B ); 

/// Multiply matrix A by vector x
VECTORTMP &operator*( const MATRIXTMP &A, const VECTORTMP &x );
/// Multiply matrix A by vector x
VECTORTMP &operator*( const MATRIX &A, const VECTORTMP &x );
/// Multiply matrix A by vector x
VECTORTMP &operator*( const MATRIXTMP &A, const VECTOR &x );
/// Multiply matrix A by vector x
VECTORTMP &operator*( const MATRIX &A, const VECTOR &x );
