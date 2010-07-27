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


/*=============*/
/*= FUNCTIONS =*/
/*=============*/


/// scales each element of X by a
static void bw_array_scale( TYPENAME *X, TYPENAME a, size_t len ) {
  CBLAS_FUN(scal)( len, a, X, 1 );
}

/// scales each element of U by a storing result in X
static void bw_array_scale( TYPENAME *X, const TYPENAME *U, TYPENAME a, size_t len ) {
  for( size_t i = 0; i < len; i ++ )
    X[i] = U[i] * a;
}

/// adds a to each element of X
static void bw_array_add( TYPENAME *X, TYPENAME a, size_t len ) {
  for( size_t i = 0; i < len; i ++ )
    X[i] += a;
}

/// addes a to each element U by a storing result in X
static inline void bw_array_add( TYPENAME *X, const TYPENAME *U, TYPENAME a, size_t len ) {
  for( size_t i = 0; i < len; i ++ )
    X[i] = U[i] + a;
}

/// elementwise multiplies X and U storing result in X
static inline void bw_array_mult(TYPENAME *X, const TYPENAME *U, size_t len ) {
  for( size_t i = 0; i < len; i ++ )
    X[i] *= U[i];
}

/// elementwise multiplies U and V storing result in X
static void bw_array_mult(TYPENAME *X, const TYPENAME *U, const TYPENAME *V, size_t len ) {
  for( size_t i = 0; i < len; i ++ )
    X[i] = U[i] * V[i];
}

/// elementwise adds X and U storing result in X
static void bw_array_add(TYPENAME *X, const TYPENAME *U, size_t len ) {
  for( size_t i = 0; i < len; i ++ )
    X[i] += U[i];
}

/// elementwise adds U and V storing result in X
static void bw_array_add(TYPENAME *X, const TYPENAME *U, const TYPENAME *V, size_t len ) {
  for( size_t i = 0; i < len; i ++ )
    X[i] = U[i] + V[i];
}

/// elementwise adds u*U and V storing result in X
static void bw_array_add(TYPENAME *X, const TYPENAME u, const TYPENAME *U,
                         const TYPENAME *V, size_t len ) {
  for( size_t i = 0; i < len; i ++ )
    X[i] = u*U[i] + V[i];
}


/// elementwise adds u*U and V storing result in X
static void bw_array_add(TYPENAME *X, const TYPENAME u, const TYPENAME *U,
                         const TYPENAME v, const TYPENAME *V, size_t len ) {
  for( size_t i = 0; i < len; i ++ )
    X[i] = u*U[i] + v*V[i];
}


/// elementwise subtracts X by U storing result in X
static inline void bw_array_sub(TYPENAME *X, const TYPENAME *U, size_t len ) {
  for( size_t i = 0; i < len; i ++ )
    X[i] -= U[i];
}

/// elementwise subtracts U by V storing result in X
static void bw_array_sub(TYPENAME *X, const TYPENAME *U, const TYPENAME *V, size_t len ) {
  for( size_t i = 0; i < len; i ++ )
    X[i] = U[i] - V[i];
}

/// Multiplies each element of X by -1
static void bw_array_negate(TYPENAME *X, size_t len ) {
  for( size_t i = 0; i < len; i ++ )
    X[i] *= -1;
}

/// Makes each element of X zero
static void bw_array_zero(TYPENAME * X, size_t len ) {
  for( size_t i = 0; i < len; i ++ )
    X[i] = 0;
}

#undef size_t

/*================*/
/*= CONSTRUCTORS =*/
/*================*/
MATRIX::~MATRIX() { }
MATRIXDYN::~MATRIXDYN() { release(); }

MATRIX::MATRIX() { clear(); }

VECTOR::VECTOR() { clear(); }

MATRIX::MATRIX(TYPENAME *array, size_t fortran_rows, size_t fortran_cols):
  my_array(array),
  my_rows(fortran_rows),
  my_cols(fortran_cols)
{}

VECTOR::VECTOR(TYPENAME *array, size_t len):
  my_array(array),
  my_size(len){}

MATRIXDYN::MATRIXDYN(MATRIXTMP &m) {
  my_array = m.array();
  my_rows = m.rows();
  my_cols = m.cols();
  m.clear();
  BW_DELETE(&m);
}

MATRIXDYN::MATRIXDYN( const MATRIX &m ) {
  alloc(m.rows(), m.cols());
  memcpy( my_array, m.array(), sizeof(TYPENAME) * size() );
}

MATRIXDYN::MATRIXDYN( const MATRIXDYN &m ) {
  alloc(m.rows(), m.cols());
  memcpy( my_array, m.array(), sizeof(TYPENAME) * size() );
}

MATRIXDYN::MATRIXDYN(size_t rows, size_t cols) {
  alloc( rows, cols );
  zero();
}


MATRIXTMP::MATRIXTMP( size_t rows, size_t cols ) :
  MATRIXDYN(rows, cols){}

MATRIXTMP::MATRIXTMP( const MATRIX &m) :
  MATRIXDYN(m) {}

MATRIXTMP::~MATRIXTMP() { release(); }

VECTOR::~VECTOR() { }

VECTORDYN::~VECTORDYN() { 
  release(); 
}

VECTORDYN::VECTORDYN( size_t size ) {
  alloc(size);
  zero();
}
VECTORDYN::VECTORDYN( const VECTOR &v ) {
  alloc( v.size() );
  memcpy( my_array, v.array(), sizeof(TYPENAME) * v.size() );
}

VECTORDYN::VECTORDYN( const VECTORDYN &v ) {
  alloc( v.size() );
  memcpy( my_array, v.array(), sizeof(TYPENAME) * v.size() );
}

VECTORDYN::VECTORDYN( const TYPENAME *v, const size_t l) {
  alloc(l);
  memcpy(my_array, v, sizeof(TYPENAME)*l);
}


VECTORDYN::VECTORDYN( VECTORTMP &v ) {
  my_array = v.array();
  my_size = v.size();
  v.clear();
  BW_DELETE(&v);
}

VECTORTMP::VECTORTMP( size_t size) :
  VECTORDYN(size){}

VECTORTMP::VECTORTMP( const VECTOR &m ): VECTORDYN(m) {}

VECTORTMP::~VECTORTMP() { release(); }

MATRIXDYN *MATRIXDYN::identity(size_t n ) {
  MATRIXDYN *I = new MATRIXDYN(n,n);
  for( size_t i = 0; i < n; i ++ )
    I->elt(i,i) = 1;
  return I;
}

/*================*/
/*= STORAGE MGMT =*/
/*================*/
void MATRIX::clear() {
  my_array = NULL;
  my_rows = 0;
  my_cols = 0;
}

void MATRIXDYN::alloc(size_t rows, size_t cols) {
  my_array = (TYPENAME*) BW_ALLOC_PURE( sizeof(TYPENAME) * rows * cols );
  this->my_rows = rows;
  this->my_cols = cols;
}

void MATRIXDYN::del() {
  if( my_array )
    BW_FREE( my_array );
}
void MATRIXDYN::release() {
  del();
  clear();
}

void MATRIX::zero() {
  bw_array_zero( my_array, size() );
}


size_t MATRIX::rows() const { return my_rows; }
size_t MATRIX::cols() const { return my_cols; }
size_t MATRIX::size() const { return rows() * cols(); }
size_t MATRIX::order() const { return rows() * cols(); }

void VECTOR::clear() {
  my_array = NULL;
  my_size = 0;
}

void VECTORDYN::alloc(size_t size ){
  my_array = (TYPENAME*) BW_ALLOC_PURE( sizeof(TYPENAME) * size );
  my_size = size;
}

void VECTORDYN::del() {
  if( my_array )
    BW_FREE( my_array );
}
void VECTORDYN::release() {
  del();
  clear();
}

void VECTOR::zero() {
  bw_array_zero( my_array, size() );
}
size_t VECTOR::size() const { return my_size; }

/*==========*/
/*= ACCESS =*/
/*==========*/
TYPENAME &MATRIX::elt( size_t row, size_t col ) const {
  assert( row < rows() && col < cols() );
  return * FORTRAN_INDEX( array(), rows(), row, col );
}

TYPENAME &MATRIX::operator()( size_t row, size_t col ) const  {
  return elt(row, col);
}

TYPENAME &VECTOR::elt( size_t i ) const  {
  assert( i < size() );
  return array()[i];
}

TYPENAME &VECTOR::operator()( size_t i ) const {
  return elt(i);
}


TYPENAME &VECTOR::operator[]( size_t i ) const {
  return elt(i);
}

void VECTOR::copy(size_t dst_start, const VECTOR *src,
                  size_t src_start, size_t nelt ) {
  memcpy(array() + dst_start, src->array() + src_start, nelt );
}

void VECTOR::copy(const VECTOR *src,
                  size_t src_start, size_t nelt ) {
  this->copy(0, src, src_start, nelt );
}

TYPENAME *MATRIX::array() const {return my_array; }
TYPENAME *VECTOR::array() const {return my_array; }

/*=============*/
/*= OPERATORS =*/
/*=============*/

/*--- ARITHMETIC ---*/


void MATRIX::mult( const MATRIX *A, const MATRIX *B ) {
  //assert( rows() == A->rows()
          //&& cols() == B->cols()
          //&& A->cols() == B->rows() );
  gemm( CblasNoTrans, CblasNoTrans, 
        1, A, B, 0 );
}
void MATRIX::mult_1t( const MATRIX *A, const MATRIX *B ) {
  this->gemm( CblasNoTrans, CblasTrans, 1, A, B, 0 );
}
void MATRIX::mult_t1( const MATRIX *A, const MATRIX *B ) {
  this->gemm( CblasTrans, CblasNoTrans, 1, A, B, 0 );
}

void MATRIX::mult_tt( const MATRIX *A, const MATRIX *B ) {
  this->gemm( CblasTrans, CblasTrans, 1, A, B, 0 );
}

void MATRIX::add( const MATRIX *A, const MATRIX *B ) {
  assert( rows() == A->rows() && rows() == B->rows()
          && cols() == A->cols() && cols() == B->cols() );
  bw_array_add( array(), A->array(), B->array(), size() );
}

void MATRIX::add(const TYPENAME a, const MATRIX *A, const MATRIX *B ) {
  assert( rows() == A->rows() && rows() == B->rows()
          && cols() == A->cols() && cols() == B->cols() );
  bw_array_add( array(), a, A->array(), B->array(), size() );
}

void MATRIX::add(const TYPENAME a, const MATRIX *A, 
                 const TYPENAME b, const MATRIX *B ) {
  assert( rows() == A->rows() && rows() == B->rows()
          && cols() == A->cols() && cols() == B->cols() );
  bw_array_add( array(), a, A->array(), b, B->array(), size() );
}

void MATRIX::sub( const MATRIX *A, const MATRIX *B ) {
  assert( rows() == A->rows() && rows() == B->rows()
          && cols() == A->cols() && cols() == B->cols() );
  bw_array_sub( array(), A->array(), B->array(), size() );
}

void MATRIX::negate() {
  bw_array_negate( array(), size() );
}


void MATRIX::add( const MATRIX *A, TYPENAME b ) {
  assert( rows() == A->rows() && cols() == A->cols() );
  bw_array_add( array(), A->array(), size() );
}

void MATRIX::scale( const MATRIX *A, TYPENAME b ) {
  assert( rows() == A->rows() && cols() == A->cols() );
  bw_array_scale( array(), A->array(), b, size() );
}

void MATRIX::transpose( const MATRIX *A) { 
  assert( A->array() != array() );
  size_t i,j;
  assert( A->cols() == rows() && A->rows() == cols() );
  for( i = 0; i < rows(); i++ ) 
    for( j = 0; j < cols(); j++ )
      elt(i,j) = (*A)(j,i);
}

size_t MATRIX::invert_size() { 
  TYPENAME work[2];
  int order = this->order();
  int lda = order;//this->cols();
  int lwork = -1;
  int info;
  int ipiv[10] = {0};
  LAPACK_FUN(getri)(&order, this->my_array,  &lda,
                    ipiv, work, &lwork, &info);
  assert( 0 == info );
  return work[0];
}

int MATRIX::invert(TYPENAME *work, int lwork, int *ipiv, int lipiv ) { 
  int M = rows();
  int N = cols();
  int lda = M;
  int info;

  assert( (M > N) ? (lipiv >= M ) : (lipiv >= N) );

  LAPACK_FUN(getrf)( &M, &N, my_array, &lda, ipiv, &info  );
  assert( 0 <= info );
  if( info ) return info;
  
  N = cols();
  lda = rows();
  LAPACK_FUN(getri)(&N, my_array, &lda, ipiv, work, &lwork, &info );
  assert( 0 == info );
  return 0;
}

void VECTOR::mult( const VECTOR *A, const VECTOR *B ) {
  assert( size() == A->size()
          && size() == B->size() );
  bw_array_mult( array(), A->array(), B->array(), size() );
}

void VECTOR::mult( const MATRIX *A, const VECTOR *b ) {
  this->gemv( CblasNoTrans, 1, A, b, 0 );
}


void VECTOR::mult_t( const MATRIX *A, const VECTOR *b ) {
  this->gemv( CblasTrans, 1, A, b, 0 );
}

void VECTOR::mult(const TYPENAME alpha, const MATRIX *A, const VECTOR *b ) {
  this->gemv( CblasNoTrans, alpha, A, b, 0 );
}

void VECTOR::mult( const MATRIX *A ) {
  this->gemv( CblasNoTrans, 1, A, this, 0 );
}

void VECTOR::add( const VECTOR *A, const VECTOR *B ) {
  assert( size() == A->size()
          && size() == B->size() );
  bw_array_add( array(), A->array(), B->array(), size() );
}
void VECTOR::scale( const VECTOR *A, TYPENAME b ) {
  assert( size() == A->size() );
  bw_array_scale( array(), A->array(), b, size() );
}

void VECTOR::sub( const VECTOR *A, const VECTOR *B ) {
  assert( size() == A->size()
          && size() == B->size() );
  bw_array_sub( array(), A->array(), B->array(), size() );
}

void VECTOR::negate() {
  bw_array_negate( array(), size() );
}


void VECTOR::add( const VECTOR *A, TYPENAME b ) {
  assert( A->size() == size() );
  bw_array_add( array(), A->array(), size() );
}

void VECTOR::add(const TYPENAME a, const VECTOR *A, const VECTOR *B ) {
  assert( this->size() == A->size() && this->size() == B->size() );
  bw_array_add( array(), a, A->array(), B->array(), size() );
}

void VECTOR::add(const TYPENAME a, const VECTOR *A, 
                 const TYPENAME b, const VECTOR *B ) {
  assert( this->size() == A->size() && this->size() == B->size() );
  bw_array_add( array(), a, A->array(), b, B->array(), size() );
}

TYPENAME VECTOR::dot( const VECTOR *A ) {
  assert( size() == A->size() );
  return CBLAS_FUN(dot)(size(), array(), 1, A->array(), 1);
}

void VECTOR::cross( const VECTOR *A, const VECTOR *B ) {
  assert( A->size() == B->size() );
}

/*--- ASSIGNMENT ---*/
MATRIX &MATRIX::operator=( const MATRIX &other) {
  assert( rows() == other.rows() && cols() == other.cols() );
  memcpy( array(), other.array(), sizeof(TYPENAME) * rows() * cols() );
  return *this;
}

MATRIXDYN &MATRIXDYN::operator=( MATRIXTMP &other) {
  assert( rows() == other.rows() && cols() == other.cols() );
  steal(&other);
  return *this;
}

MATRIX &MATRIX::operator+=( MATRIXTMP &other ) {
  this->add( this, &other );
  BW_DELETE( &other );
  return *this;
}
MATRIX &MATRIX::operator+=( const MATRIX &other ) {
  this->add( this, &other );
  return *this;
}
MATRIX &MATRIX::operator-=( MATRIXTMP &other ) {
  this->sub( this, &other );
  BW_DELETE( &other );
  return *this;
}
MATRIX &MATRIX::operator-=( const MATRIX &other ) {
  this->sub( this, &other );
  return *this;
}
MATRIX &MATRIX::operator*=( MATRIXTMP &other ) {
  this->mult( this, &other );
  BW_DELETE( &other );
  return *this;
}
MATRIX &MATRIX::operator*=( const MATRIX &other ) {
  this->mult( this, &other );
  return *this;
}

MATRIX &MATRIX::operator*=( TYPENAME a ) {
  bw_array_scale( array(), a, size() );
  return *this;
}

MATRIX &MATRIX::operator+=( TYPENAME a ) {
  bw_array_add( array(), a, size() );
  return *this;
}
MATRIX &MATRIX::operator-=( TYPENAME a ) {
  return (*this)+= -a;
}


VECTOR &VECTOR::operator=( const VECTOR &other) {
  assert( size() == other.size() );
  memcpy( array(), other.array(), sizeof(TYPENAME) * size() );
  return *this;
}


VECTORDYN &VECTORDYN::operator=( VECTORTMP &other) {
  assert( size() == other.size() );
  steal( &other );
  return *this;
}

VECTOR &VECTOR::operator+=( VECTORTMP &other ) {
  this->add( this, &other );
  BW_DELETE( &other );
  return *this;
}
VECTOR &VECTOR::operator+=( const VECTOR &other ) {
  this->add( this, &other );
  return *this;
}
VECTOR &VECTOR::operator-=( VECTORTMP &other ) {
  this->sub( this, &other );
  BW_DELETE( &other );
  return *this;
}
VECTOR &VECTOR::operator-=( const VECTOR &other ) {
  this->sub( this, &other );
  return *this;
}
VECTOR &VECTOR::operator*=( VECTORTMP &other ) {
  this->mult( this, &other );
  BW_DELETE( &other );
  return *this;
}
VECTOR &VECTOR::operator*=( const VECTOR &other ) {
  this->mult( this, &other );
  return *this;
}

VECTOR &VECTOR::operator*=( TYPENAME a ) {
  bw_array_scale( array(), a, size() );
  return *this;
}

VECTOR &VECTOR::operator+=( TYPENAME a ) {
  bw_array_add( array(), a, size() );
  return *this;
}
VECTOR &VECTOR::operator-=( TYPENAME a ) {
  return (*this)+= -a;
}

/*========*/
/*= BLAS =*/
/*========*/
static int bw_gemm_check( enum CBLAS_TRANSPOSE transA, const MATRIX *A,
                          enum CBLAS_TRANSPOSE transB, const MATRIX *B,
                          const MATRIX *R) {
  size_t rowA, colA, rowB, colB;
  switch( transA ) {
  case CblasTrans:
    rowA = A->cols();
    colA = A->rows();
    break;
  case CblasNoTrans:
    rowA = A->rows();
    colA = A->cols();
    break;
  default:
    assert(0);
  }
  switch( transB ) {
  case CblasTrans:
    rowB = B->cols();
    colB = B->rows();
    break;
  case CblasNoTrans:
    rowB = B->rows();
    colB = B->cols();
    break;
  default:
    assert(0);
  }
  assert( R->rows() == rowA );
  assert( R->cols() == colB );
  assert( colA == rowB );
  return 1;
}
void MATRIX::gemm( enum CBLAS_TRANSPOSE transA, enum CBLAS_TRANSPOSE transB, 
                   TYPENAME alpha, const MATRIX *A, const MATRIX *B, TYPENAME beta ) {
  assert(bw_gemm_check( transA, A, transB, B, this ) );
  CBLAS_FUN(gemm)(CblasColMajor, transA, transB, 
                  rows(), cols(), A->cols(),
                  alpha, A->array(), A->rows(),
                  B->array(), B->rows(), 
                  beta, array(), this->rows() );
}

void VECTOR::gemv( const enum CBLAS_TRANSPOSE transA, 
                   const int M, const int N,
                   const TYPENAME alpha, const TYPENAME *A, 
                   const TYPENAME *X, const TYPENAME beta ) {

  assert( size() == (unsigned int)M );
  CBLAS_FUN(gemv)( CblasColMajor, transA, 
                   M, N, alpha, A, M, 
                   X, 1, beta, array(), 1);
}

void VECTOR::gemv( const enum CBLAS_TRANSPOSE TransA, const TYPENAME alpha, const MATRIX *A, 
                   const VECTOR *X, const TYPENAME beta )  {
  assert( size() == A->rows() && X->size() == A->cols() );
  gemv( TransA, A->rows(), A->cols(),
        alpha, A->array(), X->array(), beta );
}


/*=======*/
/*= I/O =*/
/*=======*/

int VECTOR::write_csv( FILE *fout ) const {
  for( size_t i = 0; i < size(); i ++ ) {
    fprintf(fout, "%f%s", elt(i), ((i+1 < size())?",":"\n") );
  }
  return 0;
}


int MATRIX::write_csv( FILE *fout ) const {
  for( size_t i = 0; i < rows(); i ++ ) 
    for( size_t j = 0; j < cols(); j ++ ) 
      fprintf(fout, "%f%s", elt(i,j), ((j+1 < cols())?",":"\n") );
  return 0;
}

int MATRIX::read_csv( FILE *fout ) {
  int r;
  int i = 0, j = 0;
  while( !feof(fout) ) {
    float f;
    r = fscanf(fout, "%f", &f);
    printf("read[%d][%d]: %f\n",i,j, f);
    if( EOF == r )
      break;
    elt(i,j) = f;
  EAT:
    int c = fgetc( fout );
    switch( c ) {
    case ' ':
    case '\t':
      goto EAT;
    case ',':
      j++;
      break;
    case '\n':
      i++; j = 0;
      break;
    case EOF:
      break;
    default:
      return 1;
    }
  }
  return 0;
}


int MATRIX::read_tsv( FILE *fin ) {
  int r;
  int c;
  size_t i = 0, j = 0;
  float f;
  while( !feof(fin) ) {
    while(! (isdigit(c = fgetc( fin )) || '+' == c || '-' == c ) ){
      switch( c ) {
      case ' ':
      case '\t':
        break;
      case '\n':
        if( j != cols() ) return j + 1;
        i++; j = 0;
        break;
      case EOF:
        return 0;
      default:
        return 1;
      }
    }
    ungetc( c, fin );
    r = fscanf(fin, "%f", &f);
    //printf("read[%d][%d]: %f\n",i,j, f);
    if( EOF == r )
      break;
    if( i >= rows() || j >= cols() ) return -1;
    elt(i,j) = f;
    j++;
  }
  if( i != rows() ) return( i + 1 );
  return 0;
}



/*============*/
/*= Controls =*/
/*============*/

void VECTOR::lsim( const MATRIX *A, const MATRIX *B, 
                   const VECTOR *u, TYPENAME dt ) {
  lsim(A,dt);
  this->gemv( CblasNoTrans, dt, B, u, 1. );
}


void VECTOR::lsim( const MATRIX *A, TYPENAME dt ) {
  this->gemv( CblasNoTrans, dt, A, this, 1. );
}



/*============*/
/*= TMP FUNS =*/
/*============*/

TYPENAME *MATRIXTMP::forfeit() {
  TYPENAME *r = array();
  clear();
  return r;
}


TYPENAME *VECTORTMP::forfeit() {
  TYPENAME *r = array();
  clear();
  return r;
}

void VECTORDYN::steal( VECTORTMP *m ) {
  assert( size() == m->size() );
  
  TYPENAME *a = m->forfeit();
  BW_DELETE(m);
  del();
  my_array = a;
}

void MATRIXDYN::steal( MATRIXTMP *m ) {
  assert( rows() == m->rows() && cols() == m->cols() );
  TYPENAME *a = m->forfeit();
  BW_DELETE(m);
  del();
  my_array = a;
}
/*=============*/
/*= OPERATORS =*/
/*=============*/

MATRIXTMP &operator+( MATRIXTMP &A, MATRIXTMP &B ) {
  A += B;
  BW_DELETE( &B );
  return A;
}

MATRIXTMP &operator+( MATRIXTMP &A, const MATRIX &B ) {
  A += B;
  return A;
}

MATRIXTMP &operator+( const MATRIX &A, MATRIXTMP &B ) {
  return B + A;
}

MATRIXTMP &operator+( const MATRIX &A, const MATRIX &B ) {
  MATRIXTMP *t = new MATRIXTMP( A.rows(), A.cols() );
  t->add( &A, &B );
  return *t;
}

MATRIXTMP &operator-( MATRIXTMP &A, MATRIXTMP &B ) {
  A -= B;
  BW_DELETE( &B );
  return A;
}

MATRIXTMP &operator-( MATRIXTMP &A, const MATRIX &B ) {
  A -= B;
  return A;
}

MATRIXTMP &operator-( const MATRIX &A, MATRIXTMP &B ) {
  B.sub( &A, &B);
  return B;
}

MATRIXTMP &operator-( const MATRIX &A, const MATRIX &B ) {
  MATRIXTMP *t = new MATRIXTMP( A.rows(), A.cols() );
  t->sub( &A, &B );
  return *t;
}

MATRIXTMP &operator*( MATRIXTMP &A, MATRIXTMP &B ) {
  MATRIXTMP *t = new MATRIXTMP( A.rows(), B.cols() );
  t->mult(&A,&B);
  BW_DELETE( &B );
  BW_DELETE( &A );
  return *t;
}

MATRIXTMP &operator*( MATRIXTMP &A, const MATRIX &B ) {
  MATRIXTMP *t = new MATRIXTMP( A.rows(), B.cols() );
  t->mult(&A,&B);
  BW_DELETE( &A );
  return *t;
}

MATRIXTMP &operator*( const MATRIX &A, MATRIXTMP &B ) {
  MATRIXTMP *t = new MATRIXTMP( A.rows(), B.cols() );
  t->mult(&A,&B);
  BW_DELETE( &B );
  return *t;
}

MATRIXTMP &operator*( const MATRIX &A, const MATRIX &B ) {
  MATRIXTMP *t = new MATRIXTMP( A.rows(), B.cols() );
  t->mult(&A,&B);
  return *t;
}



MATRIXTMP &operator*( MATRIXTMP &A, TYPENAME b ) {
  return (MATRIXTMP&) (A*=b);
}
MATRIXTMP &operator*( TYPENAME a, MATRIXTMP &B ){
  return (MATRIXTMP&) (B*=a);
} 
MATRIXTMP &operator*( const MATRIX &A, TYPENAME b ){
  MATRIXTMP *t = new MATRIXTMP( A );
  (*t) *= b;
  return *t;
} 
MATRIXTMP &operator*( TYPENAME a, const MATRIX &B ){
  return B * a;
} 
MATRIXTMP &operator+( MATRIXTMP &A, TYPENAME b ){
  return (MATRIXTMP&)(A += b);
} 
MATRIXTMP &operator+( TYPENAME a, MATRIXTMP &B ){
  return (MATRIXTMP&)(B += a);
} 
MATRIXTMP &operator+( const MATRIX &A, TYPENAME b ){
  MATRIXTMP *t = new MATRIXTMP( A.rows(), A.cols() );
  t->add(&A,b);
  return *t;
} 
MATRIXTMP &operator+( TYPENAME a, const MATRIX &B ){
  return B + a;
} 





VECTORTMP &operator+( VECTORTMP &A, VECTORTMP &B ) {
  A += B;
  BW_DELETE( &B );
  return A;
}

VECTORTMP &operator+( VECTORTMP &A, const VECTOR &B ) {
  A += B;
  return A;
}

VECTORTMP &operator+( const VECTOR &A, VECTORTMP &B ) {
  return B + A;
}

VECTORTMP &operator+( const VECTOR &A, const VECTOR &B ) {
  VECTORTMP *t = new VECTORTMP( A.size() );
  t->add( &A, &B );
  return *t;
}

VECTORTMP &operator-( VECTORTMP &A, VECTORTMP &B ) {
  A -= B;
  BW_DELETE( &B );
  return A;
}

VECTORTMP &operator-( VECTORTMP &A, const VECTOR &B ) {
  A -= B;
  return A;
}

VECTORTMP &operator-( const VECTOR &A, VECTORTMP &B ) {
  B.sub( &A, &B);
  return B;
}

VECTORTMP &operator-( const VECTOR &A, const VECTOR &B ) {
  VECTORTMP *t = new VECTORTMP( A.size() );
  t->sub( &A, &B );
  return *t;
}

VECTORTMP &operator*( VECTORTMP &A, VECTORTMP &B ) {
  VECTORTMP *t = new VECTORTMP( A.size() );
  t->mult(&A,&B);
  BW_DELETE( &B );
  BW_DELETE( &A );
  return *t;
}

VECTORTMP &operator*( VECTORTMP &A, const VECTOR &B ) {
  VECTORTMP *t = new VECTORTMP( A.size() );
  t->mult(&A,&B);
  BW_DELETE( &A );
  return *t;
}

VECTORTMP &operator*( const VECTOR &A, VECTORTMP &B ) {
  VECTORTMP *t = new VECTORTMP( A.size() );
  t->mult(&A,&B);
  BW_DELETE( &B );
  return *t;
}

VECTORTMP &operator*( const VECTOR &A, const VECTOR &B ) {
  VECTORTMP *t = new VECTORTMP( A.size() );
  t->mult(&A,&B);
  return *t;
}



VECTORTMP &operator*( VECTORTMP &A, TYPENAME b ) {
  return (VECTORTMP&) (A*=b);
}

VECTORTMP &operator*( TYPENAME a, VECTORTMP &B ){
  return (VECTORTMP&) (B*=a);
} 

VECTORTMP &operator*( const VECTOR &A, TYPENAME b ){
  VECTORTMP *t = new VECTORTMP( A );
  (*t) *= b;
  return *t;
} 

VECTORTMP &operator*( TYPENAME a, const VECTOR &B ){
  return B * a;
} 

VECTORTMP &operator+( VECTORTMP &A, TYPENAME b ){
  return (VECTORTMP&)(A += b);
} 

VECTORTMP &operator+( TYPENAME a, VECTORTMP &B ){
  return (VECTORTMP&)(B += a);
} 

VECTORTMP &operator+( const VECTOR &A, TYPENAME b ){
  VECTORTMP *t = new VECTORTMP( A.size() );
  t->add(&A,b);
  return *t;
} 

VECTORTMP &operator+( TYPENAME a, const VECTOR &B ){
  return B + a;
} 


VECTORTMP &operator*( const MATRIXTMP &A, const VECTORTMP &x ) {
  VECTORTMP *t = & ( (MATRIX)A * (VECTOR)x);
  BW_DELETE( &A);
  BW_DELETE( &x);
  return *t;
}
VECTORTMP &operator*( const MATRIX &A, const VECTORTMP &x ){ 
  VECTORTMP *t = & ( A * (VECTOR)x);
  BW_DELETE( &x);
  return *t;
} 

VECTORTMP &operator*( const MATRIXTMP &A, const VECTOR &x ) {
  VECTORTMP *t = & ( (MATRIX)A *x);
  BW_DELETE( &A);
  return *t;
}
VECTORTMP &operator*( const MATRIX &A, const VECTOR &x ) {
  VECTORTMP *t = new VECTORTMP( A.rows() );
  t->gemv( CblasNoTrans, 1, &A, &x, 1);
  return *t;
}
