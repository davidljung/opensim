/* ________________________________________________________
  |                                                        |
  | Program Name:   matrix.h                               |
  |________________________________________________________|
  |                                                        |
  | description: Contains MATRIX data structure and func-  |
  |    tion declarations and descriptions for matrix.c     |
  |                                                        |
  |________________________________________________________| */

#include <stdlib.h>
#include <IKOR/general.h>
#define SVD_THRESHOLD     1.0e-6
#define PrintIfBiggerThan 0.0e+0
#define TINY              1.0e-20
#define SV_SMALL          1.0e-6

void ludcmp();        /*   LU decomposition                    */
void lubksb();        /*   LU backsubstitution                 */
void svdcmp(float **a, int m, int n, float *w, float **v);        /*   Singular Value Decomposition        */
void svbksb();        /*   SVD backsubstitution                */
void mat_free(MATRIX *m);      /*   Free matrix pointer                 */
void mat_pr(MATRIX *a);        /*   Print matrix (not full precision)   */
void mat_prf(MATRIX *a);       /*   Print matrix (full precision)       */
void mat_mul(MATRIX *a,MATRIX  *b,MATRIX *c);       /*   Multiplicate two matrices           */
void mat_add(MATRIX *a,MATRIX *b,MATRIX *c);       /*   Add two matrices                    */
void mat_sub(MATRIX *a,MATRIX *b,MATRIX *c);       /*   Subtract a matrix from another      */
void mat_sca(MATRIX *a,float  c);       /*   Scale a matrix by a factor          */
void mat_LU_inv(MATRIX *a);    /*   Invert a matrix using LU-dec.       */
void mat_pseudoinv(MATRIX *a); /*   Inverse or pseudoinverse using SVD  */
double mat_det(MATRIX *a);     /*   Determinant using SVD               */
double mat_eigen(MATRIX *a);   /*   Max eigen value of AtA              */
void mat_tra(MATRIX *a);       /*   Transpose a matrix                  */
void mat_cp(MATRIX *a,MATRIX *cpy);        /*   Copy a matrix into another          */

float mat_vec_dot();  /* Dot product two vectors: rows 0 of the two matrices  */
void  mat_vec_cross();/* Cross product two vectors: rows 0 of the two matrices*/
float mat_vec_abs();  /* Length (absolute value) of a vector: rows 0 a matrix */
void mat_null(MATRIX *a, int* n_rank, MATRIX* n, float* K2);

/* All these functions *mat_xxx2() are equivalent to their mat_xxx() function,
 * except that they automatically creates the return matrix by allocating
 * space for them, and returning the pointer to the MATRIX structure.
 */

MATRIX *mat_malloc(int rows, int cols); /*   Matrix allocation                     */
MATRIX *mat_mul2(MATRIX *a,MATRIX *b);      
MATRIX *mat_cp2();      
MATRIX *mat_add2(MATRIX *a,MATRIX *b);      
MATRIX *mat_sub2(MATRIX *a,MATRIX *b);      
MATRIX *mat_tra2(MATRIX *A);   /* B = mat_tra2(A); is:  B = transpose of A */
