/****************************************************************************
*
* File		: matrix.c
* Author	: Ole Henry Dorum
* Created	: January 28'th 1992
* Purpose       : Perform matrix operations.
* 
*****************************************************************************/

#include <UTILS/matrix.h>

/****************************************************************************
 **                                                                        **
 **   Num Rec in C:    LU-decomp        &   LU-backsubstitution            **
 **                    SV-decomposition &   SVD-backsubstitution           **
 **                                                                        **
 ****************************************************************************/
extern FILE* gcheck;     

static float at,bt,ct;

              /**************************************************/


     /***************************************************/
     /* Return the null space of a matrix using svd.	*/
     /***************************************************/
void mat_null(a, n_rank, n, K2)
MATRIX *a, *n;
int    *n_rank;
float  *K2;
{
  MATRIX *a_sqr,
         *v,
         *S_temp;
  float **U, *S, **V,
        *vector(),
        **conv_2_nric_ptr();
  float s_min, s_max, temp;
  void free_vector(),
       free_ivector();
  int i, j, R, C, *order;
  int *ivector();

  R = a->rows;
  C = a->cols;

  if ((R==0) || (C==0)) {
      fprintf(stderr, "Can't calculate null-space of a 0x0 matrix.\n");
      return;
  }

  S_temp = mat_malloc(C+1,1);
  v      = mat_malloc(C,C);
  V      = conv_2_nric_ptr(v);
  S      = vector(1,C);
  order  = ivector(1,C);
     

  if (R < C)
     {
     a_sqr = mat_malloc(C,C);
     for (i=0; i<R; i++)
        for (j=0; j<C; j++)
           a_sqr->p[i][j]=a->p[i][j];
     for (i=R; i<C; i++)
        for (j=0; j<C; j++)
           a_sqr->p[i][j]=0.0;
     }
  else
     {
     a_sqr = mat_malloc(R,C);
     mat_cp(a, a_sqr);
     }

  U = conv_2_nric_ptr(a_sqr);

  svdcmp(U, C, C, S, V);

  for (i=1; i<(C+1); i++)
     {
     S_temp->p[i][0] = S[i];
     order[i] = i;
     }
  for (i=C; i>1; i--)
     for (j=i-1; j>=1; j--)
        if (fabs(S_temp->p[i][0])>fabs(S_temp->p[j][0]))
           {
           temp = S_temp->p[i][0];
           S_temp->p[i][0] = S_temp->p[j][0];
           S_temp->p[j][0] = temp;
           temp = order[i];
           order[i] = order[j];
           order[j] = temp;
           }
  if (R<=C)
     s_min = fabs(S_temp->p[R][0]);
  else
     s_min = fabs(S_temp->p[C][0]);
  s_max = fabs(S_temp->p[1][0]);

  *n_rank = 0;
  while (fabs(S_temp->p[C-*n_rank][0]) < SV_SMALL)
     {
     for (j=1; j<=C; j++)
        n->p[j-1][*n_rank]
	  = V[j][order[C-*n_rank]];
     *n_rank = *n_rank + 1;
     }
 
  if ((fabs(s_min)>=SV_SMALL) && (fabs(s_min/s_max) < SV_SMALL))
     {
     for (j=1; j<=C; j++)
        n->p[j-1][*n_rank] = V[order[j]][C-*n_rank];
     *n_rank = *n_rank+1;
     }

  *K2 = s_max/s_min;

  mat_free(v);
  mat_free(a_sqr);
  mat_free(S_temp);
  free_vector(S,1,C);
  free_ivector(order,1,C);
  free(V);
  free(U);
}



void ludcmp(a,n,indx,d)
int n,*indx;
float **a,*d;
{
	int i,imax,j,k;
	float big,dum,sum,temp;
	float *vv,*vector();
	void nrerror(),free_vector();

	vv=vector(1,n);
	*d=1.0;
	for (i=1;i<=n;i++) {
		big=0.0;
		for (j=1;j<=n;j++)
			if ((temp=fabs(a[i][j])) > big) big=temp;
		if (big == 0.0) nrerror("Singular matrix in routine LUDCMP");
		vv[i]=1.0/big;
	}
	for (j=1;j<=n;j++) {
		for (i=1;i<j;i++) {
			sum=a[i][j];
			for (k=1;k<i;k++) sum -= a[i][k]*a[k][j];
			a[i][j]=sum;
		}
		big=0.0;
		for (i=j;i<=n;i++) {
			sum=a[i][j];
			for (k=1;k<j;k++)
				sum -= a[i][k]*a[k][j];
			a[i][j]=sum;
			if ( (dum=vv[i]*fabs(sum)) >= big) {
				big=dum;
				imax=i;
			}
		}
		if (j != imax) {
			for (k=1;k<=n;k++) {
				dum=a[imax][k];
				a[imax][k]=a[j][k];
				a[j][k]=dum;
			}
			*d = -(*d);
			vv[imax]=vv[j];
		}
		indx[j]=imax;
		if (a[j][j] == 0.0) a[j][j]=TINY;
		if (j != n) {
			dum=1.0/(a[j][j]);
			for (i=j+1;i<=n;i++) a[i][j] *= dum;
		}
	}
	free_vector(vv,1,n);
}


void lubksb(a,n,indx,b)
float **a,b[];
int n,*indx;
{
	int i,ii=0,ip,j;
	float sum;

	for (i=1;i<=n;i++) {
		ip=indx[i];
		sum=b[ip];
		b[ip]=b[i];
		if (ii)
			for (j=ii;j<=i-1;j++) sum -= a[i][j]*b[j];
		else if (sum) ii=i;
		b[i]=sum;
	}
	for (i=n;i>=1;i--) {
		sum=b[i];
		for (j=i+1;j<=n;j++) sum -= a[i][j]*b[j];
		b[i]=sum/a[i][i];
	}
}


void svbksb(u,w,v,m,n,b,x)
float **u,w[],**v,b[],x[];
int m,n;
{
	int jj,j,i;
	float s,*tmp,*vector();
	void free_vector();

	tmp=vector(1,n);
	for (j=1;j<=n;j++) {
		s=0.0;
		if (w[j]) {
			for (i=1;i<=m;i++) s += u[i][j]*b[i];
			s /= w[j];
		}
		tmp[j]=s;
	}
	for (j=1;j<=n;j++) {
		s=0.0;
		for (jj=1;jj<=n;jj++) s += v[j][jj]*tmp[jj];
		x[j]=s;
	}
	free_vector(tmp,1,n);
}


void svdcmp(a, m, n, w, v)
  float **a, *w, **v;
  int m, n;
{
  int flag, i, its, j, jj, k, l, nm;
  float c, f, h, s, x, y, z;
  float anorm = 0.0, g = 0.0, scale = 0.0;
  float *rv1, *vector();
  void nrerror(), free_vector();
  if (m < n)
    nrerror("SVDCMP: You must augment A with extra zero rows");
  rv1 = vector(1, n);
  for (i = 1; i <= n; i++)
  {
    l = i + 1;
    rv1[i] = scale * g;
    g = s = scale = 0.0;
    if (i <= m)
    {
      for (k = i; k <= m; k++)
	scale += fabs(a[k][i]);
      if (scale)
      {
	for (k = i; k <= m; k++)
	{
	  a[k][i] /= scale;
	  s += a[k][i] * a[k][i];
	}
	f = a[i][i];
	g = -SIGN(sqrt(s), f);
	h = f * g - s;
	a[i][i] = f - g;
	if (i != n)
	{
	  for (j = l; j <= n; j++)
	  {
	    for (s = 0.0, k = i; k <= m; k++)
	      s += a[k][i] * a[k][j];
	    f = s / h;
	    for (k = i; k <= m; k++)
	      a[k][j] += f * a[k][i];
	  }
	}
	for (k = i; k <= m; k++)
	  a[k][i] *= scale;
      }
    }
    w[i] = scale * g;
    g = s = scale = 0.0;
    if (i <= m && i != n)
    {
      for (k = l; k <= n; k++)
	scale += fabs(a[i][k]);
      if (scale)
      {
	for (k = l; k <= n; k++)
	{
	  a[i][k] /= scale;
	  s += a[i][k] * a[i][k];
	}
	f = a[i][l];
	g = -SIGN(sqrt(s), f);
	h = f * g - s;
	a[i][l] = f - g;
	for (k = l; k <= n; k++)
	  rv1[k] = a[i][k] / h;
	if (i != m)
	{
	  for (j = l; j <= m; j++)
	  {
	    for (s = 0.0, k = l; k <= n; k++)
	      s += a[j][k] * a[i][k];
	    for (k = l; k <= n; k++)
	      a[j][k] += s * rv1[k];
	  }
	}
	for (k = l; k <= n; k++)
	  a[i][k] *= scale;
      }
    }
    anorm = MAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
  }
  for (i = n; i >= 1; i--)
  {
    if (i < n)
    {
      if (g)
      {
	for (j = l; j <= n; j++)
	  v[j][i] = (a[i][j] / a[i][l]) / g;
	for (j = l; j <= n; j++)
	{
	  for (s = 0.0, k = l; k <= n; k++)
	    s += a[i][k] * v[k][j];
	  for (k = l; k <= n; k++)
	    v[k][j] += s * v[k][i];
	}
      }
      for (j = l; j <= n; j++)
	v[i][j] = v[j][i] = 0.0;
    }
    v[i][i] = 1.0;
    g = rv1[i];
    l = i;
  }
  for (i = n; i >= 1; i--)
  {
    l = i + 1;
    g = w[i];
    if (i < n)
      for (j = l; j <= n; j++)
	a[i][j] = 0.0;
    if (g)
    {
      g = 1.0 / g;
      if (i != n)
      {
	for (j = l; j <= n; j++)
	{
	  for (s = 0.0, k = l; k <= m; k++)
	    s += a[k][i] * a[k][j];
	  f = (s / a[i][i]) * g;
	  for (k = i; k <= m; k++)
	    a[k][j] += f * a[k][i];
	}
      }
      for (j = i; j <= m; j++)
	a[j][i] *= g;
    }
    else
    {
      for (j = i; j <= m; j++)
	a[j][i] = 0.0;
    }
    ++a[i][i];
  }
  for (k = n; k >= 1; k--)
  {
    for (its = 1; its <= 30; its++)
    {
      flag = 1;
      for (l = k; l >= 1; l--)
      {
	nm = l - 1;
	if (fabs(rv1[l]) + anorm == anorm)
	{
	  flag = 0;
	  break;
	}
	if (fabs(w[nm]) + anorm == anorm)
	  break;
      }
      if (flag)
      {
	c = 0.0;
	s = 1.0;
	for (i = l; i <= k; i++)
	{
	  f = s * rv1[i];
	  if (fabs(f) + anorm != anorm)
	  {
	    g = w[i];
	    h = PYTHAG(f, g);
	    w[i] = h;
	    h = 1.0 / h;
	    c = g * h;
	    s = (-f * h);
	    for (j = 1; j <= m; j++)
	    {
	      y = a[j][nm];
	      z = a[j][i];
	      a[j][nm] = y * c + z * s;
	      a[j][i] = z * c - y * s;
	    }
	  }
	}
      }
      z = w[k];
      if (l == k)
      {
	if (z < 0.0)
	{
	  w[k] = -z;
	  for (j = 1; j <= n; j++)
	    v[j][k] = (-v[j][k]);
	}
	break;
      }
      if (its == 30)
	nrerror("No convergence in 30 SVDCMP iterations");
      x = w[l];
      nm = k - 1;
      y = w[nm];
      g = rv1[nm];
      h = rv1[k];
      f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
      g = PYTHAG(f, 1.0);
      f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;
      c = s = 1.0;
      for (j = l; j <= nm; j++)
      {
	i = j + 1;
	g = rv1[i];
	y = w[i];
	h = s * g;
	g = c * g;
	z = PYTHAG(f, h);
	rv1[j] = z;
	c = f / z;
	s = h / z;
	f = x * c + g * s;
	g = g * c - x * s;
	h = y * s;
	y = y * c;
	for (jj = 1; jj <= n; jj++)
	{
	  x = v[jj][j];
	  z = v[jj][i];
	  v[jj][j] = x * c + z * s;
	  v[jj][i] = z * c - x * s;
	}
	z = PYTHAG(f, h);
	w[j] = z;
	if (z)
	{
	  z = 1.0 / z;
	  c = f * z;
	  s = h * z;
	}
	f = (c * g) + (s * y);
	x = (c * y) - (s * g);
	for (jj = 1; jj <= m; jj++)
	{
	  y = a[jj][j];
	  z = a[jj][i];
	  a[jj][j] = y * c + z * s;
	  a[jj][i] = z * c - y * s;
	}
      }
      rv1[l] = 0.0;
      rv1[k] = f;
      w[k] = x;
    }
  }
  free_vector(rv1, 1, n);
}


#undef SIGN
#undef PYTHAG
#undef TINY

/****************************************************************************/


MATRIX *mat_malloc(rows, cols)
int rows, cols;
{
  int    i;
  float  *mat, **row;
  MATRIX *matrix;

  /* Allocate space for structure, elements and pointers.
   *
   * Note, that the allocated number of row pointers is MAX(row, cols)
   * because it facilitates transposing rectangular matrices.
   */
 if ((rows > 0) && (cols > 0)) {
   mat = (float * ) malloc(rows * cols * sizeof(float));
   row = (float **) malloc((MAX(rows, cols)) * sizeof(float *));
 }
 else {
   mat = (float * ) malloc(sizeof(float));
   row = (float **) malloc(sizeof(float *));
   row[0]=mat;
 }
   matrix = (MATRIX *) malloc( sizeof( MATRIX ) );

  if(!mat || !row || !matrix)
    {
 /*     IKerrror(18, FATAL);*/
      fprintf(stderr, "Matrix allocation failed\n");
      return(NULL);
    }

  matrix->p = row;
  matrix->rows = rows;
  matrix->cols = cols;

  /* The Nth element of the array row points to the 1st element 
   * on the Nth row. Thus, **m = *m[0] = m[0][0]
   *
   * Calculate the addresses of the pointers pointing to the 
   * rows of the matrix
   */

  for(i = 0; i < rows; i++)
  {
    row[i] = mat; 
    mat += cols;
  }

  return(matrix);
}



void mat_free(m)
MATRIX *m;
{
  free( (char *) *m->p);
  free( (char *)  m->p);
  free( (char *)  m);
}


void mat_pr(a)
MATRIX *a;
{
  int i, j;

  printf("\n");
  for (i = 0; i < a->rows; i++)
  {
    for (j = 0; j < a->cols; j++)
      {
	if (fabs(a->p[i][j]) >= PrintIfBiggerThan)
	  printf("\t% .8f", a->p[i][j]);
	else
	  printf("\t  - ");
      }
    printf("\n");
  }
  printf("\n");
}


void mat_prf(a)
MATRIX *a;
{
  int i, j;

  printf("\n");
  for (i = 0; i < a->rows; i++)
  {
    for (j = 0; j < a->cols; j++)
    {
      printf("\t% f", a->p[i][j]);
      }
    printf("\n");
  }
  printf("\n");
}



/* The inverted matrix will still reside in a */
void mat_LU_inv(a)
MATRIX *a;
{
  MATRIX *y;
  float  **A, **Y, d, *col;
  int    N, i, j, *indx;
  float  **conv_2_nric_ptr();

  N = a->rows;

  /* Allocate space for y matrix to hold result, column vector col,
   * and indx array.
   */
  y    = mat_malloc(a->rows, a->cols);
  col  = vector(1, N);
  indx = (int *) malloc(N * sizeof(int));

  /* Create pointers to a and y conforming with Num-Rec-in-C format.
   * A and Y is from [1..N][1..N]
   */
  A = conv_2_nric_ptr(a);
  Y = conv_2_nric_ptr(y);

  /* LU decompose A
   */
  ludcmp(A, N, indx, &d);

  /* Find inverse by columns. (There is no better way to do it.)
   */
  for (j = 1; j <= N; j++)
    {
      for (i = 1; i <= N; i++)
	col[i] = 0.0;

      col[j] = 1.0;
      lubksb(A, N, indx, col);

      for (i = 1; i <= N; i++)
	Y[i][j] = col[i];
    }

  /* Inverted matrix is now in y. Copy y into a and free matrix y
   */
  mat_cp(y, a);

  mat_free(y);
  free_vector(col, 1, N);
  free( (char *) indx);
  free( (char *) A);
  free( (char *) Y);
}


/* The pseudoinverted matrix will still reside in a. In the case of a
 * square matrix, the result will actually be the inverted matrix. If
 * the matrix is rectangular, the pseudoinverse will have the correct 
 * dimension.
 */
void mat_pseudoinv(a)
MATRIX *a;
{
  MATRIX *q2, *z, *tmp;
  float  **A, **Q2, *Z;
  float  z_min, z_max;
  int    M, N, i, j;
  float  **conv_2_nric_ptr();

  M = a->rows; N = a->cols;

  /* Allocate space for q2 matrix, vector Z[1..N], z and tmp
   */
  q2 = mat_malloc(N, N);
  z  = mat_malloc(N, N);
  Z  = vector(1, N);

  /* Create pointers to a and q2 conforming with Num-Rec-in-C format.
   */
  A  = conv_2_nric_ptr(a);
  Q2 = conv_2_nric_ptr(q2);

  /* Compute A[1..M][1..N]`s singular value decomposition (SVD): A = Q1*Z*Q2_tra
   *
   * Q1  will replace A, and the diagonal value of singular values Z is output
   * as a vector Z[1..N]. The matrix Q2 (not the transpose Q2_tra) is output 
   * as Q2[1..N][1..N]. M must be greater than or equal to N; If it is smaller, 
   * then A should be filled up to square with zero rows.
   */
  svdcmp(A, M, N, Z, Q2);

  /* (Singular values = squareroot of the eigenvalues), find maximum.
   */
  z_max = 0.0;
  for (i = 1; i <= N; i++) if (Z[i] > z_max) z_max = Z[i];

  /* Set threshold value of the minimum singular value allowed 
   * to be nonzero.
   */
  z_min = z_max*SVD_THRESHOLD;

  /* Invert while copying from the Z vector into the z+ matrix, and weed out 
   * the too small singular values.
   */
  for (i = 0; i < N; i++)
    for (j = 0; j < N; j++)
      z->p[i][j] = ((i == j) && Z[i+1] > z_min) ? 1.0/Z[i+1] : 0.0; 
     /* z->p[i][j] = 1.0/Z[i+1];*/


  /*                               
   * A_pseudoinv = Q2 * Z_pseudoinv * Q1_tra
   *
   * Returned matrix A from svdcmp() is actually Q1, therefore:
   */

/* 
  printf("Testing the inverse:\n");
  q1t = mat_tra2(a);
  qq = mat_mul2(a,q1t);
  printf("Q1t*q1 = \n");
  mat_pr(qq);
  mat_free(qq);
  mat_free(q1t);
*/


  mat_tra(a);              /* Transpose Q1 to Q1_tra                          */
  tmp = mat_mul2(z, a);    /* tmp = Z_pseudoinv * Q1_tra                      */
  mat_mul(q2, tmp, a);     /* A_pseudoinv = Q2 * tmp = Q2 * Z_pseudo * Q1_tra */

/*
  q2t = mat_tra2(q2);
  qq2 = mat_mul2(q2,q2t);
  printf("Q2t*Q2 = \n");
  mat_pr(qq2);
  mat_free(qq2);
  mat_free(q2t);
*/



  mat_free(q2);
  mat_free(z);
  mat_free(tmp);
  free_vector(Z, 1, N);
  free( (char *) A);
  free( (char *) Q2);

}


/* The pseudoinverted matrix will still reside in a. In the case of a
 * square matrix, the result will actually be the inverted matrix. If
 * the matrix is rectangular, the pseudoinverse will have the correct 
 * dimension.
 */


double mat_det(a)
MATRIX *a;
{
  MATRIX *q2, *cp;
  float  **A, **Q2, *Z;
  int    M, N, i, j;
  float  **conv_2_nric_ptr();
  double det;

  M = a->rows; N = a->cols;

  /* Allocate space for q2 matrix, vector Z[1..N], z and tmp */
  cp = mat_cp2(a);
  q2 = mat_malloc(N, N);
  Z  = vector(1, N);

  /* Create pointers to a and q2 conforming with Num-Rec-in-C format.
   */
  A  = conv_2_nric_ptr(cp);
  Q2 = conv_2_nric_ptr(q2);

  /* Compute A[1..M][1..N]`s singular value decomposition (SVD): A = Q1*Z*Q2_tra
   *
   * Q1 will replace A, and the diagonal value of singular values Z is output
   * as a vector Z[1..N]. The matrix Q2 (not the transpose Q2_tra) is output 
   * as Q2[1..N][1..N]. M must be greater than or equal to N; If it is smaller, 
   * then A should be filled up to square with zero rows.
   */
  svdcmp(A, M, N, Z, Q2);

  /* (Singular values = squareroot of the eigenvalues of AtA), find maximum. */
    
  for(det=1.0,i=1;i<=N;i++)
       det *= (double) Z[i];

  mat_free(q2);
  mat_free(cp);
  free( (char *) A);
  free( (char *) Q2);
  free_vector(Z, 1, N);

  return(det);
}


double mat_eigen(a)
MATRIX *a;
{
  MATRIX *q2;
  float  **A, **Q2, *Z;
  float  z_min, z_max;
  int    M, N, i, j;
  float  **conv_2_nric_ptr();

  M = a->rows; N = a->cols;

  /* Allocate space for q2 matrix, vector Z[1..N]
   */
  q2 = mat_malloc(N, N);
  Z  = vector(1, N);

  /* Create pointers to a and q2 conforming with Num-Rec-in-C format.
   */
  A  = conv_2_nric_ptr(a);
  Q2 = conv_2_nric_ptr(q2);

  /* Compute A[1..M][1..N]`s singular value decomposition (SVD): A = Q1*Z*Q2_tra
   *
   * Q1  will replace A, and the diagonal value of singular values Z is output
   * as a vector Z[1..N]. The matrix Q2 (not the transpose Q2_tra) is output 
   * as Q2[1..N][1..N]. M must be greater than or equal to N; If it is smaller, 
   * then A should be filled up to square with zero rows.
   */
  svdcmp(A, M, N, Z, Q2);

  /* (Singular values = squareroot of the eigenvalues), find maximum.
   */
  z_max = 0.0;
  for (i = 1; i <= N; i++) if (Z[i] > z_max) z_max = Z[i];

  mat_free(q2);
  free_vector(Z, 1, N);
  free( (char *) A);
  free( (char *) Q2);

  return(z_max);
}




/* The 'c' matrix must already be declared float of size: arows x bcols.
 */
void mat_mul(a, b, c)
MATRIX *a, *b, *c;
{
  int i, j, k;

  for (k = 0; k < b->cols; k++)
    for (i = 0; i < a->rows; i++)
    {
      c->p[i][k] = 0;
      for (j = 0; j < a->cols; j++)
	c->p[i][k] += a->p[i][j]*b->p[j][k];
    }
}


/* The function will automatically create the result matrix of correct
 * dimension. The pointer to this matrix structure is returned.
 */
MATRIX *mat_mul2(a, b)
MATRIX *a, *b;
{
  MATRIX *c;

  c = mat_malloc(a->rows, b->cols);

  mat_mul(a, b, c);

  return(c);
}


/* The 'c' matrix must already be declared float of size: arows x bcols.
 *
 * c = a + b;
 */
void mat_add(a, b, c)
MATRIX *a, *b, *c;
{
  float *p, *q, *r;
  int   i;

  p = *a->p; q = *b->p; r = *c->p;

  for (i = a->rows*a->cols; i--;)
    *r++ = *p++ + *q++; 
}


/* The function will automatically create the result matrix of correct
 * dimension. The pointer to this matrix structure is returned.
 *
 * c = a + b;
 */
MATRIX *mat_add2(a, b)
MATRIX *a, *b;
{
  MATRIX *c;

  c = mat_malloc(a->rows, a->cols);

  mat_add(a, b, c);

  return(c);
}


/* The 'c' matrix must already be declared float of size: arows x acols.
 *
 * c = a - b;
 */
void mat_sub(a, b, c)
MATRIX *a, *b, *c;
{
  float *p, *q, *r;
  int   i;

  p = *a->p; q = *b->p; r = *c->p;

  for (i = a->rows*a->cols; i--;)
    *r++ = *p++ - *q++; 
}



/* The function will automatically create the result matrix of correct
 * dimension. The pointer to this matrix structure is returned.
 *
 * c = a - b;
 */
MATRIX *mat_sub2(a, b)
MATRIX *a, *b;
{
  MATRIX *c;

  c = mat_malloc(a->rows, a->cols);

  mat_sub(a, b, c);

  return(c);
}



/* The matrix 'a' is scaled by the factor 'c'
 */
void mat_sca(a, c)
MATRIX *a;
float  c;
{
  float *p;
  int   i;

  p = *a->p;

  for (i = a->rows*a->cols; i--;)
    *p++ *= c; 
}



/* The transposed matrix still resides in 'a' after transposition
 */
void mat_tra(a)
MATRIX *a;
{
  int   i, j, temp;
  float *p;

  if (a->rows == a->cols)              /* Square matrix */
  {
    for (i = a->rows-1; i--;)
      for (j = a->cols-1; j > i; j--)
	SWAP(a->p[i][j], a->p[j][i]);
  }
  else                                 /* Rectangular matrix */
  {
    temp = a->rows; 
    a->rows = a->cols; 
    a->cols = temp;

    /* Recompute pointers to the new rows of the matrix
     */
    p = *a->p;
    for(i = 0; i < a->rows; i++)
    {
      a->p[i] = p; 
      p += a->cols;
    }

    /* No need to swap elements if the matrix has only one row or column.
     */
    if (!((a->rows == 1) || (a->cols == 1)))
    {
      MATRIX *m;

      /*...otherwise put elements in temporary matrix and
       * copy from it into the columns of the transposed matrix.
       */
      m = mat_malloc(a->rows, a->cols);
      mat_cp(a,m);
      p = *m->p;

      for (j = 0; j < a->cols; j++)
	for (i = 0; i < a->rows; i++)
	  a->p[i][j] = *p++;

      mat_free(m);
    }    
  }
}


/* The function will automatically create the result matrix of correct
 * dimension which will be the transpose of A. The pointer to this matrix 
 * structure is returned.
 */
MATRIX *mat_tra2(A)
MATRIX *A;
{
  int   i, j;
  MATRIX *At;

  At = mat_malloc(A->cols, A->rows);

  for (i = A->rows-1; i--;)
    for (j = A->cols-1; j > i; j--)
      At->p[i][j] = A->p[j][i];

  return(At);
}


/* The copied matrix 'cpy' must already be declared float of 
 * same size as 'a'.
 */
void mat_cp(a, cpy)
MATRIX *a, *cpy;
{
  float *p, *q;
  int   i;

  p = *a->p; q = *cpy->p;

  for (i = a->rows*a->cols; i--;)
    *q++ = *p++; 
}



/* The function will automatically create the result matrix of correct
 * dimension . The pointer to this matrix structure is returned.
 */
MATRIX *mat_cp2(a)
MATRIX *a;
{
  int   i;
  float *p, *q;
  MATRIX *cpy;

  cpy = mat_malloc(a->rows, a->cols);

  mat_cp(a, cpy);

  return(cpy);
}



/* Returns an pointer which points to an array of pointers to matrix row 
 * elements in a way so that the matrix a->p[0..n][0..m] can be accessed
 * from [1..n-1][1..m-1] which is required by the Numerical Rec. i C.
 */
float **conv_2_nric_ptr(a)
MATRIX *a;
{
  float **new_ptr_2_row;
  int   i;

  /* Allocate space for an array of OFFSET pointers.
   */
  new_ptr_2_row = (float **) malloc((a->rows + 1) * sizeof(float *));

  /* The array of pointers to row must be offset by -1
   */
  for (i = 0; i < a->rows; i++)
    new_ptr_2_row[i+1] = a->p[i] - 1;

  /* To not have element[0][] dangling, let it point to a->p[0][0].
   */
  new_ptr_2_row[0] = a->p[0];

  return( new_ptr_2_row );
}



/* Free pointer pointing to the matrix in Numerical-Rec.-in-C-format
 */
void free_nric_ptr(p)
float **p;
{
  free( (char *)  p);
}



/* Returns vector dot product of the two vectors: row 0 of matrix a and b.
 */
float mat_vec_dot(a, ar, b, br)
MATRIX *a, *b;
int ar, br;
{
  int i;
  float d = 0;

  for (i = 0; i < a->cols; i++) 
    d += a->p[ar][i] * b->p[br][i];

  return( d );
}



/* Returns vector cross product of the two vectors: row ra, rb of matrix a and b.
 * Result vector matrix c must be of size [1][m] or [n][m].
 */
void mat_vec_cross(a, ar, b, br, c)
MATRIX *a, *b, *c;
int ar, br;
{
  c->p[0][0] = a->p[ar][1] * b->p[br][2] - a->p[ar][2] * b->p[br][1];
  c->p[0][1] = a->p[ar][2] * b->p[br][0] - a->p[ar][0] * b->p[br][2];
  c->p[0][2] = a->p[ar][0] * b->p[br][1] - a->p[ar][1] * b->p[br][0];
}



/* returns length (absolute value) of the vector: row r of matrix a.
 */
float mat_vec_abs(a, r)
MATRIX *a;
int r;
{
  int i;
  float d = 0;

  for (i = 0; i < a->cols; i++) 
    d += a->p[r][i] * a->p[r][i];

  return( sqrt(d) );
}
