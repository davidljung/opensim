/****************************************************************************
  Copyright (C)2002 David Jung <opensim@pobox.com>

  This program/file is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.
  
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details. (http://www.gnu.org)
  
  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
  
  $Id: SVD.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.16 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/


/*
 *
 * Singular Value Decomposition of a Matrix
 *
 * Currently uses the code from Numerical Recipies in C
 */


 
#include <base/SVD>

#include <base/Math>

using base::SVD;

using base::Math;
using base::Matrix;
using base::Vector;



const Real SVD::maxCondition = 1e100;
const Real SVD::minSingValue = 1e-6;

inline float FMIN(float a, float b) { return a<b?a:b; }
inline float FMAX(float a, float b) { return a>b?a:b; }
inline int IMIN(int a, int b) { return a<b?a:b; }
inline int IMAX(int a, int b) { return a>b?a:b; }

inline float SQR(float a) { return (a==0.0)?0.0:a*a; }


inline void nrerror(char error_text[])
{ throw std::runtime_error(Exception(String(error_text))); }


extern "C" {

#include <stdio.h>  
#include <stdlib.h>
#include <math.h>
  
  
#define NR_END 1
#define FREE_ARG char*
  

#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))



float *vector(long nl, long nh)
/* allocate a float vector with subscript range v[nl..nh] */
{
	float *v;

	v=(float *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(float)));
	if (!v) nrerror("allocation failure in vector()");
	return v-nl+NR_END;
}

void free_matrix(float **m, long nrl, long nrh, long ncl, long nch)
/* free a float matrix allocated by matrix() */
{
	free((FREE_ARG) (m[nrl]+ncl-NR_END));
	free((FREE_ARG) (m+nrl-NR_END));
}



void free_vector(float *v, long nl, long nh)
/* free a float vector allocated with vector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}



float **matrix(long nrl, long nrh, long ncl, long nch)
/* allocate a float matrix with subscript range m[nrl..nrh][ncl..nch] */
{
	long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
	float **m;

	/* allocate pointers to rows */
	m=(float **) malloc((size_t)((nrow+NR_END)*sizeof(float*)));
	if (!m) nrerror("allocation failure 1 in matrix()");
	m += NR_END;
	m -= nrl;

	/* allocate rows and set pointers to them */
	m[nrl]=(float *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(float)));
	if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
	m[nrl] += NR_END;
	m[nrl] -= ncl;

	for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

	/* return pointer to array of pointers to rows */
	return m;
}


float pythag(float a, float b)
{
	float absa, absb;
	absa=fabs(a);
	absb=fabs(b);
	if(absa>absb)
		return(absa*sqrt(1.0+SQR(absb/absa)));
	else
		return(absb==0.0 ? 0.0 : absb*sqrt(1.0+SQR(absa/absb)));
}



// the actual SVD routine

void svdcmp(float **a, int m, int n, float w[], float **v)
{
	float pythag(float a, float b);
	int flag,i,its,j,jj,k,l,nm, front, back;
	float anorm,c,f,g,h,s,scale,x,y,z,*rv1, *rv2, tol;
	float **ut, **vt;

	vt=matrix(1,n,1,n);
	ut=matrix(1,m,1,n);

	tol=(float)(1e-6);

	rv1=vector(1,n);
	rv2=vector(1,n);
	g=scale=anorm=0.0;
	for (i=1;i<=n;i++) {
		l=i+1;
		rv1[i]=scale*g;
		g=s=scale=0.0;
		if (i <= m) {
			for (k=i;k<=m;k++) 
				scale += fabs(a[k][i]);
			if (scale) {
				for (k=i;k<=m;k++) {
					a[k][i] /= scale;
					s += a[k][i]*a[k][i];
				}
				f=a[i][i];
				g = -SIGN(sqrt(s),f);
				h=f*g-s;
				a[i][i]=f-g;
				for (j=l;j<=n;j++) {
					for (s=0.0,k=i;k<=m;k++) s += a[k][i]*a[k][j];
					f=s/h;
					for (k=i;k<=m;k++) a[k][j] += f*a[k][i];
				}
				for (k=i;k<=m;k++) a[k][i] *= scale;
			}
		}
		w[i]=scale *g;
		g=s=scale=0.0;
		if (i <= m && i != n) {
			for (k=l;k<=n;k++) scale += fabs(a[i][k]);
			if (scale) {
				for (k=l;k<=n;k++) {
					a[i][k] /= scale;
					s += a[i][k]*a[i][k];
				}
				f=a[i][l];
				g = -SIGN(sqrt(s),f);
				h=f*g-s;
				a[i][l]=f-g;
				for (k=l;k<=n;k++) rv1[k]=a[i][k]/h;
				for (j=l;j<=m;j++) {
					for (s=0.0,k=l;k<=n;k++) s += a[j][k]*a[i][k];
					for (k=l;k<=n;k++) a[j][k] += s*rv1[k];
				}
				for (k=l;k<=n;k++) a[i][k] *= scale;
			}
		}
		anorm=FMAX(anorm,(fabs(w[i])+fabs(rv1[i])));
	}
	for (i=n;i>=1;i--) {
		if (i < n) {
			if (g) {
				for (j=l;j<=n;j++)
					v[j][i]=(a[i][j]/a[i][l])/g;
				for (j=l;j<=n;j++) {
					for (s=0.0,k=l;k<=n;k++) s += a[i][k]*v[k][j];
					for (k=l;k<=n;k++) v[k][j] += s*v[k][i];
				}
			}
			for (j=l;j<=n;j++) v[i][j]=v[j][i]=0.0;
		}
		v[i][i]=1.0;
		g=rv1[i];
		l=i;
	}
	for (i=IMIN(m,n);i>=1;i--) {
		l=i+1;
		g=w[i];
		for (j=l;j<=n;j++) a[i][j]=0.0;
		if (g) {
			g=1.0/g;
			for (j=l;j<=n;j++) {
				for (s=0.0,k=l;k<=m;k++) s += a[k][i]*a[k][j];
				f=(s/a[i][i])*g;
				for (k=i;k<=m;k++) a[k][j] += f*a[k][i];
			}
			for (j=i;j<=m;j++) a[j][i] *= g;
		} else for (j=i;j<=m;j++) a[j][i]=0.0;
		++a[i][i];
	}
	for (k=n;k>=1;k--) {
		for (its=1;its<=30;its++) {
			flag=1;
			for (l=k;l>=1;l--) {
				nm=l-1;
				if ((float)(fabs(rv1[l])+anorm) == anorm) {
					flag=0;
					break;
				}
				if ((float)(fabs(w[nm])+anorm) == anorm) break;
			}
			if (flag) {
				c=0.0;
				s=1.0;
				for (i=l;i<=k;i++) {
					f=s*rv1[i];
					rv1[i]=c*rv1[i];
					if ((float)(fabs(f)+anorm) == anorm) break;
					g=w[i];
					h=pythag(f,g);
					w[i]=h;
					h=1.0/h;
					c=g*h;
					s = -f*h;
					for (j=1;j<=m;j++) {
						y=a[j][nm];
						z=a[j][i];
						a[j][nm]=y*c+z*s;
						a[j][i]=z*c-y*s;
					}
				}
			}
			z=w[k];
			if (l == k) {
				if (z < 0.0) {
					w[k] = -z;
					for (j=1;j<=n;j++) v[j][k] = -v[j][k];
				}
				break;
			}
			if (its == 30) nrerror("no convergence in 30 svdcmp iterations");
			x=w[l];
			nm=k-1;
			y=w[nm];
			g=rv1[nm];
			h=rv1[k];
			f=((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
			g=pythag(f,1.0);
			f=((x-z)*(x+z)+h*((y/(f+SIGN(g,f)))-h))/x;
			c=s=1.0;
			for (j=l;j<=nm;j++) {
				i=j+1;
				g=rv1[i];
				y=w[i];
				h=s*g;
				g=c*g;
				z=pythag(f,h);
				rv1[j]=z;
				c=f/z;
				s=h/z;
				f=x*c+g*s;
				g = g*c-x*s;
				h=y*s;
				y *= c;
				for (jj=1;jj<=n;jj++) {
					x=v[jj][j];
					z=v[jj][i];
					v[jj][j]=x*c+z*s;
					v[jj][i]=z*c-x*s;
				}
				z=pythag(f,h);
				w[j]=z;
				if (z) {
					z=1.0/z;
					c=f*z;
					s=h*z;
				}
				f=c*g+s*y;
				x=c*y-s*g;
				for (jj=1;jj<=m;jj++) {
					y=a[jj][j];
					z=a[jj][i];
					a[jj][j]=y*c+z*s;
					a[jj][i]=z*c-y*s;
				}
			}
			rv1[l]=0.0;
			rv1[k]=f;
			w[k]=x;
		}
	}



	front=1;
	back=n;
	for(i=1;i<=n;i++)
	{
		if(w[i]<tol)
		{
			rv1[back]=i;
			back--;
		}
		else
		{
			rv1[front]=i;
			front++;
		}
	}

	for(i=1;i<=n;i++)
	{
		rv2[i]=w[(int)(rv1[i])];
		for(j=1;j<=n;j++)
			vt[j][i]=v[j][(int)(rv1[i])];
		for(j=1;j<=m;j++)
			ut[j][i]=a[j][(int)(rv1[i])];
	}

	for(i=1;i<=n;i++)
	{
		w[i]=rv2[i];
		for(j=1;j<=n;j++)
			v[j][i]=vt[j][i];
		for(j=1;j<=m;j++)
			a[j][i]=ut[j][i];
	}

	free_vector(rv1,1,n);
	free_vector(rv2,1,n);
	free_matrix(vt,1,n,1,m);
	free_matrix(ut,1,m,1,n);

}




} // extern C



SVD::SVD(const Matrix &A)
  : M(A.rows()), N(A.cols())
{
  float **v, *w, **u;

  // convert A to an NR matrix (put in u, which will be replaced)
  v=::matrix(1,N,1,N);	// v matrix
  u=::matrix(1,M,1,N);	// u matrix has extra columns of zeros
  w=::vector(1,N);	// vector of singular values
  
  for(Int r=0; r<M; r++)
    for(Int c=0; c<N; c++)
      u[r+1][c+1] = float( A(r,c) ); // NB: possible loss of precision (Real != float necesarily)
  
  // call NR SVD
  svdcmp(u, int(M), int(N), w, v);
  
  // now store results into U, V & s
  UMat = zeroMatrix(M,M);
  for(Int r=0; r<M; r++)
    for(Int c=0; c< Math::minimum(N,M); c++)
      UMat(r,c) = Real( u[r+1][c+1] );
  
  VMat.resize(N,N);
  for(Int r=0; r<N; r++)
    for(Int c=0; c<N; c++)
      VMat(r,c) = Real( v[r+1][c+1] );
  
  s.resize(N);
  for(Int i=0; i<N; i++)
    s[i] = Real( w[i+1] );
}



const Matrix& SVD::U() const
{
  return UMat;
}

const Matrix& SVD::V() const
{
  return VMat;
}

const Vector SVD::diag() const
{
  Vector d( Math::minimum(M,N) );
  d = vectorRange(s, Range(0, d.size()));
  return d;
}
  

Matrix SVD::S() const
{
  Matrix S( zeroMatrix(M,N) );
  for(Int i=0; i< Math::minimum(N,M); i++)
    S(i,i) = s[i];
  return S;
}


Real SVD::condition(Real maxCond) const
{
  Vector d(diag());
  Int minInd = min_index(d);
  Int maxInd = max_index(d);
   
  Real min = d[minInd];
  Real max = d[maxInd];

  if(min / max < 1.0/maxCond)
    return maxCond;
  else
    return max / min;

}


Matrix SVD::inv(Real minSingVal) const
{
  // combine the products to form the pseudo inverse solution
  const Int M = UMat.size1();
  const Int N = VMat.size1();
  Matrix tmpMat( zeroMatrix(N, M) ); // VMat * 'D^-1'
  matrixRange(tmpMat, Range(0,N), Range(0,N) ) = VMat;
   
  // invert the diagonal elements and apply to the tmpMat matrix
  for(Int i=0; i<s.size(); i++)
    {
      Real tmp = s[i];
      if(fabs(tmp) < minSingVal)
        tmp = 0.0;
      else
        tmp = 1.0 / tmp;
       
      matrixColumn(tmpMat,i) *= tmp;
    }
                                                                                                                                                                                                    
  // U^-1 == U^T
  Matrix invA(N, M);
  invA = tmpMat * transpose(UMat);
                                                                                                                                                                                                    
  return invA;
}
 

