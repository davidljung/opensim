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

  $Id: Math.cpp 1084 2004-09-13 17:18:15Z jungd $
  $Revision: 1.17 $
  $Date: 2004-09-13 13:18:15 -0400 (Mon, 13 Sep 2004) $
  $Author: jungd $

****************************************************************************/

#include <base/Math>

#include <base/SVD>

using base::Math;

using base::Matrix;
using base::Vector;
using base::swap;
using base::SVD;


// define this to use the old IKORv2 mat_null for null-space computation instead of
//  the class SVD in method nullSpace().  Although both provide the null-space of
//  a matrix, the spanning vector set is not unique - and each routine typically
//  produces a different set.  This is useful for comparison with old code.
//  (NB: the old IKORv2 code must be included in the build via externally defining
//   BUILDOLDIKOR)
//#define USEOLDIKOR_NULLSPACE

#ifdef BUILDOLDIKOR
  #ifdef USEOLDIKOR_NULLSPACE
  extern "C" {
    #include <robot/control/oldikor/IKOR/general.h>
    #include <robot/control/oldikor/IKOR/headers.h>
    #include <robot/control/oldikor/UTILS/matrix.h>
  }
  #endif
#else
  #undef USEOLDIKOR_NULLSPACE
#endif



Real Math::angleDifference(Real angle1, Real angle2)
{
  Real a1 = normalizeAngle(angle1);
  Real a2 = normalizeAngle(angle2);
  if (a1 >= 0) {
    if (a2 >= 0)
      return a1-a2;
    else {
      if ((a1-a2) <= consts::Pi)
        return a1-a2;
      else
        return (-consts::Pi-a2)-(consts::Pi-a1);
    }
  }
  else {
    if (a2 < 0)
      return a1-a2;
    else {
      if ((a1-a2) > -consts::Pi)
        return a1-a2;
      else
        return (consts::Pi-a2)-(-consts::Pi-a1);
    }
  }
}







void Math::decomposeLUP(const Matrix& A, Matrix& L, Matrix& U, Vector& Pi, Real epsilon)
{
  const Int n = A.size1();

  Assertm( (L.size1() == n) && (L.size2() == n)
           && (U.size1() == n) && (U.size2() == n) && (Pi.size() == n),
           "L,U,Pi are square and of same dimension as A");

  Real p,a;
  Int kp,i,j,k;
  Matrix AA(A);

  for(i=0; i<n; i++) Pi[i] = i;
  for(k=0; k<n-1; k++) {
    p = Real(0); kp = 0;
    for(i=k; i<n; i++) {
      a = Real(abs(AA(i,k)));
      if ( a > p ) {
        p = a;
        kp = i;
      }
    }
    if (equals(p,0,epsilon))
      throw std::invalid_argument(Exception("Matrix A is singular"));

    swap(Pi[k],Pi[kp]);
    for(i=0; i<n;i++) swap(AA(k,i),AA(kp,i));

    for(i=k+1; i<n;i++) {
      AA(i,k) /= AA(k,k);
      for(j=k+1; j<n; j++)
        AA(i,j) -= AA(i,k)*AA(k,j);
    }
  }

  for(i=0;i<n;i++) {
    for(j=0;j<n;j++)
      if (i>j) {
        L(i,j) = AA(i,j); U(i,j) = 0;
      }
      else {
        U(i,j) = AA(i,j); L(i,j) = 0;
      }
    L(i,i) = Real(1);

  }

}


Vector Math::solveLUP(const Matrix& L, const Matrix& U, const Vector& Pi, const Vector& b)
{
  Int n = L.size1();

  Assertm( (b.size() == n) && (L.size1() == n)
           && (U.size1()==n) || (U.size2() == n) && (Pi.size() == n),
           "L,U,Pi must are square and of same dimension (that of b)");

  SInt i,j;
  Vector y(n), x(n);
  Real s;
  // forward subst.
  for(i=0;i<SInt(n);i++) {
    s = 0;
    for(j=0;j<i;j++) s += L(i,j)*y[j];
    y[i] = b[Int(Pi[i])] - s;
  }
  // backward subst.
  for(i=n-1;i>=0;i--) {
    s = 0;
    for(j=i+1;j<SInt(n);j++) s += U(i,j)*x[j];
    x[i] = (y[i] - s)/(U(i,i));
  }
  return x;
}




Matrix Math::inverse(const Matrix& A, Real epsilon)
{
  const Int nr = A.size1();
  const Int nc = A.size2();

  if (nr == nc) {
    // Solve AX = I by solving eqns: Ax = e (for each e a col of I)
    Matrix X(nr,nc);
    Vector x(nr);

    Matrix L(nr,nc),U(nr,nc);
    Vector Pi(nr);

    decomposeLUP(A, L, U, Pi, epsilon);

    for(Int c=0; c<nc; c++) {
      x = solveLUP(L,U,Pi,unitVector(nc,c));
      for(Int r=0; r<nr; r++) X(r,c) = x[r];
    }

    return X;
  }
  else
    throw std::invalid_argument(Exception("Can't invert a non-square matrix"));

}


Matrix Math::nullSpace(const Matrix& A, Int& nullSpaceRank, Real& k2)
{
  if ((A.size1() == 0) || (A.size2() == 0))
    throw std::invalid_argument(Exception("Can't compute null-space of a 0x0 matrix!"));

#ifndef USEOLDIKOR_NULLSPACE
  // this is a transliteration of the IKORv2.0 function mat_null() (in matrix.c)
  // (except that mat_null uses the Numerical Recipies in C function svdcmp() and
  //  we use class SVD)

  Assert(A.size1() <= A.size2());

  const Int R = A.size1(); // M
  const Int C = A.size2(); // N

  // make R == C by adding rows of zeros if necessary
  Matrix Asqr(C,C);
  if (R < C) {
    matrixRange(Asqr,Range(0,R), Range(0,C)) = A;
    matrixRange(Asqr,Range(R,C), Range(0,C)) = zeroMatrix(C-R,C);
  }
  else
    Asqr = A;

  SVD svd(Asqr);

  // reorder (and record the reordering in 'order')
  const Matrix& V(svd.V());
  Vector  S(svd.diag());

  IVector order(C);
  for(Int i=0; i<C; i++) order[i]=i;

  for(SInt i=C-1; i>0; i--)
    for(SInt j=i-1; j>=0; j--)
      if ( Math::abs(S[i]) > Math::abs(S[j]) ) {
        base::swap(S[i],S[j]);
        base::swap(order[i],order[j]);
      }

  Real s_min;
  if (R<=C)
    s_min = Math::abs(S[R-1]);
  else
    s_min = Math::abs(S[C-1]);
  Real s_max = Math::abs(S[0]);


  Matrix ns(C,C); // truncate unused columns before return
  nullSpaceRank=0;
  while (Math::abs(S[C-1-nullSpaceRank]) < SVD::minSingValue) {
    for(Int j=0; j<C; j++)
      ns(j,nullSpaceRank) = V(j,order[C-1-nullSpaceRank]);
    nullSpaceRank++;
    if (nullSpaceRank == C) break;
  }

  if (nullSpaceRank < C) {
    if ( (Math::abs(s_min)>=SVD::minSingValue) && (Math::abs(s_min/s_max) < SVD::minSingValue)) {
      for(Int j=0; j<C; j++)
        ns(j,nullSpaceRank) = V(order[j],C-1-nullSpaceRank);
      nullSpaceRank++;
    }
  }

  if (equals(s_min,0,SVD::minSingValue))
    k2 = consts::maxReal;
  else
    k2 = s_max/s_min;

  Matrix n(C,nullSpaceRank);
  if ((C > 0) && (nullSpaceRank > 0))
    n = matrixRange(ns, Range(0,C), Range(0,nullSpaceRank));

  return n;

#else
  // use the old IKOR mat_null() function so we get exactly the same
  //  vectors to aid debugging

  MATRIX *a = mat_malloc(A.size1(), A.size2());
  for(Int r=0; r<A.size1(); r++)
    for(Int c=0; c<A.size2(); c++)
      a->p[r][c] = A(r,c);

  //  Int m = Math::maximum(A.size1(), A.size2());
  MATRIX *n = mat_malloc(A.size2(), A.size1());
  int n_rank;
  float K2;
  mat_null(a, &n_rank, n, &K2);

  nullSpaceRank = n_rank;
  k2 = K2;

  Matrix nn(A.size2(), nullSpaceRank);
  for(Int r=0; r<A.size2(); r++)
    for(Int c=0; c<nullSpaceRank; c++)
      nn(r,c) = n->p[r][c];

  mat_free(a);
  mat_free(n);

  return nn;
#endif
}


Matrix Math::pseudoInverse(const Matrix& A)
{
  // pad first if necessary
  const Int M=A.size1(); // rows
  const Int N=A.size2(); // cols

  // Use Single-Value-Decomposition (SVD)
  //  this requires that A be MxN where M>=N.  If the system
  //  is under-determined, then M<N, so we need to
  //  pad A with rows of 0s to make it square

  if (M<N) {
    Matrix pA(N,N);
    matrixRange(pA,Range(0,M), Range(0,N)) = A;
    matrixRange(pA,Range(M,N), Range(0,N)) = zeroMatrix(N-M,N); // pad

    Matrix pAinv( SVD(pA).inv() ); // padded pseudo inverse of A

    return matrixRange(pAinv, Range(0,N), Range(0,M)); // remove padding
  }

  // M>=N
  return SVD(A).inv();
}



Matrix Math::S(const Vector& v)
{
  Matrix s(3,3);
  s(0,0) = 0;    s(0,1) = -v[2]; s(0,2) = v[1];
  s(1,0) = v[2]; s(1,1) = 0;     s(1,2) = -v[0];
  s(2,0) = -v[1];s(2,1) = v[0];  s(2,2) = 0;
  return s;
}

