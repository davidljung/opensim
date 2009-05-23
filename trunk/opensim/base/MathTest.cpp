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

  $Id: MathTest.cpp 1088 2004-09-13 17:28:59Z jungd $
  $Revision: 1.12 $
  $Date: 2004-09-13 13:28:59 -0400 (Mon, 13 Sep 2004) $
  $Author: jungd $

****************************************************************************/

#include <base/MathTest>

#include <base/SVD>
#include <base/Path>

using base::MathTest;

using base::consts;
using base::Matrix;
using base::Vector;
using base::SVD;
using base::Path;



MathTest::MathTest()
  : I(4,4), A(4,4), S(3,3),
    b(4), x(4)
{
}


void MathTest::setUp()
{
  // identity
  I = identityMatrix(4,4);

  // a matrix with a unique inverse
  A(0,0) = 1; A(0,1) = -1; A(0,2) = 0; A(0,3) = 0;
  A(1,0) = 2; A(1,1) = -2; A(1,2) = 1; A(1,3) = 2;
  A(2,0) = 0; A(2,1) = 1;  A(2,2) = 0; A(2,3) = 1;
  A(3,0) = 0; A(3,1) = 0;  A(3,2) = 2; A(3,3) = 1;

  b[0] = 0; b[1] = 4; b[2] = 0; b[3] = 5;

  x[0] = -1; x[1]=-1; x[2] = 2; x[3] = 1; // Ax=b

  S(0,0) = 1; S(0,1) = 2; S(0,2) = 3; // singluar
  S(1,0) = 1; S(1,1) = 2; S(1,2) = 2;
  S(2,0) = 1; S(2,1) = 2; S(2,2) = 3;


  P.resize(3,9);
  P(0,0) = 1; P(0,1) = 0; P(0,2) = -0.71595; P(0,3) = -0.13884; P(0,4) = -0.46236; P(0,5) = -0.29068; P(0,6) = 0.06875; P(0,7) = -0.36885; P(0,8) = -0.24926;
  P(1,0) = 0; P(1,1) = 1; P(1,2) = 0.94988;  P(1,3) = 0.20359;  P(1,4) = -0.66261; P(1,5) = -0.98019; P(1,6) = -0.55012; P(1,7) = -0.10177; P(1,8) = 0.23327;
  P(2,0) = 0; P(2,1) = 0; P(2,2) = 0;        P(2,3) = 0;        P(2,4) = -0.11290; P(2,5) = -0.53204; P(2,6) = -0.54088; P(2,7) = -0.60098; P(2,8) = -0.20846;

}


void MathTest::tearDown()
{
}


void MathTest::testMatrixVector()
{
  // just test some elementary Matrix & Vector ops first
  Matrix m(3,3);
  m = identityMatrix(3,3);
  for(Int r=0; r<3; r++)
    for(Int c=0; c<3; c++)
      CPPUNIT_ASSERT( m(r,c) == ((r==c)?1:0) );

  Vector u(3);
  u = unitVector(3,2);
  CPPUNIT_ASSERT( equals(norm_2(u),1) );
  CPPUNIT_ASSERT( u[2] == 1 );
  Vector mr(3), mc(3);
  mr = matrixRow(m,2);
  mc = matrixColumn(m,1);

  CPPUNIT_ASSERT( mr[2] == 1 );
  CPPUNIT_ASSERT( mc[1] == 1 );

  CPPUNIT_ASSERT( norm_2(zeroVector(3)) == 0 );

  Matrix z(3,3);
  z = zeroMatrix(3,3);
  CPPUNIT_ASSERT( norm_2(matrixRow(z,0)) == 0 );

  matrixRow(z,0) = unitVector(3,2);
  CPPUNIT_ASSERT( norm_2(matrixRow(z,0)) == 1 );

  Vector v6(zeroVector(6));
  vectorRange(v6,Range(0,3)) = unitVector(3,0);
  CPPUNIT_ASSERT( v6[0] == 1 );

  Matrix sm(2,2);
  sm = matrixRange(m,Range(0,2), Range(0,2));
  CPPUNIT_ASSERT( base::equals(sm, identityMatrix(2,2)) );

  Matrix m1(3,3);
  m1(0,0) = 1; m1(0,1) = 2; m1(0,2) = 3;
  m1(1,0) = 4; m1(1,1) = 5; m1(1,2) = 6;
  m1(2,0) = 7; m1(2,1) = 8; m1(2,2) = 9;


  Matrix newm1(m1);
  matrixRange(newm1, Range(0,2), Range(0,2)) = identityMatrix(2,2);
  CPPUNIT_ASSERT( base::equals( matrixRange(newm1, Range(0,2), Range(0,2)), identityMatrix(2,2)) );


  Matrix m2(3,3);
  m2(0,0) = 0; m2(0,1) = 3; m2(0,2) = 6;
  m2(1,0) = 7; m2(1,1) = 3; m2(1,2) = 5;
  m2(2,0) = -4;m2(2,1) = 5; m2(2,2) = -3;

  // test simple multiplication
  Matrix m12(3,3); // m1*m2
  m12(0,0) = 2; m12(0,1) = 24; m12(0,2) = 7;
  m12(1,0) = 11;m12(1,1) = 57; m12(1,2) = 31;
  m12(2,0) = 20;m12(2,1) = 90; m12(2,2) = 55;

  CPPUNIT_ASSERT( base::equals(m1*m2, m12) );


  // and matrix*vector mult
  Vector v1(3);
  v1[0] = 2; v1[1] = 5; v1[2] = 7;

  Vector m1v1(3);
  m1v1[0] = 33; m1v1[1] = 75; m1v1[2] = 117;

  CPPUNIT_ASSERT( base::equals(m1*v1, m1v1) );

  // test correct negate behaviour
  Vector n(1); n[0]=1;
  -n;
  CPPUNIT_ASSERT( base::equals(n[0], 1) );
  n = -n;
  CPPUNIT_ASSERT( base::equals(n[0], -1) );

  Matrix nm(1,1);
  nm(0,0) = 1;
  -nm;
  CPPUNIT_ASSERT( base::equals(nm(0,0), 1) );
  nm = -nm;
  CPPUNIT_ASSERT( base::equals(nm(0,0), -1) );

}




void MathTest::testRanges()
{
  Vector v1(b); // 0,4,0,5
  Vector v2(x); // -1,-1,2,1

  x[0] = -1; x[1]=-1; x[2] = 2; x[3] = 1; // Ax=b

  vectorRange(v1, Range(0,2)) = vectorRange(v2, Range(0,2));

  Vector v3(4);
  v3[0] = -1; v3[1] = -1; v3[2] = 0; v3[3] = 5;
//!!! this fails, but should pass !!!!
  CPPUNIT_ASSERT( v1.equals(v3) );
}




void MathTest::testNullSpace()
{
  Vector r(3); r[0]=1; r[1]=2; r[2]=3;

  Matrix A(3,3);
  matrixRow(A,0) = r;
  matrixRow(A,1) = r;
  matrixRow(A,2) = r;

  Int rank;
  Real k2;
  Matrix Z( Math::nullSpace(A,rank,k2) );

  CPPUNIT_ASSERT( equals(rank,2) ); // rank of null space (not of A)

  CPPUNIT_ASSERT( base::equals(A*Z, zeroMatrix(3,2), 0.00001) );

  Vector x(3);
  for(Int i=0; i<10; i++) {
    Real r = Math::random();
    Real s = r-1;
    x = matrixColumn(Z,0)*r + matrixColumn(Z,1)*s;
    CPPUNIT_ASSERT( equals(A*x, zeroVector(3), 0.00001) );
  }

}


void MathTest::testInverse()
{
  // check fixture first
  Assert( equals(A*x,b) );
  Assert( A.size1() == A.size2() ); // invert only works for square Matrices

  // calc inverse and check it's the right size
  Matrix Ainv( Math::inverse(A) );
  CPPUNIT_ASSERT( Ainv.size1() == A.size1() );
  CPPUNIT_ASSERT( Ainv.size2() == A.size2() );

  // mult A by its inverse, should give same size
  Matrix AAinv( A*Ainv );
  CPPUNIT_ASSERT( AAinv.size1() == A.size1() );
  CPPUNIT_ASSERT( AAinv.size2() == Ainv.size2() );

  // and = identity
  CPPUNIT_ASSERT( equals(AAinv, I) );

  // check we get x back
  CPPUNIT_ASSERT( equals(Ainv*b, x) );
}


void MathTest::testInverseSingular()
{
  // should throw an std::invalid_argument exception
  Matrix Sinv( Math::inverse(S) );
}


void MathTest::testSVD()
{
  // decompose square A
  SVD svda(A);

  Matrix U(svda.U());
  Matrix V(svda.V());
  Matrix S(svda.S());

  Matrix NA( U * S * transpose(V) );

  CPPUNIT_ASSERT( NA.equals(A,0.000001) );

  // compare singular values with those calc'd using Octave
  Vector so(4);
  so[3] = 3.95858;
  so[2] = 2.16562;
  so[1] = 1.24951;
  so[0] = 0.28007;

  // sort from lowest to highest
  bool swapped=false;
  do {
    swapped = false;
    for(Int i=0; i<3; i++)
    if (S(i,i) > S(i+1,i+1)) {
      base::swap( S(i,i), S(i+1,i+1) );
      swapped=true;
    }
  } while (swapped);

  Assert(S.size1() == 4);
  for(Int i=0; i<S.size1(); i++) {
    CPPUNIT_ASSERT( Math::equals(S(i,i), so[i], 0.00001) );
  }


  // and a non-square matrix (underconstrained)
  Matrix A1(A.rows(), A.cols()+2);
  matrixRange(A1, Range(0,A.rows()), Range(0,A.cols())) = A;
  Vector v1( zeroVector(A.rows()) );
  v1[0] = 1; v1[1] = 5; v1[2] = -0.2; v1[v1.size()-1] = -4;
  MatrixColumn(A1, A.cols()) = v1;
  v1[0] = 3; v1[1] = -6; v1[2] = -0.3; v1[v1.size()-1] = 2;
  MatrixColumn(A1, A.cols()+1) = v1;

  const Int M1 = A1.rows();
  const Int N1 = A1.cols();

  SVD svda1(A1);

  Matrix U1(svda1.U());
  Matrix V1(svda1.V());
  Matrix S1(svda1.S());

  CPPUNIT_ASSERT( U1.rows() == M1 );
  CPPUNIT_ASSERT( U1.cols() == M1 );
  CPPUNIT_ASSERT( S1.rows() == M1 );
  CPPUNIT_ASSERT( S1.cols() == N1 );
  CPPUNIT_ASSERT( V1.rows() == N1 );
  CPPUNIT_ASSERT( V1.cols() == N1 );

  Matrix NA1( U1 * S1 * transpose(V1) );

  CPPUNIT_ASSERT( NA1.equals(A1,0.00001) );


  // a non-square matrix (overconstrained)
  Matrix A2(A.rows()+2, A.cols());
  matrixRange(A2, Range(0,A.rows()), Range(0,A.cols())) = A;
  Vector v( zeroVector(A.cols()) );
  v[0] = 1; v[1] = 5; v[2] = -0.2; v[v.size()-1] = -4;
  MatrixRow(A2, A.rows()) = v;
  v[0] = 3; v[1] = -6; v[2] = -0.3; v[v.size()-1] = 2;
  MatrixRow(A2, A.rows()+1) = v;

  const Int M = A2.rows();
  const Int N = A2.cols();

  SVD svda2(A2);

  Matrix U2(svda2.U());
  Matrix V2(svda2.V());
  Matrix S2(svda2.S());

  CPPUNIT_ASSERT( U2.rows() == M );
  CPPUNIT_ASSERT( U2.cols() == M );
  CPPUNIT_ASSERT( S2.rows() == M );
  CPPUNIT_ASSERT( S2.cols() == N );
  CPPUNIT_ASSERT( V2.rows() == N );
  CPPUNIT_ASSERT( V2.cols() == N );

  Matrix NA2( U2 * S2 * transpose(V2) );

  CPPUNIT_ASSERT( NA2.equals(A2,0.00001) );
}



void MathTest::testPseudoInverse()
{
  Matrix PInv( Math::pseudoInverse(P) );

  Matrix PIO(9,3); // pinv according to Octave

  PIO(0,0) = 0.5819252;   PIO(0,1) = 0.1170694;  PIO(0,2) = -0.3666462;
  PIO(1,0) =  0.1170694;  PIO(1,1) = 0.3703392;  PIO(1,2) = -0.3903029;
  PIO(2,0) = -0.3054275;  PIO(2,1) = 0.2679620;  PIO(2,2) = -0.1082405;
  PIO(3,0) = -0.0569603;  PIO(3,1) = 0.0591435;  PIO(3,2) = -0.0285566;
  PIO(4,0) = -0.3052359;  PIO(4,1) = -0.2554535; PIO(4,2) =  0.2557433;
  PIO(5,0) = -0.0888338;  PIO(5,1) = -0.1893758; PIO(5,2) = -0.3232752;
  PIO(6,0) =  0.1739168;  PIO(6,1) = 0.0154245;  PIO(6,2) = -0.6364151;
  PIO(7,0) = -0.0062102;  PIO(7,1) = 0.1536938;  PIO(7,2) = -0.7427355;
  PIO(8,0) = -0.0413108;  PIO(8,1) = 0.1385709;  PIO(8,2) = -0.3179733;

  CPPUNIT_ASSERT( PInv.equals(PIO, 0.00001) );
}




void MathTest::testExpression()
{
  Expression e("tan(0.1)*-2*sin(p[0])+-4.4e-1*((1+1.00)*0.5)/cos(p[0]+0.01+0*(100.0/-3.3e-8))");
  e.simplify();

  Vector p(1);
  p[0] = consts::Pi/2.0;

  CPPUNIT_ASSERT(Math::equals( e.evaluate(p), Math::tan(0.1)*-2.0*Math::sin(p[0])+-4.4e-1*((1+1)*0.5)/Math::cos(p[0]+0.01) ));
}




void MathTest::testPath()
{
  // check default path is 0
  Path zp;
  for(Real t=0.0; t<=1.0; t+=0.05) {
    CPPUNIT_ASSERT( zp.position(t).equalsZero() );
    CPPUNIT_ASSERT( zp.orientation(t).getVector3(Orient::EulerRPY).equalsZero() );
  }

  // unit length const orientation line segment
  Point3 p1(1,1,1);
  Point3 p2(2,2,2);
  Path up(p1,Orient(),p2,Orient());
  CPPUNIT_ASSERT( up.position(0.0).equals(p1) );
  CPPUNIT_ASSERT( up.position(1.0).equals(p2) );
  CPPUNIT_ASSERT( up.position(0.5).equals(Point3(1.5,1.5,1.5)) );
  CPPUNIT_ASSERT( up.orientation(0.5).getVector3(Orient::EulerRPY).equalsZero() );

  // test interpolation of both pos & orient
  Quat4 q1(Vector3(0,0,1),consts::Pi/4.0);
  Quat4 q2(Vector3(0,0,1),consts::Pi/2.0);
  Orient o1(q1), o2(q2);
  Path path(p1,o1,p2,o2);
  CPPUNIT_ASSERT( path.position(0.0).equals(p1) );
  CPPUNIT_ASSERT( path.position(1.0).equals(p2) );
  CPPUNIT_ASSERT( path.position(0.5).equals(Point3(1.5,1.5,1.5)) );
  CPPUNIT_ASSERT( path.orientation(0.0).equals(o1) );
  CPPUNIT_ASSERT( path.orientation(1.0).equals(o2) );
  CPPUNIT_ASSERT( path.orientation(0.5).equals(Orient::interpolate(o1,o2,0.5)) );

  array<Point3> ps(3);
  array<Orient> os(3);
  ps[0] = p1; ps[1] = Point3(1.5,3.0,-2.5); ps[2] = p2;
  Quat4 q3(Vector3(0,0,1),consts::Pi/3.0);
  Orient o3(q3);
  os[0] = o1; os[1] = o3; os[2] = o2;
  Path wpath(ps,os);
  CPPUNIT_ASSERT( wpath.position(0.0).equals(p1) );
  CPPUNIT_ASSERT( wpath.position(1.0).equals(p2) );
  CPPUNIT_ASSERT( wpath.position(0.5).equals(Point3(1.53099,2.93802,-2.22108),1e-4) );
  CPPUNIT_ASSERT( wpath.position(0.6).equals(Point3(1.62479,2.75042,-1.37687),1e-4) );
  CPPUNIT_ASSERT( wpath.position(0.8).equals(Point3(1.8124,2.37521,0.311566),1e-4) );

  CPPUNIT_ASSERT( wpath.orientation(0.0).equals(o1) );
  CPPUNIT_ASSERT( wpath.orientation(1.0).equals(o2) );

  // construct an orientation only (const position) path to test orientatin interpolation
  ps.clear();
  Path wpath2(ps,os);
  CPPUNIT_ASSERT( wpath2.orientation(0.0).equals(o1) );
  CPPUNIT_ASSERT( wpath2.orientation(1.0).equals(o2) );
  CPPUNIT_ASSERT( wpath2.orientation(0.5).getQuat4().equals(Quat4(Vector3(0,0,1),3.0*consts::Pi/8.0)) );


  // a parametric path
  ExpressionVector v(3);
  v[0] = base::cos( Expression::p[0] * 2.0 * consts::Pi ); v[0].simplify();
  v[1] = base::sin( Expression::p[0] * 2.0 * consts::Pi ); v[1].simplify();
  v[2] = 0;
  Path unitcircle(v);

  for(Real s=0.0; s<=1.0; s+=1/100.0) {
    Point3 p( Math::cos(s*2.0*consts::Pi), Math::sin(s*2.0*consts::Pi), 0);
    CPPUNIT_ASSERT( unitcircle.position(s).equals(p) );
  }

}


#ifdef DEBUG
CPPUNIT_TEST_SUITE_REGISTRATION( MathTest );
#endif

