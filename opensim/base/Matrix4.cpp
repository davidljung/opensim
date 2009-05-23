/****************************************************************************
  Copyright (C)1996 David Jung <opensim@pobox.com>

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
  
  $Id: Matrix4.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.7 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/Matrix4>

#include <base/Serializer>

using base::Matrix4;
using base::Vector4;
using base::Matrix3;

using std::ostream;


inline static void exchange(Real& a, Real& b)
{
  Real t(a);
  a = b;
  b = t;
}

// row, col element exchange
inline static void rcswap4(Real m[], Int r, Int c) 
{
	Real* rcp = &m[MATRIX4_ACCESS(r,c)];
	Real* crp = &m[MATRIX4_ACCESS(c,r)];
	exchange(*rcp, *crp);
}


void Matrix4::setToTranslation(const Vector3& trans)
{
  setDiag(Real(1));
  e(1,4) = trans.x;
  e(2,4) = trans.y;
  e(3,4) = trans.z;
}

void Matrix4::setTranslationComponent(const Vector3& trans)
{
  e(1,4) = trans.x;
  e(2,4) = trans.y;
  e(3,4) = trans.z;
}

void Matrix4::setRotationAboutZ(Real angle)
{
  Real cosa = cos(angle);
  Real sina = sin(angle);
  setDiag(Real(1));
  e(1,1)=cosa; e(1,2)=-sina;
  e(2,1)=sina; e(2,2)=cosa;
}


Matrix4& Matrix4::transpose()
{
  rcswap4(m,1,2); rcswap4(m,1,3); rcswap4(m,1,4);
  rcswap4(m,2,3); rcswap4(m,2,4);
  rcswap4(m,3,4);

  return *this;
}


Matrix4& Matrix4::invert()
{
  // Solve AX = I by solving 4 eqns: Ax = e (for each e a col of I)
  Matrix4 I; // identity
  Matrix4 X, A(*this);
  Vector4 x;
  
  Matrix4 L,U;
  Vector4 Pi;
  
  A.decomposeLUP(L,U,Pi);
  
  for(int c=1; c<=4; c++) {
    x = solveLUP(L,U,Pi,I.column(c));
    for(int r=1; r<=4; r++) X.e(r,c) = x[r];
  }
  
  return (*this = X);

}


void Matrix4::decomposeLUP(Matrix4& L, Matrix4& U, Vector4& Pi) const
{
  Real p,a;
  int kp,i,j,k;
  Matrix4 AA(*this);

  for(i=1; i<=4; i++) Pi[i] = i;
  for(k=1; k<=4-1; k++) {
    p = Real(0); kp = 1;
    for(i=k; i<=4; i++) {
      a = Real(base::abs(AA.e(i,k)));
      if ( a > p ) {
	p = a;
	kp = i;
      }
    }
    if (p == Real(0))  
      throw std::invalid_argument(Exception("matrix is singular"));
    
    exchange(Pi[k],Pi[kp]);
    for(i=1; i<=4;i++) exchange(AA.e(k,i),AA.e(kp,i));
    
    for(i=k+1; i<=4;i++) {
      AA.e(i,k) /= AA.e(k,k);
      for(j=k+1; j<=4; j++)
	AA.e(i,j) -= AA.e(i,k)*AA.e(k,j);
    }
  }
  
  for(i=1;i<=4;i++) {
    for(j=1;j<=4;j++)
      if (i>j) {
	L.e(i,j) = AA.e(i,j); U.e(i,j) = Real(0);
      }
      else {
	U.e(i,j) = AA.e(i,j); L.e(i,j) = Real(0);
      }
    L.e(i,i) = Real(1);
    
  }

}


Vector4 Matrix4::solve(const Vector4& b) const
{
  Matrix4 L,U;
  Vector4 Pi;

  decomposeLUP(L,U,Pi);
  return solveLUP(L,U,Pi,b);
}


Matrix4& Matrix4::negate() throw()
{
  for(int i=0; i<16; i++)
    m[i]=-m[i];
  return *this;
}


// Operators

// multiplication
Matrix4& Matrix4::operator*=(const Matrix4& m2)
{
  // !!! optimize (unroll loops)
  Matrix4 tmp;

  for(int row=1; row<=4; row++)
    for(int col=1; col<=4; col++)
      tmp.e(row,col) =  (e(row,1) * m2.e(1,col)) + (e(row,2) * m2.e(2,col))
      	              + (e(row,3) * m2.e(3,col)) + (e(row,4) * m2.e(4,col));
 
  return (*this = tmp);
}

// addition
Matrix4& Matrix4::operator+=(const Matrix4& m2)
{
  for(int i=0; i<16; i++)
    m[i] += m2.m[i];
  return (*this);
}

// subtration
Matrix4& Matrix4::operator-=(const Matrix4& m2)
{
  for(int i=0; i<16; i++)
    m[i] -= m2.m[i];
  return (*this);
}

// Scalar multiplication
Matrix4& Matrix4::operator*=(const Real& s)
{
  for(int i=0; i<16; i++)
    m[i] *= s;
  return *this;
}

// Scalar division
Matrix4& Matrix4::operator/=(const Real& s)
{
  for(int i=0; i<16; i++)
    m[i] /= s;
  return *this;
}


Matrix4::operator Matrix3() const
{
  Matrix3 r;
  r.e(1,1) = e(1,1); r.e(1,2)=e(1,2); r.e(1,3)=e(1,3);
  r.e(2,1) = e(2,1); r.e(2,2)=e(2,2); r.e(2,3)=e(2,3);
  r.e(3,1) = e(3,1); r.e(3,2)=e(3,2); r.e(3,3)=e(3,3);
  return r;
}


#ifdef USE_OSG
Matrix4::operator osg::Matrix() const
{
#ifdef OSG_USE_DOUBLE_MATRICES
    typedef double osgreal;
#else
    typedef float osgreal;
#endif
  // NB: This relies on the two types having the same
  osg::Matrix mat;
  osgreal* _mat = mat.ptr();
  for(Int e=0; e<16; e++)
    _mat[e] = m[e];
  return mat;
}
#endif


Vector4 Matrix4::matrixMulVector(const Matrix4& m, const Vector4& v) 
{ return Vector4(m.m[0]*v.x+m.m[4]*v.y+m.m[8]*v.z+m.m[12]*v.w,
                 m.m[1]*v.x+m.m[5]*v.y+m.m[9]*v.z+m.m[13]*v.w,
                 m.m[2]*v.x+m.m[6]*v.y+m.m[10]*v.z+m.m[14]*v.w,
                 m.m[3]*v.x+m.m[7]*v.y+m.m[11]*v.z+m.m[15]*v.w);
}

/* above is equivelent to: (which is slower but easier to read)
Vector4 Matrix4::matrixMulVector(const Matrix4& m, const Vector4& v) 
{ return Vector4(m.e(1,1)*v.x+m.e(1,2)*v.y+m.e(1,3)*v.z+m.e(1,4)*v.w,
                 m.e(2,1)*v.x+m.e(2,2)*v.y+m.e(2,3)*v.z+m.e(2,4)*v.w,
                 m.e(3,1)*v.x+m.e(3,2)*v.y+m.e(3,3)*v.z+m.e(3,4)*v.w,
                 m.e(4,1)*v.x+m.e(4,2)*v.y+m.e(4,3)*v.z+m.e(4,4)*v.w);
}
*/

Vector4 Matrix4::matrixMulVector(const Matrix4& m, const Vector3& v) 
{ return Vector4(m.m[0]*v.x+m.m[4]*v.y+m.m[8]*v.z+m.m[12],
                 m.m[1]*v.x+m.m[5]*v.y+m.m[9]*v.z+m.m[13],
                 m.m[2]*v.x+m.m[6]*v.y+m.m[10]*v.z+m.m[14],
                 m.m[3]*v.x+m.m[7]*v.y+m.m[11]*v.z+m.m[15]);
}

Vector4 Matrix4::matrixMulVectorAddVector(const Matrix4& m, const Vector4& v, const Vector4& v2) 
{ return Vector4(m.m[0]*v.x+m.m[4]*v.y+m.m[8]*v.z+m.m[12]*v.w+v2.x,
                 m.m[1]*v.x+m.m[5]*v.y+m.m[9]*v.z+m.m[13]*v.w+v2.y,
                 m.m[2]*v.x+m.m[6]*v.y+m.m[10]*v.z+m.m[14]*v.w+v2.z,
                 m.m[3]*v.x+m.m[7]*v.y+m.m[11]*v.z+m.m[15]*v.w+v2.w);
}


// Solve for Ax = b, given LUP decomposition of A as L, U and Pi, and given b, returns x
Vector4 Matrix4::solveLUP(const Matrix4& L, const Matrix4& U, const Vector4& Pi, const Vector4& b)
{
  int i,j;
  Vector4 y, x;
  Real s;
  // forward subst.
  for(i=1;i<=4;i++) {
    s = Real(0);
    for(j=1;j<=i-1;j++) s += L.e(i,j)*y[j];
    y[i] = b[int(Pi[i])] - s;
  }
  // backward subst.
  for(i=4;i>=1;i--) {
    s = Real(0);
    for(j=i+1;j<=4;j++) s += U.e(i,j)*x[j];
    x[i] = (y[i] - s)/(U.e(i,i));
  }
  return x;
}


void Matrix4::serialize(Serializer& s)
{
  for(Int i=0; i<15; i++) 
    s(m[i]);
}


// output
ostream& base::operator<<(ostream& out, const Matrix4& m)
{
  return out << m.e(1,1) << " " << m.e(1,2) << " " << m.e(1,3) << " " << m.e(1,4) << std::endl
             << m.e(2,1) << " " << m.e(2,2) << " " << m.e(2,3) << " " << m.e(2,4) << std::endl
	     << m.e(3,1) << " " << m.e(3,2) << " " << m.e(3,3) << " " << m.e(3,4) << std::endl
	     << m.e(4,1) << " " << m.e(4,2) << " " << m.e(4,3) << " " << m.e(4,4) << std::endl;
}







