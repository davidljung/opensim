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
  
  $Id: Matrix3.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.5 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/Matrix3>

#include <base/Math>
#include <base/Serializer>


using base::Math;
using base::Matrix3;
using base::Vector3;

using std::ostream;


inline static void exchange(Real& a, Real& b)
{
  Real t(a);
  a = b;
  b = t;
}

// row, col element exchange
inline static void rcswap3(Real m[], Int r, Int c) 
{
  Real* rcp = &m[MATRIX3_ACCESS(r,c)];
  Real* crp = &m[MATRIX3_ACCESS(c,r)];
  exchange(*rcp, *crp);
}


Matrix3& Matrix3::transpose()
{
  rcswap3(m,1,2); rcswap3(m,1,3);
  rcswap3(m,2,3);

  return *this;
}


Matrix3& Matrix3::invert()
{
  // Solve AX = I by solving 3 eqns: Ax = e (for each e a col of I)
  Matrix3 I; // identity
  Matrix3 X, A(*this);
  Vector3 x;
  
  Matrix3 L,U;
  Vector3 Pi;
  
  A.decomposeLUP(L,U,Pi);
  
  for(int c=1; c<=3; c++) {
    x = solveLUP(L,U,Pi,I.column(c));
    for(int r=1; r<=3; r++) X.e(r,c) = x[r];
  }
  
  return (*this = X);

}



void Matrix3::setOrthonormalBasisOf(const Vector3& v)
{
  Vector3 n(v); n.normalize();
  setRow(3, n);
  
  Vector3 u;
  if (Math::abs(n.x) >= Math::abs(n.y)) {
    // n.x or n.z is the largest magnitude component, swap them
    Real invLength = 1.0 / Math::sqrt(Math::sqr(n.x) + Math::sqr(n.z));
    u = Vector3(-n.z * invLength, 0, n.x * invLength);
  }
  else {
    // n.y or n.z is the largest magnitude component, swap them
    Real invLength = 1.0/Math::sqrt(Math::sqr(n.y) + Math::sqr(n.z));
    u = Vector3(0, n.z * invLength, n.y * invLength);
  }
  
  setRow(1, u);
  setRow(2, cross(n,u));
}


void Matrix3::decomposeLUP(Matrix3& L, Matrix3& U, Vector3& Pi) const
{
  Real p,a;
  int kp,i,j,k;
  Matrix3 AA(*this);

  for(i=1; i<=3; i++) Pi[i] = i;
  for(k=1; k<=3-1; k++) {
    p = Real(0); kp = 1;
    for(i=k; i<=3; i++) {
      a = Real(Math::abs(AA.e(i,k)));
      if ( a > p ) {
	p = a;
	kp = i;
      }
    }
    if (p == Real(0))
      throw std::invalid_argument(Exception("matrix is singular"));
    
    exchange(Pi[k],Pi[kp]);
    for(i=1; i<=3;i++) exchange(AA.e(k,i),AA.e(kp,i));
    
    for(i=k+1; i<=3;i++) {
      AA.e(i,k) /= AA.e(k,k);
      for(j=k+1; j<=3; j++)
	AA.e(i,j) -= AA.e(i,k)*AA.e(k,j);
    }
  }
  
  for(i=1;i<=3;i++) {
    for(j=1;j<=3;j++)
      if (i>j) {
	L.e(i,j) = AA.e(i,j); U.e(i,j) = Real(0);
      }
      else {
	U.e(i,j) = AA.e(i,j); L.e(i,j) = Real(0);
      }
    L.e(i,i) = Real(1);
    
  }

}


Vector3 Matrix3::solve(const Vector3& b) const
{
  Matrix3 L,U;
  Vector3 Pi;

  decomposeLUP(L,U,Pi);
  return solveLUP(L,U,Pi,b);
}


Matrix3& Matrix3::negate() throw()
{
  for(int i=0; i<9; i++)
    m[i]=-m[i];
  return *this;
}


// Operators

// multiplication
Matrix3& Matrix3::operator*=(const Matrix3& m2)
{
  Matrix3 tmp;

  for(int row=1; row<=3; row++)
    for(int col=1; col<=3; col++)
      tmp.e(row,col) =  (e(row,1) * m2.e(1,col)) + (e(row,2) * m2.e(2,col))
				                  + (e(row,3) * m2.e(3,col)); 
 
  return (*this = tmp);
}

// addition
Matrix3& Matrix3::operator+=(const Matrix3& m2)
{
  for(int i=0; i<9; i++)
    m[i] += m2.m[i];
  return (*this);
}

// subtration
Matrix3& Matrix3::operator-=(const Matrix3& m2)
{
  for(int i=0; i<9; i++)
    m[i] -= m2.m[i];
  return (*this);
}

// Scalar multiplication
Matrix3& Matrix3::operator*=(const Real& s)
{
  for(int i=0; i<9; i++)
    m[i] *= s;
  return *this;
}

// Scalar division
Matrix3& Matrix3::operator/=(const Real& s)
{
  for(int i=0; i<9; i++)
    m[i] /= s;
  return *this;
}



Vector3 Matrix3::matrixMulVector(const Matrix3& m, const Vector3& v) 
{ return Vector3(m.m[0]*v.x+m.m[3]*v.y+m.m[6]*v.z,
                 m.m[1]*v.x+m.m[4]*v.y+m.m[7]*v.z,
                 m.m[2]*v.x+m.m[5]*v.y+m.m[8]*v.z);
}

/* above is equivelent to: (which is slower but easier to read)
Vector4 Matrix4::matrixMulVector(const Matrix4& m, const Vector4& v) 
{ return Vector4(m.e(1,1)*v.x+m.e(1,2)*v.y+m.e(1,3)*v.z,
                 m.e(2,1)*v.x+m.e(2,2)*v.y+m.e(2,3)*v.z,
                 m.e(3,1)*v.x+m.e(3,2)*v.y+m.e(3,3)*v.z)
}
*/


Vector3 Matrix3::matrixMulVectorAddVector(const Matrix3& m, const Vector3& v, const Vector3& v2) 
{ return Vector3(m.m[0]*v.x+m.m[3]*v.y+m.m[6]*v.z+v2.x,
                 m.m[1]*v.x+m.m[4]*v.y+m.m[7]*v.z+v2.y,
                 m.m[2]*v.x+m.m[5]*v.y+m.m[8]*v.z+v2.z);
}


/// Solve for Ax = b, given LUP decomposition of A as L, U and Pi, and given b, returns x
Vector3 Matrix3::solveLUP(const Matrix3& L, const Matrix3& U, const Vector3& Pi, const Vector3& b)
{
  int i,j;
  Vector3 y, x;
  Real s;
  // forward subst.
  for(i=1;i<=3;i++) {
    s = Real(0);
    for(j=1;j<=i-1;j++) s += L.e(i,j)*y[j];
    y[i] = b[int(Pi[i])] - s;
  }
  // backward subst.
  for(i=3;i>=1;i--) {
    s = Real(0);
    for(j=i+1;j<=3;j++) s += U.e(i,j)*x[j];
    x[i] = (y[i] - s)/(U.e(i,i));
  }
  return x;
}


// eigenJacobi helper
//  (do we really need to pass g & h??)
inline void rotate(Matrix3& a, Real& g, Real& h, Real s, Real tau, Int i, Int j, Int k, Int l) {
  g=a.e(i,j); h=a.e(k,l); 
  a.e(i,j) = g-s*(h+g*tau);
  a.e(k,l) = h+s*(g-h*tau);
}


/// Computes the eigen values/vectors in dout & vout resp.
/** (returns the no. of iterations taken (max 50) )
 * see Numerical Recipies in C pp467 (Ch.11).
 */
Int Matrix3::eigenJacobi(Matrix3& vout, Vector3& dout, Int maxIter) const
{
  Int i;
  Real tresh,theta,tau,t,sm,s,h,g,c;
  Vector3 b,z,d;
  Matrix3 v;
  Matrix3 a(*this);
  
  b.x = a.e(1,1);
  b.y = a.e(2,2);
  b.z = a.e(3,3);
  d = b;
  z.setZero();
  
  Int nrot = 0;
  
  for(i=0; i<maxIter; i++) {
    
    sm=0.0; sm+=Math::abs(a.e(1,2)); sm+=Math::abs(a.e(1,3)); sm+=Math::abs(a.e(2,3));
    if (sm == 0.0) { 
      vout = v;
      dout = d;
      return i;
    }
    
    if (i < 3) tresh=0.2*sm/(3.0*3.0); else tresh=0.0;
    
    // loop unrolled
    {
      g = 100.0*Math::abs(a.e(1,2));  
      if (i>3 && Math::abs(d.x)+g==Math::abs(d.x) && Math::abs(d.y)+g==Math::abs(d.y))
	a.e(1,2)=0.0;
      else if (Math::abs(a.e(1,2))>tresh)
	{
	  h = d.y-d.x;
	  if (Math::abs(h)+g == Math::abs(h)) 
	    t=(a.e(1,2))/h;
	  else {
	    theta=0.5*h/(a.e(1,2));
	    t=1.0/(Math::abs(theta)+Math::sqrt(1.0+theta*theta));
	    if (theta < 0.0) t = -t;
	  }
	  c=1.0/Math::sqrt(1.0+t*t); s=t*c; tau=s/(1.0+c); h=t*a.e(1,2);
	  z.x -= h; z.y += h; d.x -= h; d.y += h;
	  a.e(1,2)=0.0;
	  rotate(a,g,h,s,tau,1,3,2,3);
	  rotate(v,g,h,s,tau,1,1,1,2);
	  rotate(v,g,h,s,tau,2,1,2,2);
	  rotate(v,g,h,s,tau,3,1,3,2); 
	  nrot++;
	}
    }
    
    {
      g = 100.0*Math::abs(a.e(1,3));
      if (i>3 && Math::abs(d.x)+g==Math::abs(d.x) && Math::abs(d.z)+g==Math::abs(d.z))
	a.e(1,3)=0.0;
      else if (Math::abs(a.e(1,3))>tresh)
	{
	  h = d.z-d.x;
	  if (Math::abs(h)+g == Math::abs(h)) t=(a.e(1,3))/h;
	  else
	    {
	      theta=0.5*h/(a.e(1,3));
	      t=1.0/(Math::abs(theta)+Math::sqrt(1.0+theta*theta));
	      if (theta < 0.0) t = -t;
	    }
	  c=1.0/Math::sqrt(1+t*t); s=t*c; tau=s/(1.0+c); h=t*a.e(1,3);
	  z.x -= h; z.z += h; d.x -= h; d.z += h;
	  a.e(1,3)=0.0;
	  rotate(a,g,h,s,tau,1,2,2,3);
	  rotate(v,g,h,s,tau,1,1,1,3);
	  rotate(v,g,h,s,tau,2,1,2,3);
	  rotate(v,g,h,s,tau,3,1,3,3); 
	  nrot++;
	}
    }
    
    
    {
      g = 100.0*Math::abs(a.e(2,3));
      if (i>3 && Math::abs(d.y)+g==Math::abs(d.y) && Math::abs(d.z)+g==Math::abs(d.z))
	a.e(2,3)=0.0;
      else if (Math::abs(a.e(2,3))>tresh)
	{
	  h = d.z-d.y;
	  if (Math::abs(h)+g == Math::abs(h)) t=(a.e(2,3))/h;
	  else
	    {
	      theta=0.5*h/(a.e(2,3));
	      t=1.0/(Math::abs(theta)+Math::sqrt(1.0+theta*theta));
	      if (theta < 0.0) t = -t;
	    }
	  c=1.0/Math::sqrt(1+t*t); s=t*c; tau=s/(1.0+c); h=t*a.e(2,3);
	  z.y -= h; z.z += h; d.y -= h; d.z += h;
	  a.e(2,3)=0.0;
	  rotate(a,g,h,s,tau,1,2,1,3); 
	  rotate(v,g,h,s,tau,1,2,1,3); 
	  rotate(v,g,h,s,tau,2,2,2,3);
	  rotate(v,g,h,s,tau,3,2,3,3); 
	  nrot++;
	}
    }
    
    b += z; d = b; z.setZero();
    
  } // for i
  
  return i;
}


void Matrix3::serialize(Serializer& s)
{
  for(Int i=0; i<9; i++) 
    s(m[i]);
}



// output
ostream& base::operator<<(ostream& out, const Matrix3& m)
{
  return out << m.e(1,1) << " " << m.e(1,2) << " " << m.e(1,3) << std::endl
             << m.e(2,1) << " " << m.e(2,2) << " " << m.e(2,3) << std::endl
	     << m.e(3,1) << " " << m.e(3,2) << " " << m.e(3,3) << std::endl;
}







