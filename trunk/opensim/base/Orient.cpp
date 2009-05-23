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

  $Id: Orient.cpp 1089 2004-09-13 17:34:40Z jungd $
  $Revision: 1.21 $
  $Date: 2004-09-13 13:34:40 -0400 (Mon, 13 Sep 2004) $
  $Author: jungd $

****************************************************************************/

#include <base/Orient>

using base::Orient;

#include <base/Math>
#include <base/Serializer>


using base::Byte;
using base::Math;
using base::Matrix3;
using base::Matrix4;
using base::Vector3;
using base::Point4;
using base::Quat4;
using base::Vector;
using base::Matrix;

Byte Orient::EulSafe[4] = { 0,1,2,0 };
Byte Orient::EulNext[4] = { 1,2,0,1 };

const Orient::Representation Orient::EulerXYZs = eulerRep(X,Even, NonRepeating,Static);
const Orient::Representation Orient::EulerXYXs = eulerRep(X,Even,    Repeating,Static);
const Orient::Representation Orient::EulerXZYs = eulerRep(X,Odd , NonRepeating,Static);
const Orient::Representation Orient::EulerXZXs = eulerRep(X,Odd ,    Repeating,Static);
const Orient::Representation Orient::EulerYZXs = eulerRep(Y,Even, NonRepeating,Static);
const Orient::Representation Orient::EulerYZYs = eulerRep(Y,Even,    Repeating,Static);
const Orient::Representation Orient::EulerYXZs = eulerRep(Y,Odd , NonRepeating,Static);
const Orient::Representation Orient::EulerYXYs = eulerRep(Y,Odd ,    Repeating,Static);
const Orient::Representation Orient::EulerZXYs = eulerRep(Z,Even, NonRepeating,Static);
const Orient::Representation Orient::EulerZXZs = eulerRep(Z,Even,    Repeating,Static);
const Orient::Representation Orient::EulerZYXs = eulerRep(Z,Odd , NonRepeating,Static);
const Orient::Representation Orient::EulerZYZs = eulerRep(Z,Odd ,    Repeating,Static);

const Orient::Representation Orient::EulerZYXr = eulerRep(X,Even, NonRepeating,Rotating);
const Orient::Representation Orient::EulerXYXr = eulerRep(X,Even,    Repeating,Rotating);
const Orient::Representation Orient::EulerYZXr = eulerRep(X,Odd , NonRepeating,Rotating);
const Orient::Representation Orient::EulerXZXr = eulerRep(X,Odd ,    Repeating,Rotating);
const Orient::Representation Orient::EulerXZYr = eulerRep(Y,Even, NonRepeating,Rotating);
const Orient::Representation Orient::EulerYZYr = eulerRep(Y,Even,    Repeating,Rotating);
const Orient::Representation Orient::EulerZXYr = eulerRep(Y,Odd , NonRepeating,Rotating);
const Orient::Representation Orient::EulerYXYr = eulerRep(Y,Odd ,    Repeating,Rotating);
const Orient::Representation Orient::EulerYXZr = eulerRep(Z,Even, NonRepeating,Rotating);
const Orient::Representation Orient::EulerZXZr = eulerRep(Z,Even,    Repeating,Rotating);
const Orient::Representation Orient::EulerXYZr = eulerRep(Z,Odd , NonRepeating,Rotating);
const Orient::Representation Orient::EulerZYZr = eulerRep(Z,Odd ,    Repeating,Rotating);

const Orient::Representation Orient::EulerRPY  = eulerRep(X,Even, NonRepeating,Static); // XYZs





Orient::Orient()
  : rep(EulerRPY), v(zeroVector(3))
{
}


Orient::Orient(Representation representation)
  : rep(representation), v(zeroVector(3))
{
  Assert(rep < RepEnd);
  if (rep == Mat) {
    m = MatrixRef(NewObj Matrix(3,3));
    *m = identityMatrix(3,3);
  }
  else {
    if (rep == Quat) {
      Quat4 q;
      v.resize(4);
      v[0]=q.v.x; v[1]=q.v.y; v[2]=q.v.z; v[3]=q.w;
    }
  }
}


Orient::Orient(const Quat4& q)
 : rep(Quat), v(4)
{
  v[0]=q.v.x; v[1]=q.v.y; v[2]=q.v.z; v[3]=q.w;
}


Orient::Orient(Real roll, Real pitch, Real yaw)
 : rep(EulerRPY), v(3)
{
  v[0]=roll; v[1]=pitch; v[2]=yaw;
}



Orient::Orient(const Vector& v, Representation representation)
  : rep(representation)
{
  Assert(rep < RepEnd);
  Assertm( (rep==Quat)?(v.size()==4):true, "Quaternion has 4 components");
  if (rep==Mat) {
    Assertm( v.size() == 9, "Rotation matrix has 9 components");
    m = MatrixRef(NewObj Matrix(3,3));
    (*m)(0,0) = v[0]; (*m)(0,1) = v[1]; (*m)(0,2) = v[2];
    (*m)(1,0) = v[3]; (*m)(1,1) = v[4]; (*m)(1,2) = v[5];
    (*m)(2,0) = v[6]; (*m)(2,1) = v[7]; (*m)(2,2) = v[8];
  }
  else {
    if (rep!=Quat) {
      Assertm( v.size()==3, "vector should have 3 components");
      this->v.reset(v);
    }
    else { // normalize Quat
      Quat4 q(v[0],v[1],v[2],v[3]);
      q.normalize();
      this->v.resize(4);
      this->v[0]=q.v.x; this->v[1]=q.v.y; this->v[2]=q.v.z; this->v[3]=q.w;
    }
  }
}


Orient::Orient(const Matrix3& m3)
 : rep(Mat), m(NewObj Matrix(3,3))
{
  (*m)(0,0) = m3(1,1);
  (*m)(0,1) = m3(1,2);
  (*m)(0,2) = m3(1,3);
  (*m)(1,0) = m3(2,1);
  (*m)(1,1) = m3(2,2);
  (*m)(1,2) = m3(2,3);
  (*m)(2,0) = m3(3,1);
  (*m)(2,1) = m3(3,2);
  (*m)(2,2) = m3(3,3);
}


Orient::Orient(const Matrix& m)
  : rep(Mat), m(NewObj Matrix(3,3))
{
  Assert(m.size1()==3);
  Assert(m.size2()==3);
  *(this->m) = m;
}


Orient::Orient(const Orient& copy)
 : rep(copy.rep)
{
  if (rep!=Mat) {
    v.resize(copy.v.size());
    v = copy.v;
  }
  else {
    m = MatrixRef(NewObj Matrix(*copy.m));
    *m = *copy.m;
  }
}


Orient::~Orient()
{
  if (rep==Mat)
    delete &(*m);
}


Orient& Orient::operator=(const Matrix3& mt)
{
  if (rep != Mat)
    m = MatrixRef(NewObj Matrix(3,3));
  rep = Mat;
  (*m)(0,0) = mt(1,1); (*m)(0,1) = mt(1,2);  (*m)(0,2) = mt(1,3);
  (*m)(1,0) = mt(2,1); (*m)(1,1) = mt(2,2);  (*m)(1,2) = mt(2,3);
  (*m)(2,0) = mt(3,1); (*m)(2,1) = mt(3,2);  (*m)(2,2) = mt(3,3);
  return *this;
}




base::String Orient::representationString(Representation rep)
{
  if (rep == Quat) return String("Quat");
  else if (rep == Mat) return String("Matrix");
  else if (rep == Rodriguez) return String("Rodriguez");
  else if (rep == EulerXYZs) return String("EulerXYZs");
  else if (rep == EulerXYXs) return String("EulerXYXs");
  else if (rep == EulerXZYs) return String("EulerXZYs");
  else if (rep == EulerXZXs) return String("EulerXZXs");
  else if (rep == EulerYZXs) return String("EulerYZXs");
  else if (rep == EulerYZYs) return String("EulerYZYs");
  else if (rep == EulerYXZs) return String("EulerYXZs");
  else if (rep == EulerYXYs) return String("EulerYXYs");
  else if (rep == EulerZXYs) return String("EulerZXYs");
  else if (rep == EulerZXZs) return String("EulerZXZs");
  else if (rep == EulerZYXs) return String("EulerZYXs");
  else if (rep == EulerZYZs) return String("EulerZYZs");
  else if (rep == EulerZYXr) return String("EulerZYXr");
  else if (rep == EulerXYXr) return String("EulerXYXr");
  else if (rep == EulerYZXr) return String("EulerYZXr");
  else if (rep == EulerXZXr) return String("EulerXZXr");
  else if (rep == EulerXZYr) return String("EulerXZYr");
  else if (rep == EulerYZYr) return String("EulerYZYr");
  else if (rep == EulerZXYr) return String("EulerZXYr");
  else if (rep == EulerYXYr) return String("EulerYXYr");
  else if (rep == EulerYXZr) return String("EulerYXZr");
  else if (rep == EulerZXZr) return String("EulerZXZr");
  else if (rep == EulerXYZr) return String("EulerXYZr");
  else if (rep == EulerZYZr) return String("EulerZYZr");
  else throw std::invalid_argument(Exception("unknown representation"));
}





Matrix Orient::getRotationMatrix() const
{
  Orient r(*this); r.changeRep(Mat);
  return Matrix(*r.m);
}


Matrix3 Orient::getRotationMatrix3() const
{
  Orient r(*this); r.changeRep(Mat);
  Matrix& m(*r.m);
  Matrix3 m3(m(0,0), m(0,1), m(0,2),
             m(1,0), m(1,1), m(1,2),
             m(2,0), m(2,1), m(2,2));
  return m3;
}

Quat4 Orient::getQuat4() const
{
  if (rep == Quat)
    return Quat4(v[0],v[1],v[2],v[3]);
  else {
    Orient r(*this);
    r.changeRep(Quat);
    return Quat4(r.v[0],r.v[1],r.v[2],r.v[3]);
  }
}

Vector Orient::getVector(Representation representation) const
{
  Orient c;
  const Orient* r(this);
  if (rep != representation) {
    c = *this;
    c.changeRep(representation);
    r = &c;
  }

  if (representation!=Mat) {
    return r->v;
  }
  else {
    Vector v(9);
    const Matrix& m(*(r->m));
    v[0] = m(0,0); v[1] = m(0,1); v[2] = m(0,2);
    v[3] = m(1,0); v[4] = m(1,1); v[5] = m(1,2);
    v[6] = m(2,0); v[7] = m(2,1); v[8] = m(2,2);
    return v;
   }
}


Vector3 Orient::getVector3(Representation representation) const
{
  Vector v( getVector(representation) );
  return Vector3(v[0],v[1],v[2]);
}



Orient& Orient::operator=(const Orient& copy) throw()
{
  if (&copy != this) {
    Assert(copy.rep < RepEnd);
    if (rep!=Mat) {
      if (copy.rep!=Mat) {
        rep=copy.rep;
        v.reset(copy.v);
      }
      else {
        rep=copy.rep;
        m = MatrixRef(NewObj Matrix(*copy.m));
      }
    }
    else {
      if (copy.rep!=Mat) {
        delete &(*m);
        rep=copy.rep;
        v.reset(copy.v);
      }
      else {
        *m = *copy.m;
      }
    }
  }
  return *this;
}


void Orient::setIdentity(Orient::Representation representation)
{
  Assert(representation < RepEnd)
  if (representation == Quat) {
    if (rep == Mat)
      delete &(*m);
    rep = Quat;
    Quat4 q; // zero Quat
    v.resize(4);
    v[0]=q.v.x; v[1]=q.v.y; v[2]=q.v.z; v[3]=q.w;
  }
  else if (representation == Mat) {
    if (rep != Mat)
      m = MatrixRef(NewObj Matrix(3,3));
    rep = Mat;
    (*m)(0,0) = 1; (*m)(0,1) = 0;  (*m)(0,2) = 0; // identity
    (*m)(1,0) = 0; (*m)(1,1) = 1;  (*m)(1,2) = 0;
    (*m)(2,0) = 0; (*m)(2,1) = 0;  (*m)(2,2) = 1;
  }
  else {
    rep = representation;
    v.resize(3);
    v[0]=v[1]=v[2]=0;
  }

}


void Orient::setFromRotationComponent(const Matrix4& mt)
{
  if (rep != Mat) {
    m = MatrixRef(NewObj Matrix(3,3));
    rep = Mat;
  }
  (*m)(0,0) = mt(1,1); (*m)(0,1) = mt(1,2);  (*m)(0,2) = mt(1,3);
  (*m)(1,0) = mt(2,1); (*m)(1,1) = mt(2,2);  (*m)(1,2) = mt(2,3);
  (*m)(2,0) = mt(3,1); (*m)(2,1) = mt(3,2);  (*m)(2,2) = mt(3,3);
}


/// \todo consider trucky cases, like the fact that quats can have two representation for a single orientation (perhaps even covert all the reps to quats for comparison?)
///       same with matrices etc
bool Orient::operator==(const Orient& o) const throw()
{
  if (&o == this) return true;

  Assert(o.rep < RepEnd);

  if (rep == o.rep) { // fast path
    if (rep!=Mat) {
      if (v.size() == o.v.size())
        return v == o.v;
      else
        return false;
    }
    return *m == *o.m;
  }

  // try converting each to the other first
  Orient tthis(*this);
  try {
    tthis.changeRep(o.rep);
    if (o.rep!=Mat)
      return tthis.v == o.v;
    return *tthis.m == *o.m;
  } catch (std::exception&) {}
  Orient to(o);
  try {
    to.changeRep(rep);
    if (rep!=Mat)
      return v == to.v;
    return *m == *to.m;
  } catch (std::exception&) {
    return false;
  }
}


/// \todo consider tricky cases, like the fact that quats can have two representation for a single orientation (perhaps even covert all the reps to quats for comparison?)
///       same with matrices etc
bool Orient::equals(const Orient& o, Real epsilon) const throw()
{
  if (&o == this) return true;

  Assert(o.rep < RepEnd);

  if (rep == o.rep) { // fast path
    if (rep!=Mat) {
      if (v.size() == o.v.size())
        return base::equals(v, o.v, epsilon);
      else
        return false;
    }
    return base::equals(*m, *o.m, epsilon);
  }

  // try converting each to the other first
  Orient tthis(*this);
  try {
    tthis.changeRep(o.rep);
    if (o.rep!=Mat)
      return base::equals(tthis.v, o.v, epsilon);
    return base::equals(*(tthis.m), *o.m, epsilon);
  } catch (std::exception&) {}
  Orient to(o);
  try {
    to.changeRep(rep);
    if (rep!=Mat)
      return base::equals(v, to.v, epsilon);
    return base::equals(*m, *to.m, epsilon);
  } catch (std::exception&) {
    return false;
  }

}



void Orient::changeRep(Representation newRep) const
{
  Assert(rep < RepEnd);
  Assert(newRep < RepEnd);

  if (rep == newRep) return; // no conversion necessary


  // convert from EulerIJKf
  if (isEuler()) {

    if (newRep == Quat) { // to Quat

      Quat4 qu;
      Real a[3], ti, tj, th, ci, cj, ch, si, sj, sh, cc, cs, sc, ss;
      Axis i,j,k,h;
      Parity n;
      Repetition s;
      Frame f;
      getRep(rep,i,j,k,h,n,s,f);
      if (f==Rotating) {Real t = v[X]; v[X] = v[Z]; v[Z] = t;}
      if (n==Odd) v[Y] = -v[Y];
      ti = v[X]*0.5; tj = v[Y]*0.5; th = v[Z]*0.5;
      ci = Math::cos(ti);  cj = Math::cos(tj);  ch = Math::cos(th);
      si = Math::sin(ti);  sj = Math::sin(tj);  sh = Math::sin(th);
      cc = ci*ch; cs = ci*sh; sc = si*ch; ss = si*sh;
      if (s==Repeating) {
          a[i] = cj*(cs + sc);	// Could speed up with
          a[j] = sj*(cc + ss);	// trig identities.
          a[k] = sj*(cs - sc);
          qu.w = cj*(cc - ss);
      } else {
          a[i] = cj*sc - sj*cs;
          a[j] = cj*ss + sj*cc;
          a[k] = cj*cs - sj*sc;
          qu.w = cj*cc + sj*ss;
      }
      if (n==Odd) a[j] = -a[j];
      qu.v.x = a[X]; qu.v.y = a[Y]; qu.v.z = a[Z];
      v.resize(4);
      v[X]=qu.v.x; v[Y]=qu.v.y; v[Z]=qu.v.z; v[W]=qu.w;
      rep=Quat;
      return;

    }
    else if (newRep == Mat) { // to Mat

      m = MatrixRef(NewObj Matrix(3,3));
      Matrix& M(*this->m);

      Real ti, tj, th, ci, cj, ch, si, sj, sh, cc, cs, sc, ss;
      Axis i,j,k,h;
      Parity n;
      Repetition s;
      Frame f;
      getRep(rep,i,j,k,h,n,s,f);
      if (f==Rotating) {Real t = v[X]; v[X] = v[Z]; v[Z] = t;}
      if (n==Odd) { v[X] = -v[X]; v[Y] = -v[Y]; v[Z] = -v[Z];}
      ti = v[X]; tj = v[Y]; th = v[Z];
      ci = Math::cos(ti); cj = Math::cos(tj); ch = Math::cos(th);
      si = Math::sin(ti); sj = Math::sin(tj); sh = Math::sin(th);
      cc = ci*ch; cs = ci*sh; sc = si*ch; ss = si*sh;
      if (s==Repeating) {
          M(i,i) = cj;	   M(i,j) =  sj*si;    M(i,k) =  sj*ci;
          M(j,i) = sj*sh;  M(j,j) = -cj*ss+cc; M(j,k) = -cj*cs-sc;
          M(k,i) = -sj*ch; M(k,j) =  cj*sc+cs; M(k,k) =  cj*cc-ss;
      } else {
          M(i,i) = cj*ch; M(i,j) = sj*sc-cs; M(i,k) = sj*cc+ss;
          M(j,i) = cj*sh; M(j,j) = sj*ss+cc; M(j,k) = sj*cs-sc;
          M(k,i) = -sj;	  M(k,j) = cj*si;    M(k,k) = cj*ci;
      }

      rep=Mat;
      return;
    }
    else {
      // try to convert whatever to a Quat first
      changeRep(Quat);
      changeRep(newRep);
      return;
    }

  }
  else if (rep == Quat) { // convert from Quat

    if (isEuler(newRep)) { // to EulerIJKf

      changeRep(Mat); // via Mat
      changeRep(newRep);
      return;

    }
    else if (newRep == Mat) { // to Mat

      Real nq = v[0]*v[0]+v[1]*v[1]+v[2]*v[2]+v[3]*v[3];
      //!!! NB: the Graphics Gems impl. is identical except for not taking the srqt of nq opn the next line(!?)
      Real s = (nq> Real(0))? (Real(2)/base::sqrt(nq)):Real(0);

      Real xs = v[0]*s, ys = v[1]*s, zs = v[2]*s;
      Real wx = v[3]*xs,wy = v[3]*ys,wz = v[3]*zs;
      Real xx = v[0]*xs,xy = v[0]*ys,xz = v[0]*zs;
      Real yy = v[1]*ys,yz = v[1]*zs,zz = v[2]*zs;

      m = MatrixRef(NewObj Matrix(3,3));

      (*m)(0,0) = Real(1) - (yy+zz);
      (*m)(1,0) = xy+wz;
      (*m)(2,0) = xz-wy;
      (*m)(0,1) = xy - wz;
      (*m)(1,1) = Real(1) - (xx+zz);
      (*m)(2,1) = yz+wx;
      (*m)(0,2) = xz+wy;
      (*m)(1,2) = yz-wx;
      (*m)(2,2) = Real(1) - (xx+yy);
      rep = Mat;
      return;
    }
    else if (newRep == Quat)
    {
      return; // nothing to do
    }

  }
  else if (rep == Mat) { // convert from Mat

    if (isEuler(newRep)) { // to EulerIJKf

      const Matrix& M(*this->m);
      v.resize(3);
      Axis i,j,k,h;
      Parity n;
      Repetition s;
      Frame f;
      getRep(newRep,i,j,k,h,n,s,f);
      if (s==Repeating) {
          Real sy = Math::sqrt(M(i,j)*M(i,j) + M(i,k)*M(i,k));
          if (sy > 16*consts::minReal) {
              v[X] = Math::atan2(M(i,j), M(i,k));
              v[Y] = Math::atan2(sy, M(i,i));
              v[Z] = Math::atan2(M(j,i), -M(k,i));
          } else {
              v[X] = Math::atan2(-M(j,k), M(j,j));
              v[Y] = Math::atan2(sy, M(i,i));
              v[Z] = 0;
          }
      } else {
          Real cy = Math::sqrt(M(i,i)*M(i,i) + M(j,i)*M(j,i));
          if (cy > 16*consts::minReal) {
              v[X] = Math::atan2(M(k,j), M(k,k));
              v[Y] = Math::atan2(-M(k,i), cy);
              v[Z] = Math::atan2(M(j,i), M(i,i));
          } else {
              v[X] = Math::atan2(-M(j,k), M(j,j));
              v[Y] = Math::atan2(-M(k,i), cy);
              v[Z] = 0;
          }
      }
      if (n==Odd) {v[X] = -v[X]; v[Y] = - v[Y]; v[Z] = -v[Z];}
      if (f==Rotating) {Real t = v[X]; v[X] = v[Z]; v[Z] = t;}

      rep=newRep;
      return;

    }
    else if (newRep == Quat) { // to Quat

      // Algorithm is from the Matrix and Quaternion FAQ
      //  ( currently http://www.cs.ualberta.ca/~andreas/math/matrfaq_latest.html )
      const Matrix& m(*this->m);
      v.resize(4);

      Real tr = 1 + m(0,0) + m(1,1) + m(2,2);
      if (tr > consts::epsilon) {
        Real s = sqrt(tr)*2.0;
        v[0] = (m(2,1) - m(1,2))/s;
        v[1] = (m(0,2) - m(2,0))/s;
        v[2] = (m(1,0) - m(0,1))/s;
        v[3] = 0.25*s;
      }
      else {

        if ((m(0,0) > m(1,1)) && (m(0,0) > m(2,2))) {
          Real s = sqrt(1+m(0,0)-m(1,1)-m(2,2))*2.0;
          v[0] = 0.25*s;
          v[1] = (m(1,0)+m(0,1))/s;
          v[2] = (m(0,2)+m(2,0))/s;
          v[3] = (m(2,1)-m(1,2))/s;
        }
        else if (m(1,1) > m(2,2)) {
          Real s = sqrt(1+m(1,1)-m(0,0)-m(2,2))*2.0;
          v[0] = (m(1,0)+m(0,1))/s;
          v[1] = 0.25*s;
          v[2] = (m(2,1)+m(1,2))/s;
          v[3] = (m(0,2)-m(2,0))/s;
        }
        else {
          Real s = sqrt(1+m(2,2)-m(0,0)-m(1,1))*2.0;
          v[0] = (m(0,2)+m(2,0))/s;
          v[1] = (m(2,1)+m(1,2))/s;
          v[2] = 0.25*s;
          v[3] = (m(1,0)-m(0,1))/s;
        }
      }

      rep=Quat;
      return;
    }
    else if (newRep == Mat) {
      return; // nothing to do
    }


  }

  throw std::runtime_error(Exception("conversion from "+representationString(rep)+" to "+representationString(newRep)+" unsupported"));

}





Matrix Orient::getBinv() const
{
  Matrix Binv;

  if (rep == EulerRPY) {

    // implementation taken from IKORv2 Euler_to_Velocities() function
    Binv.resize(3,3);

    //const Real& alpha(v[0]);
    const Real& beta(v[1]);
    const Real& gamma(v[2]);
    Real cb = Math::cos(beta);
    Real cg = Math::cos(gamma);
    Real sb = Math::sin(beta);
    Real sg = Math::sin(gamma);

    Binv(0,0) = cg*cb; Binv(0,1) = -sg; Binv(0,2) = 0;
    Binv(1,0) = sg*cb; Binv(1,1) = cg;  Binv(1,2) = 0;
    Binv(2,0) = -sb;   Binv(2,1) = 0;   Binv(2,2) = 1;
  }
  else if (rep == Quat) {

    Binv.resize(3,4);

    Quat4 q(v[0], v[1], v[2], v[3]); // scalar comp. last
    q.normalize();
    const Real& qx(q.v.x); // save typing
    const Real& qy(q.v.y);
    const Real& qz(q.v.z);
    const Real& qw(q.w);

    // The equation dq = 0.5w.q (where w.q is quat mult of [0,w] with q), can be re-arranged
    //  to give w=2dq.q* where q* is quat conjugate of q and |q| is assumed 1.
    // Expanded into vector form using quat mult defition, then collecting terms of dq to
    //  give matrix form w=Binv.dq, where Binv is matrix below.
    // (derivation done in Scientific WorkPlace via Maple)
    Binv(0,0) = +qw; Binv(0,1) = -qz; Binv(0,2) = +qy; Binv(0,3) = -qx;
    Binv(1,0) = +qz; Binv(1,1) = +qw; Binv(1,2) = -qx; Binv(1,3) = -qy;
    Binv(2,0) = -qy; Binv(2,1) = +qx; Binv(2,2) = +qw; Binv(2,3) = -qz;
    Binv = 2.0*Binv;

//try
//Binv(0,0) = qw*qw+qx*qx; Binv(0,1) = qz*qw+qy*qx; Binv(0,2) = qz*qx-qy*qw; Binv(0,3) = 0;
//Binv(1,0) = -qz*qw+qy*qx;Binv(1,1) = qw*qw+qy*qy; Binv(1,2) = qw*qx+qz*qy; Binv(1,3) = 0;
//Binv(2,0) = qz*qx+qy*qw; Binv(2,1) = qw*qw+qy*qy; Binv(2,2) = qw*qw+qz*qz; Binv(2,3) = 0;
//Binv = Binv/qw;
//Binv = 2.0*Binv;
// nope - doesn't work.  why isn't Binv(i,3) != 0 ?? (means dq.w isn;t being used??!!)

  }
  else {
    throw std::runtime_error(Exception("Binv generation from current orientation representation unsupported"));
  }

  return Binv;
}



Orient Orient::interpolate(const Orient& lower, const Orient& upper, Real t)
{
  Math::bound(t, 0.0, 1.0);

  //if (t == 0.0) return lower; // since q & -q are identical, lower might now be the same as inter with t=0
  //if (t == 1.0) return upper;

  // we convert both orientations to Quats and use the Quat4::interpolate() method
  //  which uses SLERP (Spherical Linear intERPolation)
  Quat4 from = lower.getQuat4(); // convert to Quat4
  Quat4 to = upper.getQuat4();
  //if (from.w<0) from.negate(); // identify -q & q  !!! needed?
  //if (to.w<0) to.negate();

  Quat4 interp(Quat4::interpolate(from,to,t)); // SLERP

  return Orient(interp);
}



Orient Orient::concatenate(const Orient& r1, const Orient& r2)
{
  Orient cr2(r2);
  if (r2.representation() != r1.representation())
    cr2.changeRepresentation(r1.representation());

  switch (r1.representation()) {

    case Orient::Quat: {
      return Orient( cr2.getQuat4() * r1.getQuat4() );
    } break;

    case Orient::Mat: {
      return Orient( cr2.getRotationMatrix() * r1.getRotationMatrix() );
    } break;

    default:
      throw std::runtime_error(Exception("concatenate not implemented for representation "+r1.representation()));
  }
}





Orient& Orient::invert()
{
  if (rep == Mat) {
    Matrix3 m3( base::toMatrix3(*m) );
    m3.invert();
    *m = base::fromMatrix3(m3);
  }
  else {
    Quat4 q( getQuat4() );
    q.invert();
    *this = q;
  }
  return *this;
}




void Orient::serialize(Serializer& s)
{
  Representation oldrep(rep);
  s(rep,"rep");

  if (s.isOutput()) {
    if (rep!=Mat)
      s(v,"v");
    else
      s(*m,"m");
  }
  else {
    if (rep!=Mat) {
      if (oldrep==Mat) {
        delete &(*m);
        m=MatrixRef(0);
      }
      s(v,"v");
    }
    else {
      if (oldrep!=Mat)
        m = MatrixRef(NewObj Matrix(3,3));
      s(*m,"m");
    }
  }
  Assert(rep < RepEnd);
}




std::ostream& base::operator<<(std::ostream& out, const Orient& o) // Output
{
  return (o.rep==Orient::Mat)?(out << *o.m):( (o.rep==Orient::Quat)?(out << o.getQuat4()):(out << o.v) );
}

