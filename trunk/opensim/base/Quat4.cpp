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

  $Id: Quat4.cpp 1082 2004-09-13 17:16:21Z jungd $
  $Revision: 1.7 $
  $Date: 2004-09-13 13:16:21 -0400 (Mon, 13 Sep 2004) $
  $Author: jungd $

****************************************************************************/

#include <base/consts>
#include <base/Quat4>
#include <base/Matrix4>
#include <base/Math>
#include <base/Serializer>


using base::Quat4;
using base::Matrix4;
using base::Matrix3;
using base::Vector3;
using base::Point4;
using base::Math;



Quat4& Quat4::operator*=(const Quat4& qr)
{
  Quat4 ql(*this);

  w   = ql.w*qr.w   - ql.v.x*qr.v.x - ql.v.y*qr.v.y - ql.v.z*qr.v.z;
  v.x = ql.w*qr.v.x + ql.v.x*qr.w   + ql.v.y*qr.v.z - ql.v.z*qr.v.y;
  v.y = ql.w*qr.v.y + ql.v.y*qr.w   + ql.v.z*qr.v.x - ql.v.x*qr.v.z;
  v.z = ql.w*qr.v.z + ql.v.z*qr.w   + ql.v.x*qr.v.y - ql.v.y*qr.v.x;

  return *this;
}

void Quat4::setRotation(const Vector3& axis, Real angle)
{
  Vector3 a(axis);
  a.normalize();
  v=a*sin(angle/2.0);
  w=cos(angle/2.0);
}


void Quat4::setRotation(const Matrix4& rotation)
{
  // Algorithm is from the Matrix and Quaternion FAQ
  //  ( currently http://www.cs.ualberta.ca/~andreas/math/matrfaq_latest.html )
  const Matrix4& m = rotation;
  Quat4& q = *this;

  Real tr = 1 + m.e(1,1) + m.e(2,2) + m.e(3,3);
  if (tr > consts::epsilon) {
    Real s = sqrt(tr)*2.0;
    q.v.x = (m.e(3,2) - m.e(2,3))/s;
    q.v.y = (m.e(1,3) - m.e(3,1))/s;
    q.v.z = (m.e(2,1) - m.e(1,2))/s;
    q.w = 0.25*s;
  }
  else {

    if ((m.e(1,1) > m.e(2,2)) && (m.e(1,1) > m.e(3,3))) {
      Real s = sqrt(1+m.e(1,1)-m.e(2,2)-m.e(3,3))*2.0;
      q.v.x = 0.25*s;
      q.v.y = (m.e(2,1)+m.e(1,2))/s;
      q.v.z = (m.e(1,3)+m.e(3,1))/s;
      q.w   = (m.e(3,2)-m.e(2,3))/s;
    }
    else if (m.e(2,2) > m.e(3,3)) {
      Real s = sqrt(1+m.e(2,2)-m.e(1,1)-m.e(3,3))*2.0;
      q.v.x = (m.e(2,1)+m.e(1,2))/s;
      q.v.y = 0.25*s;
      q.v.z = (m.e(3,2)+m.e(2,3))/s;
      q.w   = (m.e(1,3)-m.e(3,1))/s;
    }
    else {
      Real s = sqrt(1+m.e(3,3)-m.e(1,1)-m.e(2,2))*2.0;
      q.v.x = (m.e(1,3)+m.e(3,1))/s;
      q.v.y = (m.e(3,2)+m.e(2,3))/s;
      q.v.z = 0.25*s;
      q.w   = (m.e(2,1)-m.e(1,2))/s;
    }
  }

  //normalize();
}


void Quat4::getRotation(Vector3& axis, Real& angle) const
{
  Quat4 nq(*this); nq.normalize();
  angle = 2.0*acos(nq.w);
  if (!base::equals(angle,0))
    axis = nq.v / sin(angle/2.0);
  else
    axis = Vector3(0,0,1);

  if (base::equals(axis.norm(),0))
    angle = 0;
}


void Quat4::rotatePoint(Point4& p) const
{
  const Quat4& q = *this;
  Quat4 r( (q * Quat4(p)) * inverse(q) );
  p=Point4(r.v.x,r.v.y,r.v.z,r.w);
}


/// Spherical Linear Interpolation
/// As t goes from 0 to 1, the Quat4 object goes from "from" to "to"
/// Reference: Shoemake at SIGGRAPH 89
/// See also
/// http://www.gamasutra.com/features/programming/19980703/quaternions_01.htm
Quat4 Quat4::interpolate(const Quat4& from, const Quat4& to, Real t)
{
  const Real linearTolerance = 1e-4;

  Real c = base::dot(from, to);

  // Because of potential rounding errors, we must clamp c to the domain of acos.
  Math::bound(c, -1.0, 1.0);

  Real angle = acos(c);

  if (Math::abs(angle) < linearTolerance)
    return from;

  Real s = sin(angle);
  Real is = 1.0/s;

  return (sin((1-t)*angle)*is)*from + (sin(t*angle)*is) * to;
}


Real Quat4::angleBetween(const Quat4& q1, const Quat4& q2)
{
  // we calculate the angle a round-about way - there
  //  is probably a better one
  // transform a unit x-axis vector by each Quat and then use the dot product to get
  // and angle between them
  Vector3 xaxis(Vector3(1,0,0));
  Vector3 vector1(q1.rotate(xaxis));
  Vector3 vector2(q2.rotate(xaxis));
  return Math::acos(base::dot(vector1,vector2));
}


Matrix3 Quat4::E() const
{
  Matrix3 S = base::toMatrix3(Math::S(base::fromVector3(v)));
  for(Int i=1; i<=3; i++) S(i,i) = w - S(i,i);
  return S;
}


Quat4::operator Matrix4() const
{
  Real nq = norm();
  Real s = (nq> Real(0))? (Real(2)/base::sqrt(nq)):Real(0);

  Real xs = v.x*s, ys = v.y*s, zs = v.z*s;
  Real wx = w*xs,  wy = w*ys,  wz = w*zs;
  Real xx = v.x*xs,xy = v.x*ys,xz = v.x*zs;
  Real yy = v.y*ys,yz = v.y*zs,zz = v.z*zs;

  Matrix4 m; // identity

  m.e(1,1) = Real(1) - (yy+zz);  m.e(2,1) = xy+wz;             m.e(3,1) = xz-wy;
  m.e(1,2) = xy - wz;            m.e(2,2) = Real(1) - (xx+zz); m.e(3,2) = yz+wx;
  m.e(1,3) = xz+wy;              m.e(2,3) = yz-wx;             m.e(3,3) = Real(1) - (xx+yy);

  return m;
}


void Quat4::serialize(Serializer& s)
{
  s(v,"v"); s(w,"w");
}



// output
std::ostream& base::operator<<(std::ostream& out, const Quat4& q)
{
//output as axis/angle
Vector3 axis; Real angle;
q.getRotation(axis,angle);
return out << "(axis:" << axis << ", angle:" << angle << "[" << base::Math::radToDeg(angle) << "deg])";

  return out << "([" << q.v.x << "," << q.v.y << "," << q.v.z << "]," << q.w << ")";
}
