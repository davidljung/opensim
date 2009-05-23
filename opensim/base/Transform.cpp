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

  $Id: Transform.cpp 1091 2004-09-13 17:36:59Z jungd $

****************************************************************************/

#include <base/Transform>

using base::Transform;
using base::Matrix4;
using base::Vector3;
using base::Point3;
using base::Quat4;


// constructors
Transform::Transform(const Vector3& translation)
  : isIdentity(translation==Vector3(0,0,0)), isPureTranslationRotation(true),
    hasTranslation(translation!=Vector3(0,0,0)), hasRotation(false),
    trans(translation) {}

Transform::Transform(const Orient& rotation)
  : isIdentity(rotation.equals(Quat4())), isPureTranslationRotation(true),
    hasTranslation(false), hasRotation(!rotation.equals(Quat4())),
    orient(rotation) {}

Transform::Transform(const Quat4& rotation)
  : isIdentity(rotation.equals(Quat4())), isPureTranslationRotation(true),
    hasTranslation(false), hasRotation(!rotation.equals(Quat4())),
    orient(rotation) {}

Transform::Transform(const Matrix3& rotation)
  : isIdentity(false), isPureTranslationRotation(false),
    hasTranslation(false), hasRotation(true),
    t(rotation) {}

Transform::Transform(const Vector3& translation, const Orient& rotation)
  : isIdentity(false), isPureTranslationRotation(true),
    hasTranslation(translation!=Vector3(0,0,0)), hasRotation(!rotation.equals(Quat4())),
    orient(rotation), trans(translation) {}

Transform::Transform(const Vector3& translation, const Quat4& rotation)
  : isIdentity(false), isPureTranslationRotation(true),
    hasTranslation(translation!=Vector3(0,0,0)), hasRotation(!rotation.equals(Quat4())),
    orient(rotation), trans(translation) {}

Transform::Transform(const Vector3& translation, const Matrix3& rotation)
  : isIdentity(false), isPureTranslationRotation(false),
    hasTranslation(true), hasRotation(true),
    t(rotation)
{
  t.setTranslationComponent(translation);
}

Transform::Transform(const Matrix4& t)
  : isIdentity(false), isPureTranslationRotation(false), t(t) {}

Transform::Transform(const Transform& tr)
  : isIdentity(tr.isIdentity), isPureTranslationRotation(tr.isPureTranslationRotation),
    hasTranslation(tr.hasTranslation), hasRotation(tr.hasRotation),
    orient(tr.orient), trans(tr.trans), t(tr.t) {}


Transform& Transform::operator=(const Transform& t)
{
  isIdentity=t.isIdentity;
  isPureTranslationRotation=t.isPureTranslationRotation;
  hasTranslation = t.hasTranslation;
  hasRotation = t.hasRotation;

  if (!isIdentity) {
    if (!isPureTranslationRotation)
      this->t=t.t;
    else {
      orient = t.orient;
      trans = t.trans;
    }
  }
  return *this;
}

//!!! this is broken when one side is a matrix and the other not
bool Transform::operator==(const Transform& rhs) const
{
  if (isIdentity && rhs.isIdentity) return true;

  if (isPureTranslationRotation) {
    if (hasTranslation) {
      if (hasRotation) {
        return (orient == rhs.orient) && (trans == rhs.trans);
      }
      return (trans == rhs.trans);
    }
    return (orient == rhs.orient);
  }
  return t == rhs.t;
}

//!!! this is broken when one side is a matrix and the other not
bool Transform::equals(const Transform& rhs, Real epsilon) const throw()
{
  if (isIdentity && rhs.isIdentity) return true;

  if (isPureTranslationRotation) {
    if (hasTranslation) {
      if (hasRotation) {
        return (orient.equals(rhs.orient,epsilon)) && (trans.equals(rhs.trans,epsilon));
      }
      return (trans.equals(rhs.trans,epsilon));
    }
    return (orient.equals(rhs.orient,epsilon));
  }
  return t.equals(rhs.t,epsilon);
}




Matrix4 Transform::getTransform() const
{
  if (isIdentity)
    return Matrix4();

  if (isPureTranslationRotation) {
    Matrix4 tm;
    if (hasRotation)
      tm = orient.getQuat4();
    if (hasTranslation)
      tm.setTranslationComponent(trans);
    return tm;
  }
  else
    return t;
}


void Transform::setToRotation(const Vector3& axis, Real angle)
{
  isIdentity = false;
  isPureTranslationRotation = true;
  hasTranslation=false;
  hasRotation=true;
  Quat4 q;
  q.setRotation(axis,angle);
  orient = q;
}


void Transform::setToRotation(const Quat4& orient)
{
  isIdentity = false;
  isPureTranslationRotation = true;
  hasTranslation=false;
  hasRotation=true;
  this->orient=orient;
}

void Transform::setToTranslation(const Vector3& trans)
{
  isIdentity = false;
  isPureTranslationRotation = true;
  hasTranslation=true;
  hasRotation=false;
  this->trans=trans;
}


void Transform::setRotationComponent(const Vector3& axis, Real angle)
{
  if (isIdentity || isPureTranslationRotation) {
    hasRotation=true;
    Quat4 q;
    q.setRotation(axis,angle);
    orient = q;
    if (isIdentity)
      hasTranslation=isIdentity=false;
  }
  else {
    Matrix4 r(Quat4(axis,angle));
    r.e(4,4)=0;
    Matrix4 sm(t);
    sm.e(1,1)=sm.e(1,2)=sm.e(1,3)=sm.e(2,1)=sm.e(2,2)=sm.e(2,3)=sm.e(3,1)=sm.e(3,2)=sm.e(3,3)=0;
    sm+=r;
    t=sm;
  }
}


void Transform::setRotationComponent(const Quat4& orient)
{
  if (isIdentity || isPureTranslationRotation) {
    hasRotation=true;
    this->orient=orient;
    if (isIdentity)
      hasTranslation=isIdentity=false;
  }
  else {
    Matrix4 r(orient);
    r.e(4,4)=0;
    Matrix4 sm(t);
    sm.e(1,1)=sm.e(1,2)=sm.e(1,3)=sm.e(2,1)=sm.e(2,2)=sm.e(2,3)=sm.e(3,1)=sm.e(3,2)=sm.e(3,3)=0;
    sm+=r;
    t=sm;
  }
}


void Transform::setRotationComponent(const Orient& orient)
{
  if (isIdentity || isPureTranslationRotation) {
    hasRotation=true;
    this->orient=orient;
    if (isIdentity)
      hasTranslation=isIdentity=false;
  }
  else {
    Matrix4 r(orient);
    r.e(4,4)=0;
    Matrix4 sm(t);
    sm.e(1,1)=sm.e(1,2)=sm.e(1,3)=sm.e(2,1)=sm.e(2,2)=sm.e(2,3)=sm.e(3,1)=sm.e(3,2)=sm.e(3,3)=0;
    sm+=r;
    t=sm;
  }
}


void Transform::setRotationComponent(const Matrix3& rotation)
{
  if (isIdentity || isPureTranslationRotation) {
    hasRotation=true;
    this->orient = rotation;
    if (isIdentity)
      hasTranslation=isIdentity=false;
  }
  else {
    const Matrix3& R(rotation);
    t(1,1)=R(1,1); t(1,2)=R(1,2); t(1,3)=R(1,3);
    t(2,1)=R(2,1); t(2,2)=R(2,2); t(2,3)=R(2,3);
    t(3,1)=R(3,1); t(3,2)=R(3,2); t(3,3)=R(3,3);
  }
}


void Transform::setTranslationComponent(const Vector3& trans)
{
  if (isIdentity || isPureTranslationRotation) {
    hasTranslation=true;
    this->trans=trans;
    if (isIdentity)
      hasRotation=isIdentity=false;
  }
  else {
    t.setTranslationComponent(trans);
  }
}


void Transform::setTransform(const Matrix4& m)
{
  isIdentity=false;
  isPureTranslationRotation=false;
  t=m;
}


Transform& Transform::invert()
{
  if (isIdentity) return *this;
  if (!isPureTranslationRotation)
    t.invert();
  else {
    if (!hasTranslation)
      orient.invert();
    else {
      if (!hasRotation)
        trans = -trans;
      else {
        t = getTransform();
        t.invert();
        isPureTranslationRotation = false;
        hasTranslation = hasRotation = true;
        isIdentity = (t.equals(Matrix4(1)));
      }
    }
  }
  return *this;
}



void Transform::transformPoint(Point3& p) const
{
  if (isIdentity) return;
  if (isPureTranslationRotation) {
    if (hasRotation) orient.rotatePoint(p);
    if (hasTranslation) p+=trans;
  }
  else
    p = t*p;
}

void Transform::transformPoint(Point4& p) const
{
  if (isIdentity) return;
  if (isPureTranslationRotation) {
    if (hasRotation) orient.rotatePoint(p);
    if (hasTranslation) p+=trans;
  }
  else
    p=t*p;
}


base::Point3 Transform::rotate(const Point3& p) const
{
  if (isIdentity) return p;
  if (isPureTranslationRotation) {
    if (hasRotation) return orient.rotate(p);
  }
  else {
    Matrix3 R = t;
    return R*p;
  }
  return p;
}


base::Point3 Transform::translate(const Point3& p) const
{
  if (isIdentity) return p;
  if (isPureTranslationRotation) {
    if (hasTranslation) return p+trans;
  }
  else {
    return Point3(p.x+t(1,4),p.y+t(2,4),p.z+t(3,4)); // check!!!
  }
  return p;
}



Transform& Transform::operator*=(const Transform& rhs)
{
  if (rhs.identity()) return *this;

  if (isIdentity) {
    *this = rhs;
    return *this;
  }

  if (isPureTranslationRotation && rhs.isPureTranslationRotation) {

    if (hasTranslation && !hasRotation) {

      if (!rhs.containsRotation()) {
        trans += rhs.trans;
        return *this;
      }

      Vector3 lhstrans(trans);
      *this = rhs;
      setTranslationComponent(lhstrans + trans);
      return *this;
    }

    if (hasRotation && !hasTranslation && rhs.containsRotation() && !rhs.containsTranslation()) {
      orient = Orient::concatenate(rhs.orient, orient); // note order, lhs*rhs is rhs followed by lhs
      return *this;
    }

  }

  this->t = getTransform(); // convert this to Matrix4 transform
  this->t *= rhs.getTransform();
  isPureTranslationRotation=false;

  return *this;
}


std::ostream& base::operator<<(std::ostream& out, const Transform& t)
{
  if (t.identity()) {
    out << "T:" << Matrix4();
  }
  else {
    if (t.isTransRotationOnly()) {
      out << "r:" << t.getRotation() << " t:" << t.getTranslation();
    }
    else
      out << "T:" << t.getTransform();
  }

  return out;
}
