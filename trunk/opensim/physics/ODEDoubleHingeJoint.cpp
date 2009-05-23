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
  
  $Id: ODEDoubleHingeJoint.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.4 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/ODEDoubleHingeJoint>
#include <physics/ODEConstraintGroup>

using physics::ODEDoubleHingeJoint;
using physics::ODEConstraintGroup;


ODEDoubleHingeJoint::ODEDoubleHingeJoint()
{
}

ODEDoubleHingeJoint::~ODEDoubleHingeJoint() 
{
}

void ODEDoubleHingeJoint::onConstraintGroupAdd(ref<ConstraintGroup> g)
{
  Assert(g != 0);
  Assert(instanceof(*g,ODEConstraintGroup));
  group = g;

  ref<ODEConstraintGroup> ogroup = narrow_ref<ODEConstraintGroup>(group);
  setJointID( dJointCreateHinge2(ogroup->getWorldID(), ogroup->getJointGroupID()) );
}


void ODEDoubleHingeJoint::setAnchor(const Point3& p)
{
  checkAddedAndAttached();
  Point3 gp( body1->getRelPointPos(p) ); // to global frame
  dJointSetHinge2Anchor(jointID, gp.x, gp.y, gp.z);
}

base::Point3 ODEDoubleHingeJoint::getAnchor() const
{
  checkAddedAndAttached();
  dVector3 ogp;
  dJointGetHinge2Anchor(jointID, ogp);
  Point3 gp(ogp[0], ogp[1], ogp[2]);
  return body1->getGlobalPointRelPos(gp);
}

void ODEDoubleHingeJoint::setAxis1(const Vector3& v)
{
  checkAddedAndAttached();
  // transform direction vector to global frame for ODE
  Vector3 g(v);
  body1->getOrientation().rotatePoint(g);
  dJointSetHinge2Axis1(jointID, g.x, g.y, g.z);
}

base::Vector3 ODEDoubleHingeJoint::getAxis1() const
{
  checkAddedAndAttached();
  dVector3 gaxis;
  dJointGetHinge2Axis1(jointID, gaxis);
  // transform into body1 frame
  Vector3 l(gaxis[0], gaxis[1], gaxis[2]);
  body1->getOrientation().invert().rotatePoint(l);
  return l;
}
  
void ODEDoubleHingeJoint::setAxis2(const Vector3& v)
{
  checkAddedAndAttached();
  // transform direction vector to global frame for ODE
  Vector3 g(v);
  body2->getOrientation().rotatePoint(g);
  dJointSetHinge2Axis2(jointID, g.x, g.y, g.z);
}

base::Vector3 ODEDoubleHingeJoint::getAxis2() const
{
  checkAddedAndAttached();
  dVector3 gaxis;
  dJointGetHinge2Axis2(jointID, gaxis);
  // transform into body1 frame
  Vector3 l(gaxis[0], gaxis[1], gaxis[2]);
  body2->getOrientation().invert().rotatePoint(l);
  return l;
}
  
Real ODEDoubleHingeJoint::getAngle1() const
{
  return dJointGetHinge2Angle1(jointID);
}

Real ODEDoubleHingeJoint::getAngle1Rate() const
{
  return dJointGetHinge2Angle1Rate(jointID);
}

Real ODEDoubleHingeJoint::getAngle2Rate() const
{
  return dJointGetHinge2Angle2Rate(jointID);
}


void ODEDoubleHingeJoint::setHighStop(Int dof, Real angle)
{
  Assert((dof==1)||(dof==2));
  Assertm( -consts::Pi <= angle && angle <= consts::Pi, "angle within range");
  checkAddedAndAttached();
  dJointSetHinge2Param(jointID, (dof==1)?dParamHiStop:dParamHiStop2, angle);
}

void ODEDoubleHingeJoint::setLowStop(Int dof, Real angle)
{
  Assert((dof==1)||(dof==2));
  Assertm( -consts::Pi <= angle && angle <= consts::Pi, "angle within range");
  checkAddedAndAttached();
  dJointSetHinge2Param(jointID, (dof==1)?dParamLoStop:dParamLoStop2, angle);
}


void ODEDoubleHingeJoint::setStopRestitution(Int dof, Real r)
{
  checkAddedAndAttached();
  Assert((dof==1)||(dof==2));
  Assertm( (0 <= r) && (r <= 1), "restitution within range");
  dJointSetHinge2Param(jointID, (dof==1)?dParamBounce:dParamBounce2, r);
}

void ODEDoubleHingeJoint::setParameter(const String& name, Real value, Int dof)
{
  checkAddedAndAttached();
  Assert((dof==1)||(dof==2));
  if (name == "CFM") 
    dJointSetHinge2Param(jointID, dParamCFM, value);
  else if (name == "StopERP")
    dJointSetHinge2Param(jointID, (dof==1)?dParamStopERP:dParamStopERP2, value);
  else if (name == "StopCFM")
    dJointSetHinge2Param(jointID, (dof==1)?dParamStopCFM:dParamStopCFM2, value);
  else if (name == "FudgeFactor")
    dJointSetHinge2Param(jointID, (dof==1)?dParamFudgeFactor:dParamFudgeFactor2, value);
  else
    DoubleHingeJoint::setParameter(name,value, dof); // pass it up to super
}




// protected

bool ODEDoubleHingeJoint::hasMotor(Int dof) const
{
  return ((dof==1)||(dof==2));
}

void ODEDoubleHingeJoint::setMotorTargetVel(Int dof, Real vel)
{
  Assert((dof==1)||(dof==2));
  checkAddedAndAttached();
  if (dof==1)
    dJointSetHinge2Param(jointID, dParamVel, vel);
  else
    dJointSetHinge2Param(jointID, dParamVel2, vel);
}

void ODEDoubleHingeJoint::setMotorMaxForce(Int dof, Real force)
{
  Assert((dof==1)||(dof==2));
  checkAddedAndAttached();
  if (dof==1)
    dJointSetHinge2Param(jointID, dParamFMax, force); 
  else
    dJointSetHinge2Param(jointID, dParamFMax2, force); 
}


