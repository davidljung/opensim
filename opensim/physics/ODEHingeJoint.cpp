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
  
  $Id: ODEHingeJoint.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.4 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/ODEHingeJoint>
#include <physics/ODEConstraintGroup>

using physics::ODEHingeJoint;
using physics::ODEConstraintGroup;


ODEHingeJoint::ODEHingeJoint()
{
}

ODEHingeJoint::~ODEHingeJoint() 
{
}

void ODEHingeJoint::onConstraintGroupAdd(ref<ConstraintGroup> g)
{
  Assert(g != 0);
  Assert(instanceof(*g,ODEConstraintGroup));
  group = g;

  ref<ODEConstraintGroup> ogroup = narrow_ref<ODEConstraintGroup>(group);
  setJointID( dJointCreateHinge(ogroup->getWorldID(), ogroup->getJointGroupID()) );
}

void ODEHingeJoint::setAnchor(const Point3& p)
{
  checkAddedAndAttached();
  Point3 gp( body1->getRelPointPos(p) ); // to global frame
  dJointSetHingeAnchor(jointID, gp.x, gp.y, gp.z);
}

base::Point3 ODEHingeJoint::getAnchor() const
{
  checkAddedAndAttached();
  dVector3 ogp;
  dJointGetHingeAnchor(jointID, ogp);
  Point3 gp(ogp[0], ogp[1], ogp[2]);
  return body1->getGlobalPointRelPos(gp);
}

void ODEHingeJoint::setAxis(const Vector3& v)
{
  checkAddedAndAttached();
  // transform direction vector to global frame for ODE
  Vector3 g(v);
  body1->getOrientation().rotatePoint(g);
  dJointSetHingeAxis(jointID, g.x, g.y, g.z);
}

base::Vector3 ODEHingeJoint::getAxis() const
{
  checkAddedAndAttached();
  dVector3 gaxis;
  dJointGetHingeAxis(jointID, gaxis);
  // transform into body1 frame
  Vector3 l(gaxis[0], gaxis[1], gaxis[2]);
  body1->getOrientation().invert().rotatePoint(l);
  return l;
}
  
Real ODEHingeJoint::getAngle() const
{
  return dJointGetHingeAngle(jointID);
}

Real ODEHingeJoint::getAngleRate() const
{
  return dJointGetHingeAngleRate(jointID);
}

void ODEHingeJoint::setHighStop(Real angle)
{
  checkAddedAndAttached();
  Assertm( -consts::Pi <= angle && angle <= consts::Pi, "angle within range");
  dJointSetHingeParam(jointID, dParamHiStop, angle); 
}

void ODEHingeJoint::setLowStop(Real angle)
{
  checkAddedAndAttached();
  Assertm( -consts::Pi <= angle && angle <= consts::Pi, "angle within range");
  dJointSetHingeParam(jointID, dParamLoStop, angle);
}


void ODEHingeJoint::setStopRestitution(Real r)
{
  checkAddedAndAttached();
  Assertm( 0 <= r && r <= 1, "restitution within range");
  dJointSetHingeParam(jointID, dParamBounce, r);
}


void ODEHingeJoint::setParameter(const String& name, Real value, Int dof)
{
  Assert(dof==1);
  checkAddedAndAttached();
  if (name == "CFM") 
    dJointSetHingeParam(jointID, dParamCFM, value);
  else if (name == "StopERP")
    dJointSetHingeParam(jointID, dParamStopERP, value);
  else if (name == "StopCFM")
    dJointSetHingeParam(jointID, dParamStopCFM, value);
  else if (name == "FudgeFactor")
    dJointSetHingeParam(jointID, dParamFudgeFactor, value);
  else
    HingeJoint::setParameter(name,value, dof); // pass it up to super
}




// protected

bool ODEHingeJoint::hasMotor(Int dof) const
{
  return (dof==1);
}

void ODEHingeJoint::setMotorTargetVel(Int dof, Real vel)
{
  Assert(dof==1);
  dJointSetHingeParam(jointID, dParamVel, vel);
}

void ODEHingeJoint::setMotorMaxForce(Int dof, Real force)
{
  Assert(dof==1);
  dJointSetHingeParam(jointID, dParamFMax, force); 
}

