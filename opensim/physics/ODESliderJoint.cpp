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
  
  $Id: ODESliderJoint.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.2 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/ODESliderJoint>
#include <physics/ODEConstraintGroup>

using physics::ODESliderJoint;
using physics::ODEConstraintGroup;


ODESliderJoint::ODESliderJoint()
{
}

ODESliderJoint::~ODESliderJoint() 
{
}

void ODESliderJoint::onConstraintGroupAdd(ref<ConstraintGroup> g)
{
  Assert(g != 0);
  Assert(instanceof(*g,ODEConstraintGroup));
  group = g;

  ref<ODEConstraintGroup> ogroup = narrow_ref<ODEConstraintGroup>(group);
  setJointID( dJointCreateSlider(ogroup->getWorldID(), ogroup->getJointGroupID()) );
}



void ODESliderJoint::setAxis(const Vector3& v)
{
  checkAddedAndAttached();
  // transform direction vector to global frame for ODE
  Vector3 g(v);
  body1->getOrientation().rotatePoint(g);
  dJointSetSliderAxis(jointID, g.x, g.y, g.z);
}

base::Vector3 ODESliderJoint::getAxis() const
{
  checkAddedAndAttached();
  dVector3 gaxis;
  dJointGetSliderAxis(jointID, gaxis);
  // transform into body1 frame
  Vector3 l(gaxis[0], gaxis[1], gaxis[2]);
  body1->getOrientation().invert().rotatePoint(l);
  return l;
}

Real ODESliderJoint::getPosition() const
{
  checkAddedAndAttached();
  return dJointGetSliderPosition(jointID);
}

Real ODESliderJoint::getPositionRate() const
{
  checkAddedAndAttached();
  return dJointGetSliderPositionRate(jointID);
}


void ODESliderJoint::setHighStop(Real pos)
{
  checkAddedAndAttached();
  dJointSetSliderParam(jointID, dParamHiStop, pos);
}

void ODESliderJoint::setLowStop(Real pos)
{
  checkAddedAndAttached();
  dJointSetSliderParam(jointID, dParamLoStop, pos);
}

void ODESliderJoint::setStopRestitution(Real r)
{
  checkAddedAndAttached();
  Assertm( 0 <= r && r <= 1, "restitution within range");
  dJointSetSliderParam(jointID, dParamBounce, r);
}


void ODESliderJoint::setParameter(const String& name, Real value, Int dof)
{
  Assert(dof==1);
  if (name == "CFM") 
    dJointSetSliderParam(jointID, dParamCFM, value);
  else if (name == "StopERP")
    dJointSetSliderParam(jointID, dParamStopERP, value);
  else if (name == "StopCFM")
    dJointSetSliderParam(jointID, dParamStopCFM, value);
  else if (name == "FudgeFactor")
    dJointSetSliderParam(jointID, dParamFudgeFactor, value);
  else
    SliderJoint::setParameter(name,value, dof); // pass it up to super
}



// protected

bool ODESliderJoint::hasMotor(Int dof) const
{
  return (dof==1);
}

void ODESliderJoint::setMotorTargetVel(Int dof, Real vel)
{
  Assert(dof==1);
  dJointSetSliderParam(jointID, dParamVel, vel);
}

void ODESliderJoint::setMotorMaxForce(Int dof, Real force)
{
  Assert(dof==1);
  dJointSetSliderParam(jointID, dParamFMax, force); 
}
