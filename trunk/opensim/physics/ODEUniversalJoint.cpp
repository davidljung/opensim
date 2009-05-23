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
  
  $Id: ODEUniversalJoint.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.2 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/ODEUniversalJoint>
#include <physics/ODEConstraintGroup>

using physics::ODEUniversalJoint;
using physics::ODEConstraintGroup;


ODEUniversalJoint::ODEUniversalJoint()
{
}

ODEUniversalJoint::~ODEUniversalJoint() 
{
}

void ODEUniversalJoint::onConstraintGroupAdd(ref<ConstraintGroup> g)
{
  Assert(g != 0);
  Assert(instanceof(*g,ODEConstraintGroup));
  group = g;

  ref<ODEConstraintGroup> ogroup = narrow_ref<ODEConstraintGroup>(group);
  setJointID( dJointCreateUniversal(ogroup->getWorldID(), ogroup->getJointGroupID()) );
}


void ODEUniversalJoint::setAnchor(const Point3& p)
{
  checkAddedAndAttached();
  Point3 gp( body1->getRelPointPos(p) ); // to global frame
  dJointSetUniversalAnchor(jointID, gp.x, gp.y, gp.z);
}

base::Point3 ODEUniversalJoint::getAnchor() const
{
  checkAddedAndAttached();
  dVector3 ogp;
  dJointGetUniversalAnchor(jointID, ogp);
  Point3 gp(ogp[0], ogp[1], ogp[2]);
  return body1->getGlobalPointRelPos(gp);
}

void ODEUniversalJoint::setAxis1(const Vector3& v)
{
  checkAddedAndAttached();
  // transform direction vector to global frame for ODE
  Vector3 g(v);
  body1->getOrientation().rotatePoint(g);
  dJointSetUniversalAxis1(jointID, g.x, g.y, g.z);
}

base::Vector3 ODEUniversalJoint::getAxis1() const
{
  checkAddedAndAttached();
  dVector3 gaxis;
  dJointGetUniversalAxis1(jointID, gaxis);
  // transform into body1 frame
  Vector3 l(gaxis[0], gaxis[1], gaxis[2]);
  body1->getOrientation().invert().rotatePoint(l);
  return l;
}
  
void ODEUniversalJoint::setAxis2(const Vector3& v)
{
  checkAddedAndAttached();
  // transform direction vector to global frame for ODE
  Vector3 g(v);
  body1->getOrientation().rotatePoint(g);
  dJointSetUniversalAxis2(jointID, g.x, g.y, g.z);
}

base::Vector3 ODEUniversalJoint::getAxis2() const
{
  checkAddedAndAttached();
  dVector3 gaxis;
  dJointGetUniversalAxis2(jointID, gaxis);
  // transform into body1 frame
  Vector3 l(gaxis[0], gaxis[1], gaxis[2]);
  body1->getOrientation().invert().rotatePoint(l);
  return l;
}


// Don't think ODE Universal Joint supports a joint motor
// perhaps use an AMotor instead??? !!!
bool ODEUniversalJoint::hasMotor(Int dof) const
{
  return false;
  //return ((dof==1)||(dof==2));
}

void ODEUniversalJoint::setMotorTargetVel(Int dof, Real vel)
{
  Assert(false);
  /*
  Assert((dof==1)||(dof==2));
  if (dof==1)
    dJointSetUniversalParam(jointID, dParamVel, vel);
  else
    dJointSetUniversalParam(jointID, dParamVel2, vel);
  */
}

void ODEUniversalJoint::setMotorMaxForce(Int dof, Real force)
{
  Assert(false);
  /*
  Assert((dof==1)||(dof==2));
  if (dof==1)
    dJointSetUniversalParam(jointID, dParamFMax, force); 
  else
    dJointSetUniversalParam(jointID, dParamFMax2, force); 
  */
}
