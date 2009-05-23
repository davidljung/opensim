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
  
  $Id: ODEBallJoint.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.2 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/ODEBallJoint>
#include <physics/ODEConstraintGroup>

using physics::ODEBallJoint;
using physics::ODEConstraintGroup;


ODEBallJoint::ODEBallJoint()
{
}

ODEBallJoint::~ODEBallJoint() 
{
}

void ODEBallJoint::onConstraintGroupAdd(ref<ConstraintGroup> g)
{
  Assert(g != 0);
  Assert(instanceof(*g,ODEConstraintGroup));
  group = g;

  ref<ODEConstraintGroup> ogroup = narrow_ref<ODEConstraintGroup>(group);
  setJointID( dJointCreateBall(ogroup->getWorldID(), ogroup->getJointGroupID()) );
}



void ODEBallJoint::setAnchor(const Point3& p)
{
  checkAddedAndAttached();
  Point3 gp( body1->getRelPointPos(p) ); // to global frame
  dJointSetBallAnchor(jointID, gp.x, gp.y, gp.z);
}

base::Point3 ODEBallJoint::getAnchor() const
{
  checkAddedAndAttached();
  dVector3 ogp;
  dJointGetBallAnchor(jointID, ogp);
  Point3 gp(ogp[0], ogp[1], ogp[2]);
  return body1->getGlobalPointRelPos(gp);
}


// ODEBallJoint doesn't support any Motors
//  Perhaps we could add an ODE AMotor joint to do this? !!!
bool ODEBallJoint::hasMotor(Int dof) const
{
  return false;
}

void ODEBallJoint::setMotorTargetVel(Int dof, Real vel)
{
  Assert(false);
}

void ODEBallJoint::setMotorMaxForce(Int dof, Real force)
{
  Assert(false);
}
