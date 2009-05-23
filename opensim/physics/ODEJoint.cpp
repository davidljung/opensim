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
  
  $Id: ODEJoint.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.4 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/ODEJoint>
#include <physics/ODEMotor>
#include <physics/ODEConstraintGroup>
#include <physics/ODESolid>

using physics::ODEJoint;
using physics::ODEMotor;
using physics::Joint;
using physics::Motor;
using physics::Body;
using physics::ODESolid;


ODEJoint::ODEJoint()
{
}

ODEJoint::~ODEJoint()
{
}


void ODEJoint::attach(ref<Body> body1, ref<Body> body2)
{
  Assert(body1 != 0);
  Assert(body2 != 0);
  Assertm(body1 != body2, "body1 is not body2");

  if (jointID == 0)
    throw std::runtime_error(Exception("Constraint must be added to a ConstraintGroup before being attached to bodies"));
  
  this->body1 = body1;
  this->body2 = body2;

  ref<ODESolid> solid1 = narrow_ref<ODESolid>(body1);
  ref<ODESolid> solid2 = narrow_ref<ODESolid>(body2);

  dJointAttach(jointID, solid1->bodyID, solid2->bodyID);
}

ref<Body> ODEJoint::getBody(Int index)
{
  Assert(index < 2);
  return (index==0)?body1:body2;
}

ref<const Body> ODEJoint::getBody(Int index) const
{
  Assert(index < 2);
  return (index==0)?body1:body2;
}


void ODEJoint::attachMotor(Int dof, ref<Motor> motor)
{
  if (!hasMotor(dof))
    throw std::invalid_argument(Exception("Invalid motor dof attachment for this Joint"));

  if (motor) {
    
    Assertm( instanceof(*motor, ODEMotor), "Motor created by same SolidSystem as Joint");
    
    ref<ODEMotor> omotor = narrow_ref<ODEMotor>(motor);
    motors[dof] = omotor;
    if (motor != 0) {
      ref<Joint> self(this);
      omotor->setJoint(self, dof);
    }
  } else { // remove
    motors[dof]->setJoint(ref<Joint>(0),dof);
  }

}


ref<Motor> ODEJoint::getMotor(Int dof)
{
  if (!hasMotor(dof))
    throw std::invalid_argument(Exception("Invalid motor dof attachment for this Joint"));

  return motors[dof];
}

ref<const Motor> ODEJoint::getMotor(Int dof) const
{
  if (!hasMotor(dof))
    throw std::invalid_argument(Exception("Invalid motor dof attachment for this Joint"));

  return motors[dof];
}



