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
  
  $Id: ODEMotor.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.4 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/ODEMotor>
#include <physics/ODEJoint>

using physics::ODEMotor;
using physics::ODEJoint;
using physics::Joint;


ODEMotor::ODEMotor()
{
}

ODEMotor::~ODEMotor()
{
}

void ODEMotor::setTargetVel(Real vel)
{
  checkAttached();
  joint->setMotorTargetVel(dof, vel);
}

void ODEMotor::setMaxForce(Real force)
{
  checkAttached();
  joint->setMotorMaxForce(dof, force);
}

base::ref<Joint> ODEMotor::getJoint()
{
  checkAttached();
  return joint;
}

base::ref<const Joint> ODEMotor::getJoint() const
{
  checkAttached();
  return joint;
}

void ODEMotor::setParameter(const String& name, Real value)
{
  checkAttached();

  Motor::setParameter(name,value);
}


void ODEMotor::setJoint(ref<Joint> joint, Int dof)
{
  if (joint) {
    Assert( instanceof(*joint, ODEJoint));
    this->joint = narrow_ref<ODEJoint>(joint);
    this->dof = dof;
  }
  else { // remove
    this->joint = ref<ODEJoint>(0);
    this->dof=0;
  }
}
