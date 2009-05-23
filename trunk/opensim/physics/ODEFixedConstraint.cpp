/****************************************************************************
  Copyright (C)2002 David Jung <opensim@pobox.com>

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
  
  $Id: ODEFixedConstraint.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.5 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/ODEFixedConstraint>
#include <physics/ODEConstraintGroup>
#include <physics/ODESolid>

using physics::ODEFixedConstraint;
using physics::FixedConstraint;
using physics::Body;
using physics::ODESolid;


ODEFixedConstraint::ODEFixedConstraint()
{
}

ODEFixedConstraint::~ODEFixedConstraint()
{
}


void ODEFixedConstraint::attach(ref<Body> body1, ref<Body> body2)
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

  dJointAttach(jointID, solid1->getBodyID(), solid2->getBodyID());
  dJointSetHingeAnchor(jointID, 0,0,0);
  dJointSetHingeAxis(jointID, 0,0,1 );
  dJointSetHingeParam(jointID, dParamLoStop, -consts::epsilon);
  dJointSetHingeParam(jointID, dParamHiStop, consts::epsilon);
  dJointSetHingeParam(jointID, dParamBounce, 0);
}

ref<Body> ODEFixedConstraint::getBody(Int index)
{
  Assert(index < 2);
  return (index==0)?body1:body2;
}

ref<const Body> ODEFixedConstraint::getBody(Int index) const
{
  Assert(index < 2);
  return (index==0)?body1:body2;
}


void ODEFixedConstraint::onConstraintGroupAdd(ref<ConstraintGroup> g)
{
  Assert(g != 0);
  Assert(instanceof(*g,ODEConstraintGroup));
  group = g;

  ref<ODEConstraintGroup> ogroup = narrow_ref<ODEConstraintGroup>(group);
  Assert(ogroup->getWorldID());

  // the ODE fixed joint type isn't implemented to maintain a fixed orientation
  //  between the two bodies yet.  So, we hack a fixed joint by using a Hinge joint
  //  and setting its lo-hi stops to the same angle
  setJointID ( dJointCreateHinge(ogroup->getWorldID(), ogroup->getJointGroupID()) );
}

