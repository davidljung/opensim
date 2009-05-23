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
  
  $Id: ODEContactConstraint.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.1 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/ODEContactConstraint>
#include <physics/ODEConstraintGroup>
#include <physics/ODESolid>

using physics::ODEContactConstraint;
using physics::ContactConstraint;
using physics::Body;
using physics::ODESolid;


ODEContactConstraint::ODEContactConstraint()
 : addedToGroup(false)
{
}

ODEContactConstraint::~ODEContactConstraint()
{
}


void ODEContactConstraint::attach(ref<Body> body1, ref<Body> body2)
{
  Assert(body1 != 0);
  Assert(body2 != 0);
  Assertm(body1 != body2, "body1 is not body2");
  
  if (!addedToGroup)
    throw std::runtime_error(Exception("Constraint must be added to a ConstraintGroup before being attached to bodies"));
  
  ref<ODEConstraintGroup> ogroup = narrow_ref<ODEConstraintGroup>(group);
  Assert(ogroup->getWorldID());

  this->body1 = body1;
  this->body2 = body2;

  // create an ODE contact joint
  base::clearMemory(&oc,1);
  oc.surface.mode = dContactApprox1 | dContactSoftERP | dContactSoftCFM | dContactBounce;
  //dContactSlip1 | dContactSlip2 ;
  //| dContactSoftERP | dContactSoftCFM; | dContactBounce;
  oc.surface.mu = 8; //!!! arbitrary - should be calc'd from the two materials
  //oc.surface.slip1 = 0.1;
  //oc.surface.slip2 = 0.1;
  oc.surface.soft_erp = 0.5;
  oc.surface.soft_cfm = 1e-5;
  oc.surface.bounce = 0.3;
  //oc.surface.bounce_vel = 0.1;
  
  // transform contact point and normal to global coordinates
  Point3 gpos(body1->getOrientation().rotate(position) + body1->getPosition() );
  oc.geom.pos[0] = gpos.x;
  oc.geom.pos[1] = gpos.y;
  oc.geom.pos[2] = gpos.z;

  Vector3 gnormal(body1->getOrientation().rotate(normal));
  oc.geom.normal[0] = gnormal.x;
  oc.geom.normal[1] = gnormal.y;
  oc.geom.normal[2] = gnormal.z;
//!!!Debugln(Tmp,"gpos=" << gpos << " normal=" << gnormal);

  oc.geom.depth = depth;

  dJointID cj = dJointCreateContact(ogroup->getWorldID(), ogroup->getJointGroupID(), &oc);
  
  setJointID(cj);

  ref<ODESolid> solid1 = narrow_ref<ODESolid>(body1);
  ref<ODESolid> solid2 = narrow_ref<ODESolid>(body2);

  dJointAttach(jointID, solid1->getBodyID(), solid2->getBodyID());

}


ref<Body> ODEContactConstraint::getBody(Int index)
{
  Assert(index < 2);
  return (index==0)?body1:body2;
}

ref<const Body> ODEContactConstraint::getBody(Int index) const
{
  Assert(index < 2);
  return (index==0)?body1:body2;
}


void ODEContactConstraint::onConstraintGroupAdd(ref<ConstraintGroup> g)
{
  Assert(g != 0);
  Assert(instanceof(*g,ODEConstraintGroup));
  group = g;
  
  if (Math::equals(normal.norm(),0) && Math::equals(position.norm(),0))
    throw std::runtime_error(Exception("must set ContactConstraint position,normal & depth before adding it to a ConstraintGroup"));
    
  addedToGroup = true;
    
  // joint will be created on attach()  
}


void ODEContactConstraint::setContactPosition(const Point3& pos)
{
  Assertm(group==0, "contact position set before ContactConstraint added to ConstraintGroup");
  position = pos;
}


void ODEContactConstraint::setContactNormal(const Vector3& normal)
{
  Assertm(group==0, "contact normal set before ContactConstraint added to ConstraintGroup");
  this->normal = normal;
}


void ODEContactConstraint::setContactDepth(Real depth)
{
  Assertm(group==0, "contact depth set before ContactConstraint added to ConstraintGroup");
  this->depth = depth;
}


