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

  $Id: ODECollisionResponseHandler.cpp 1160 2004-09-29 17:51:55Z jungd $

****************************************************************************/

#include <physics/ODECollisionResponseHandler>

#include <physics/CollisionState>
#include <physics/ODEMotor>
#include <physics/ODEConstraintGroup>
#include <physics/ODECollidableBody>

using physics::ODECollisionResponseHandler;

using physics::CollidableBody;
using physics::ODECollidableBody;
using physics::ConstraintGroup;
using physics::ODEConstraintGroup;



ODECollisionResponseHandler::ODECollisionResponseHandler(dWorldID worldID)
  : worldID(worldID)
{
  contactJointGroupID = dJointGroupCreate(0);
}


ODECollisionResponseHandler::~ODECollisionResponseHandler()
{
  dJointGroupDestroy(contactJointGroupID);
}


void ODECollisionResponseHandler::preCollisionTesting()
{
  dJointGroupEmpty(contactJointGroupID); // clear contact constraints from previous collisions
}


void ODECollisionResponseHandler::handleCollision(ref<CollisionState> collisionState)
{
  ref<const Solid> solid1(collisionState->solid1);
  ref<const Solid> solid2(collisionState->solid2);
//!Debugln(Collision,"  **handling**");
  CollisionState::Contacts::const_iterator c(collisionState->contacts.begin());
  CollisionState::Contacts::const_iterator end(collisionState->contacts.end());
  while (c != end) {
    const CollisionState::Contact& contact(*c);

    // setup ODE contact structure
    dContact oc;
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
    Point3 gpos(solid1->getOrientation().rotate(contact.p1) + solid1->getPosition() );
    oc.geom.pos[0] = gpos.x;
    oc.geom.pos[1] = gpos.y;
    oc.geom.pos[2] = gpos.z;

    Vector3 gnormal(solid1->getOrientation().rotate(contact.n1));
    oc.geom.normal[0] = gnormal.x;
    oc.geom.normal[1] = gnormal.y;
    oc.geom.normal[2] = gnormal.z;

    oc.geom.depth = contact.depth;

    // create the contact 'joint' & attach it to the Solids
    dJointID cj = dJointCreateContact(worldID, contactJointGroupID, &oc);

    ref<const ODESolid> solid1(narrow_ref<const ODESolid>(collisionState->solid1));
    ref<const ODESolid> solid2(narrow_ref<const ODESolid>(collisionState->solid2));
    dJointAttach(cj, solid1->bodyID, solid2->bodyID);

    ++c;
  }
}

