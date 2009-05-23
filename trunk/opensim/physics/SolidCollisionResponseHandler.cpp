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
  
  $Id: SolidCollisionResponseHandler.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.3 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/SolidCollisionResponseHandler>

#include <physics/CollisionState>
#include <physics/CollidableBody>
#include <physics/SolidConnectedCollidableBody>
#include <physics/ContactConstraint>

using physics::SolidCollisionResponseHandler;

using physics::CollidableBody;
using physics::SolidConnectedCollidableBody;
using physics::ConstraintGroup;
using physics::ContactConstraint;



SolidCollisionResponseHandler::SolidCollisionResponseHandler(ref<SolidSystem> solidSystem, ref<CollisionDetector> collisionDetector)
  : CollisionResponseHandler(collisionDetector), solidSystem(solidSystem)
{
  contactConstraintGroup = solidSystem->createConstraintGroup();
  solidSystem->addConstraintGroup(contactConstraintGroup);
}


SolidCollisionResponseHandler::~SolidCollisionResponseHandler()
{
  solidSystem->removeConstraintGroup(contactConstraintGroup);
}


void SolidCollisionResponseHandler::reset()
{
  contactConstraintGroup->clear();  // clear contact constraints from previous collisions
}


void SolidCollisionResponseHandler::handleCollision(ref<CollisionState> collisionState)
{
  if (   !instanceof(*collisionState->collidable1, const SolidConnectedCollidableBody)
      || !instanceof(*collisionState->collidable2, const SolidConnectedCollidableBody))
    return; // can only handle SolidConnectedCollidableBodies
  
  ref<const SolidConnectedCollidableBody> collidable1(narrow_ref<const SolidConnectedCollidableBody>(collisionState->collidable1) );
  ref<const SolidConnectedCollidableBody> collidable2(narrow_ref<const SolidConnectedCollidableBody>(collisionState->collidable2) );

  CollisionState::Contacts::const_iterator c(collisionState->contacts.begin());
  CollisionState::Contacts::const_iterator end(collisionState->contacts.end());
  while (c != end) {
    const CollisionState::Contact& contact(*c);

    // create the contact constraint & attach it to the Solids
    ref<ContactConstraint> contactConstraint( solidSystem->createContactConstraint() );
    contactConstraint->setContactPosition( contact.p1 );
    contactConstraint->setContactNormal( contact.n1 );
    contactConstraint->setContactDepth( contact.depth );

    contactConstraintGroup->addConstraint( contactConstraint );
    
    contactConstraint->attach(collidable1->getSolid(), collidable2->getSolid());

    ++c;
  }
}

