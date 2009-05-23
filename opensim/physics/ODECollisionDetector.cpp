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

  $Id: ODECollisionDetector.cpp 1152 2004-09-29 17:39:07Z jungd $

****************************************************************************/

#include <physics/ODECollisionDetector>

#include <physics/ODECollidableBody>

using physics::ODECollisionDetector;

using base::Vector3;
using physics::CollisionState;
using physics::CollidableBody;
using physics::ODECollidableBody;

//!!!debug
#include <physics/SolidConnectedCollidableBody>
using physics::SolidConnectedCollidableBody;



ODECollisionDetector::ODECollisionDetector()
{
}


ODECollisionDetector::~ODECollisionDetector()
{
}


ref<CollidableBody> ODECollisionDetector::createCollidableBody(ref<Shape> shape)
{
  return ref<CollidableBody>(NewNamedObj("ODECollidableBody") ODECollidableBody(shape));
}


ODECollisionDetector::ODECollisionState::ODECollisionState(ref<const CollidableBody> collidable1, ref<const CollidableBody> collidable2)
  : CollisionState(collidable1, collidable2)
{
}


ODECollisionDetector::ODECollisionState::~ODECollisionState()
{
}




ref<CollisionState> ODECollisionDetector::newCollisionState(ref<const CollidableBody> collidable1, ref<const CollidableBody> collidable2) const
{
  if (&(*collidable1) > &(*collidable2))
    base::swap(collidable1,collidable2);

  return ref<ODECollisionState>(NewNamedObj("ODECollisionState") ODECollisionState(collidable1, collidable2) );
}



void ODECollisionDetector::collide(ref<const Collidable> collidable1, ref<const Collidable> collidable2)
{
  if (!isCollisionEnabled()) return;
//!Debugln(Collision,"Colliding non-culled bodies:" << collidable1->getName() << " & " << collidable2->getName());
  Assertm( &(*collidable1) != &(*collidable2), "collidable1 is not collidable2");

  if (!instanceof(*collidable1, const ODECollidableBody) || !instanceof(*collidable2, const ODECollidableBody))
    throw new std::runtime_error(Exception("can only collide() Collidables of type CollidableBody (specifically ODECollidableBody"));

  ref<const ODECollidableBody> collidableBody1(narrow_ref<const ODECollidableBody>(collidable1));
  ref<const ODECollidableBody> collidableBody2(narrow_ref<const ODECollidableBody>(collidable2));

  if (&(*collidableBody1) > &(*collidableBody2))
    base::swap(collidableBody1,collidableBody2);

  // Get the CollisionState for the pair of CollidableBodys
  ref<CollisionState> cstate(getCollisionState(collidableBody1,collidableBody2));


  //
  // Do a pair-wise collision check

  dGeomID geomID1 = collidableBody1->getGeomID();
  dGeomID geomID2 = collidableBody2->getGeomID();
  Assert( !dGeomIsSpace(geomID1) );
  Assert( !dGeomIsSpace(geomID2) );

  // call ODE dCollide
  const Int maxContacts = 4;
  dContactGeom contacts[maxContacts]; //!!! probably needs to be greater (perhaps object dependent, so that boxes only use 4 etc.)
  Int numContacts = dCollide(geomID1, geomID2, maxContacts, contacts, sizeof(dContactGeom));
  bool collided = (numContacts != 0);


  if (collided) {
//!Debugln(Collision,"  **collided**");
//!!!
//String name1( instanceof(*collidable1, const SolidConnectedCollidableBody)? (narrow_ref<const SolidConnectedCollidableBody>(collidable1))->getSolid()->getName():"unknown");
//String name2( instanceof(*collidable2, const SolidConnectedCollidableBody)? (narrow_ref<const SolidConnectedCollidableBody>(collidable2))->getSolid()->getName():"unknown");
//if ((name1 == "TitanII link 3") && (name2 == "ground")) {
//Debugcln(Tmp,"collision between:" << name1 << "(geomID:" << (void*)geomID1
//                            << ") and " << name2 << "(geomID:" << (void*)geomID2 << ")");
//}

    // construct contacts
    cstate->contacts.clear();
    for (Int c=0; c<numContacts; c++) {
      dContactGeom* odeContact(&contacts[c]);
      // transform global contact point into local coords  of each Solid
      Point3 gcpos(odeContact->pos[0], odeContact->pos[1], odeContact->pos[2]); // global coord. frame
      Point3 cpos1( inverse(collidableBody1->getConfiguration()) * gcpos );
      Point3 cpos2( inverse(collidableBody2->getConfiguration()) * gcpos );

      //!!!
//      Quat4 orientation1( collidableBody1->getOrientation() );
//      Quat4 orientation2( collidableBody2->getOrientation() );
//      Point3 position1( collidableBody1->getPosition() );
//      Point3 position2( collidableBody2->getPosition() );

//      Point3 cpos1( inverse(orientation1).rotate(gcpos-position1) );
//      Point3 cpos2( inverse(orientation2).rotate(gcpos-position2) );

      // transform normal into Solid1 frame
      Vector3 gnormal( odeContact->normal[0], odeContact->normal[1], odeContact->normal[2] );
      Orient  inverseRot( collidableBody1->getOrientation() ); inverseRot.invert();
      Vector3 normal1 = inverseRot.rotate(gnormal);

//      Vector3 normal1( inverse(orientation1).rotate(gnormal));

      CollisionState::Contact contact(cpos1, cpos2, normal1, Real(odeContact->depth) );
      cstate->contacts.push_back(contact);
    }

    notifyListeners(collidable1, collidable2);

  }

}

