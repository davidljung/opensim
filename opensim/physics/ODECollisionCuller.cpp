/****************************************************************************
  Copyright (C)2003 David Jung <opensim@pobox.com>

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

  $Id: ODECollisionCuller.cpp 1158 2004-09-29 17:45:25Z jungd $

****************************************************************************/

#include <physics/ODECollisionCuller>

#include <physics/ODECollidableGroup>

#include <physics/ODECollidableBody>
//!!!
#include <physics/VisualDebugUtil>

using physics::ODECollisionCuller;

using physics::Collider;
using physics::Collidable;
using physics::CollidableBody;
using physics::ODECollidableBody;
using physics::CollidableGroup;
using physics::ODECollidableGroup;


ref<CollidableGroup> ODECollisionCuller::createCollidableGroup() const
{
  return ref<ODECollidableGroup>(NewObj ODECollidableGroup());
}


void ODECollisionCuller::nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  (static_cast<ODECollisionCuller*>(data))->callback(o1,o2);
}


void ODECollisionCuller::callback(dGeomID o1, dGeomID o2)
{
  if (dGeomIsSpace(o1)) {

    ref<const ODECollidableGroup> group1( (const ODECollidableGroup*)dGeomGetData(o1) );

    if (dGeomIsSpace(o2)) {
      ref<const ODECollidableGroup> group2( (const ODECollidableGroup*)dGeomGetData(o2) );
//!Debugln(Collision,"collision(g-g):" << group1->getName() << " & " << group2->getName())
      collide(group1, group2);
    }
    else {
      ref<const ODECollidableBody> body2( (const ODECollidableBody*)dGeomGetData(o2) );
      collide(group1, body2);
//!Debugln(Collision,"collision(g-b):" << group1->getName() << " & " << body2->getName())
    }

  }
  else {

    ref<const ODECollidableBody> body1( (const ODECollidableBody*)dGeomGetData(o1) );

    if (dGeomIsSpace(o2)) {
      ref<const ODECollidableGroup> group2( (const ODECollidableGroup*)dGeomGetData(o2) );
//!Debugln(Collision,"collision(b-g):" << body1->getName() << " & " << group2->getName())
      collide(body1, group2);
    }
    else {
      ref<const ODECollidableBody> body2( (const ODECollidableBody*)dGeomGetData(o2) );
//!Debugln(Collision,"collision(b-b):" << body1->getName() << " & " << body2->getName())
      collide(body1,body2);
    }

  }

}



void ODECollisionCuller::collide(ref<const Collidable> collidable)
{
  if (!isCollisionEnabled()) return;

  if (!instanceof(*collidable, const CollidableGroup))
    return; // can't collide one Collidable with anything - do nothing

  Assert(instanceof(*collidable, const ODECollidableGroup));
  ref<const ODECollidableGroup> group(narrow_ref<const ODECollidableGroup>(collidable));
  if (!group->isChildIntercollisionEnabled()) { Debugln(Tmp, "child collision disabled, returning"); return;}//!!!
Collider::collide(collidable);
  //!!!dSpaceCollide(group->getSpaceID(), this, nearCallback);
}



void ODECollisionCuller::collide(ref<const Collidable> collidable1, ref<const Collidable> collidable2)
{
  if (!isCollisionEnabled()) return;

  if (collidable1->getUserClass() != 0)
    if (collidable1->getUserClass() == collidable2->getUserClass()) {
//!Debugln(Tmp,"CLASS CULL:" << collidable1->getName() << " and " << collidable2->getName());
      return; // same user class (not 0) => cull
    }

//!!!

if (instanceof(*collidable1, const ODECollidableBody)) {
  VisualDebugUtil::setConfiguration(  collidable1->getName()+" Sensor", narrow_ref<const ODECollidableBody>(collidable1)->getConfiguration());
  //Debugcln(Tmp,"set " << collidable1->getName());
}
if (instanceof(*collidable2, const ODECollidableBody)) {
  VisualDebugUtil::setConfiguration(  collidable2->getName()+" Sensor", narrow_ref<const ODECollidableBody>(collidable2)->getConfiguration());
  //Debugcln(Tmp,"set " << collidable2->getName());
}
//!!!

  if (instanceof(*collidable1, const ODECollidableBody) && instanceof(*collidable2, const ODECollidableBody)) {
    if (!isCollisionEnabled(collidable1, collidable2)) return; // cull
//!Debugln(Tmp,"PASSED0: " << collidable1->getName() << " and " << collidable2->getName());
    notifyListeners(collidable1, collidable2); // both bodies
    return;
  }

  if (instanceof(*collidable1, const ODECollidableGroup) && instanceof(*collidable2, const ODECollidableGroup)) {
    // both groups
//!Debugln(Tmp,"RECURSING 2 GROUPS:" << collidable1->getName() << " and " << collidable2->getName());

    ref<const ODECollidableGroup> group1(narrow_ref<const ODECollidableGroup>(collidable1));
    ref<const ODECollidableGroup> group2(narrow_ref<const ODECollidableGroup>(collidable2));

    if (isCollisionEnabled(collidable1, collidable2))
      dSpaceCollide2((dGeomID)group1->getSpaceID(), (dGeomID)group2->getSpaceID(), this, nearCallback);

    // collide each the elements of the groups among themselves
    if (group1->isChildIntercollisionEnabled())
      dSpaceCollide(group1->getSpaceID(), this, nearCallback);

    if (group2->isChildIntercollisionEnabled())
      dSpaceCollide(group2->getSpaceID(), this, nearCallback);

  }
  else {
//!Debugln(Tmp,"RECURSING 1 GROUP:" << collidable1->getName() << " and " << collidable2->getName());

    // one of each, make the group the first one
    if (instanceof(*collidable1, const ODECollidableBody))
      base::swap(collidable1, collidable2);

    // now collidable1 is group & collidable2 is a body
    // Collide each group element with the body
    Assert(instanceof(*collidable1, const ODECollidableGroup));
    Assert(instanceof(*collidable2, const ODECollidableBody));

    ref<const ODECollidableGroup> group(narrow_ref<const ODECollidableGroup>(collidable1));
    ref<const ODECollidableBody>  body(narrow_ref<const ODECollidableBody>(collidable2));

    // collide each element of the group with the body
    if (isCollisionEnabled(collidable1, collidable2))
      dSpaceCollide2((dGeomID)group->getSpaceID(), body->getGeomID(), this, nearCallback);

    // and collide the group elements among themselves
    if (group->isChildIntercollisionEnabled())
      dSpaceCollide(group->getSpaceID(), this, nearCallback);

  }

}
