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

  $Id: ODECollidableGroup.cpp 1159 2004-09-29 17:49:19Z jungd $

****************************************************************************/

#include <physics/ODECollidableGroup>

#include <physics/ODECollidableBody>

using base::Dimension3;
using base::Point3;
using base::Orient;

using physics::ODECollidableGroup;
using physics::ODECollidableBody;


ODECollidableGroup::ODECollidableGroup()
{
  setName(className());
  create(ODESimpleSpace);
}

ODECollidableGroup::~ODECollidableGroup()
{
  clear();
  dSpaceDestroy(spaceID);
}


void ODECollidableGroup::create(ODECollidableGroup::ODESpaceType type)
{
  switch (type) {
    case ODESimpleSpace:
      spaceID = dSimpleSpaceCreate(0);
      break;

    case ODEHashSpace: {
      spaceID = dHashSpaceCreate(0);
    } break;

    case ODEQuadTreeSpace: {
      // some defaults
      dVector3 center = {0,0,0};
      dVector3 extent = {1025.0,1025.0,1025.0};
      spaceID = dQuadTreeSpaceCreate(0,center,extent,6);
    } break;

    default:
      spaceID = dSimpleSpaceCreate(0);
  }

  // now put this into the geom data so that the ODECollisionCuller can fetch it
  dGeomSetData((dGeomID)spaceID, this);

  dSpaceSetCleanup(spaceID, 0); // don't delete geoms when space is destroyed
}


void ODECollidableGroup::addCollidable(ref<Collidable> c)
{
  Assert(c);
  if (instanceof(*c, ODECollidableBody)) {
    dSpaceAdd(spaceID, (narrow_ref<ODECollidableBody>(c))->getGeomID() );
  }
  else if (instanceof(*c, ODECollidableGroup)) {
    dSpaceAdd(spaceID, (dGeomID)(narrow_ref<ODECollidableGroup>(c))->getSpaceID() );
  }
  else
    throw std::invalid_argument(Exception("only ODE type Collidables can be added to an ODECollidableGroup (not a "+c->className()+")"));

  CollidableGroup::addCollidable(c);
}


void ODECollidableGroup::removeCollidable(ref<Collidable> c)
{
  dGeomID geomID = 0;
  if (instanceof(*c, ODECollidableBody)) {
    geomID = (narrow_ref<ODECollidableBody>(c))->getGeomID();
  }
  else if (instanceof(*c, ODECollidableGroup)) {
    geomID = (dGeomID)(narrow_ref<ODECollidableGroup>(c))->getSpaceID();
  }

  if (geomID != 0)
    if (dSpaceQuery(spaceID, geomID) == 0)
      geomID = 0;

  if (geomID == 0)
    throw std::invalid_argument(Exception("the specified Collidable is not in this CollidableGroup"));

  dSpaceRemove(spaceID, geomID);
  CollidableGroup::removeCollidable(c);
}


void ODECollidableGroup::clear()
{
  while(dSpaceGetNumGeoms(spaceID) > 0)
    dSpaceRemove(spaceID, dSpaceGetGeom(spaceID, dSpaceGetNumGeoms(spaceID)-1) ); // remove last

  CollidableGroup::clear();
}

