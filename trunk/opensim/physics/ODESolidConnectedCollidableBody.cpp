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

  $Id: ODESolidConnectedCollidableBody.cpp 1153 2004-09-29 17:40:21Z jungd $

****************************************************************************/

#include <physics/ODESolidConnectedCollidableBody>

#include <physics/ODECollidableBody>
#include <physics/ODESolid>
#include <physics/Box>
#include <physics/Sphere>
#include <physics/Cylinder>
#include <physics/Polyhedron>


using base::Dimension3;
using base::Point3;
using base::Orient;

using physics::ODESolidConnectedCollidableBody;
using physics::ODECollidableBody;
using physics::ODESolid;
using physics::Shape;
using physics::Box;
using physics::Sphere;
using physics::Cylinder;



ODESolidConnectedCollidableBody::ODESolidConnectedCollidableBody(ref<Solid> solid, ref<const Shape> shape)
  : CollidableBody(shape), SolidConnectedCollidableBody(solid, shape), ODECollidableBody(shape, false)
{
  Assert(getShape());

  Assert(geomID != 0);
  Assert( instanceof(*solid, ODESolid) );
  ref<ODESolid> osolid( narrow_ref<ODESolid>(solid) );
  dGeomSetBody( geomID, osolid->getBodyID() ); // connect geom to body
  setName( osolid->getName());
//!!!  Debugcln(Tmp,"geomID=" << (void*)geomID << " " << getName());
}

ODESolidConnectedCollidableBody::~ODESolidConnectedCollidableBody()
{
//Debugcln(Tmp,"destroying geomID=" << (void*)geomID << " " << getName());//!!!
  dGeomSetBody( geomID, 0 ); // disconnect geom from body
  dGeomDestroy(geomID);
  geomID=0;
}


