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

  $Id: ODECollidableBody.cpp 1161 2004-09-29 17:53:32Z jungd $

****************************************************************************/

#include <physics/ODECollidableBody>

#include <physics/Box>
#include <physics/Sphere>
#include <physics/Cylinder>
#include <physics/Capsule>
#include <physics/Polyhedron>


using base::Dimension3;
using base::Point3;
using base::Orient;

using physics::ODECollidableBody;
using physics::Shape;
using physics::Box;
using physics::Sphere;
using physics::Cylinder;
using physics::Capsule;



ODECollidableBody::ODECollidableBody(ref<const Shape> shape)
  : CollidableBody(shape)
{
  init(true);
  setName(shape->className()+" ODECollidableBody");
}

ODECollidableBody::~ODECollidableBody()
{
  dGeomDestroy(geomID);
}


void ODECollidableBody::createGeom()
{
  ref<const Shape> shape(getShape());
  Assert(shape);
  geomID = 0;
  if (instanceof(*shape, const Box)) {
    ref<const Box> box = narrow_ref<const Box>(shape);
    Dimension3 dim = box->dimensions();
    geomID = dCreateBox(0, dim.x, dim.y, dim.z);
  }
  else if (instanceof(*shape, const Sphere)) {
    ref<const Sphere> sphere = narrow_ref<const Sphere>(shape);
    geomID = dCreateSphere(0, sphere->radius());
  }
  else if (instanceof(*shape, const Cylinder)) {
    ref<const Cylinder> cylinder = narrow_ref<const Cylinder>(shape);
    geomID = dCreateCCylinder(0, cylinder->radius(), cylinder->height());
    // !!! switch this to use a real cylinder, not a capped cylinder (capsule)
    // (may need a geom transform, as currently ODE's cylinder is oriented fifferently, unless
    //  that changes)
  }
  else if (instanceof(*shape, const Capsule)) {
    ref<const Capsule> capsule = narrow_ref<const Capsule>(shape);
    geomID = dCreateCCylinder(0, capsule->radius(), capsule->height());
    //!!! is ODE CCylinder axis aligned with Capsule axis?? suspect so (didn't I change the orient of mine to match?)
  }
  else {
    Logln("Unsupported Shape: " << shape->className() << ", using bounding sphere for collision region");
    Real radius = (shape->getBoundingSphere()).radius();
    geomID = dCreateSphere(0, radius);
  }

  // now put this into the geom data so that the ODECollisionCuller can fetch it
  //  (to get the ODECollidableBody form the geomID)
  dGeomSetData(geomID, this);
}


void ODECollidableBody::setPosition(const Point3& x)
{
  dGeomSetPosition(geomID, x.x, x.y, x.z);
}

void ODECollidableBody::setOrientation(const Orient& orient)
{
  dQuaternion quat;
  Quat4 q( orient );
  quat[0] = q.w; quat[1] = q.v.x; quat[2] = q.v.y; quat[3] = q.v.z;
  dGeomSetQuaternion(geomID, quat);
}

Point3 ODECollidableBody::getPosition() const
{
  const dReal* pos = dGeomGetPosition(geomID);
  return Point3(pos[0],pos[1],pos[2]);
}

Orient ODECollidableBody::getOrientation() const
{
  dQuaternion orient;
  dGeomGetQuaternion(geomID, orient);
  return Orient(Quat4(orient[1], orient[2], orient[3], orient[0]));
}


void ODECollidableBody::saveState()
{
  state->savedPos = getPosition();
  state->savedOrient = getOrientation();
  state->savedVel = state->vel;
  state->savedAngVel = state->angVel;
}

void ODECollidableBody::restoreState()
{
  setPosition(state->savedPos);
  setOrientation(state->savedOrient);
  state->vel = state->savedVel;
  state->angVel = state->savedAngVel;
}

