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
  
  $Id: ODESolid.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.6 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/ODESolid>
#include <base/ref>

// need to know about shapes to map them to ODE's equivelent geoms
#include <physics/Box>
#include <physics/Sphere>

#include <physics/ODECollidableBody>
#include <physics/ODESolidConnectedCollidableBody>


using base::Point3;
using base::Quat4;
using base::Orient;
using base::Vector3;
using base::Matrix3;

using base::copyMemory;

using physics::Body;
using physics::Shape;
using physics::Material;
using physics::MassProperties;
using physics::ODESolid;
using physics::Collidable;
using physics::CollidableBody;
using physics::ODECollidableBody;
using physics::ODESolidConnectedCollidableBody;

using physics::Box;
using physics::Sphere;


  

ODESolid::ODESolid(ref<const Shape> shape, ref<const Material> material)
  : Solid(shape, material), created(false)
{
}

void ODESolid::create(dWorldID worldID) const
{
  if (created)
    throw std::runtime_error(Exception("create() called twice"));

  this->worldID = worldID;

  bodyID = dBodyCreate(worldID);
  // initialize default state
  dBodySetPosition(bodyID, 0,0,0);
  dQuaternion q = { 1,0,0,0 };
  dBodySetQuaternion(bodyID, q);
  dBodySetLinearVel(bodyID, 0,0,0);
  dBodySetAngularVel(bodyID, 0,0,0);

  // calculate mass properties 
  MassProperties massProps( Solid::massProperties() );

  dMass odemass;
  dMassSetZero(&odemass);
  const Matrix3& Ibody(massProps.Ibody());
  dMassSetParameters(&odemass, massProps.mass,
		     0,0,0,
		     Ibody(1,1), Ibody(2,2), Ibody(3,3),
		     Ibody(1,2), Ibody(1,3), Ibody(2,3));

  dBodySetMass(bodyID, &odemass);

  created = true;
}


void ODESolid::destroy() const
{
  if (created) {
    dBodyDestroy(bodyID);
    worldID = 0; 
    bodyID = 0;
    created = false;
  }
}




ODESolid::ODESolid(const Solid& s)
  : Solid(s)
{
  // copy state (prev state will be lost)
  setPosition(s.getPosition());
  setOrientation(s.getOrientation());
  setVelocity(s.getVelocity());
  setAngVelocity(s.getAngVelocity());
  setEnabled(s.isEnabled());
}

ODESolid::~ODESolid() 
{
  if (created) destroy();
}


MassProperties ODESolid::massProperties() const
{
  if (!created) return Solid::massProperties();

  // get properties from ODE
  dMass odemass;
  dBodyGetMass(bodyID, &odemass);

  MassProperties massProps;
  massProps.mass = odemass.mass;
  massProps.centerOfMass = Point3(odemass.c[0], odemass.c[1], odemass.c[2]);
  Matrix3 Ibody;
  // get from standard ODE matrix format. order or row/col irrelevant as matrix is symmetric
  for(int r=1; r<3; r++)
    for(int c=1; c<3; c++)
      Ibody(r,c) = odemass.I[(r-1)*4+(c-1)];
  massProps.setIbody(Ibody);

  return massProps;
}



Body& ODESolid::operator=(const Body& b)
{
  if (!created) { Logln("Warning: no effect until added to SolidSystem"); return *this; }

  setPosition(b.getPosition());
  setOrientation(b.getOrientation());
  setVelocity(b.getVelocity());
  setAngVelocity(b.getAngVelocity());
  setEnabled(true);
  return *this;
}


void ODESolid::setEnabled(bool enable)
{
  if (!created) { Logln("Warning: no effect until added to SolidSystem"); return; }

  if (enable)
    dBodyEnable(bodyID);
  else
    dBodyDisable(bodyID);
}

bool ODESolid::isEnabled() const
{
  if (!created) { Logln("Warning: no effect until added to SolidSystem"); return false; }

  return dBodyIsEnabled(bodyID);
}


void ODESolid::setPosition(const Point3& x)
{
  if (!created) { Logln("Warning: no effect until added to SolidSystem"); return; }

  dBodySetPosition(bodyID, x.x, x.y, x.z);
}

void ODESolid::ODESolid::setOrientation(const Orient& orient)
{
  if (!created) { Logln("Warning: no effect until added to SolidSystem"); return; }

  dQuaternion quat;
  Quat4 q( orient );
  quat[0] = q.w; quat[1] = q.v.x; quat[2] = q.v.y; quat[3] = q.v.z;
  dBodySetQuaternion(bodyID, quat);
}

void ODESolid::setVelocity(const Vector3& v)
{
  if (!created) { Logln("Warning: no effect until added to SolidSystem"); return; }

  dBodySetLinearVel(bodyID, v.x, v.y, v.z);
}

void ODESolid::setAngVelocity(const Vector3& w)
{
  if (!created) { Logln("Warning: no effect until added to SolidSystem"); return; }

  dBodySetAngularVel(bodyID, w.x, w.y, w.z);

}

Point3  ODESolid::getPosition() const
{
  if (!created) return Point3(); 

  const dReal* pos = dBodyGetPosition(bodyID);
  return Point3(pos[0],pos[1],pos[2]);
}

Orient  ODESolid::getOrientation() const
{
  if (!created) return Orient(); 

  const dReal* orient = dBodyGetQuaternion(bodyID);
  return Orient(Quat4(orient[1], orient[2], orient[3], orient[0]));
}

Vector3 ODESolid::getVelocity() const
{
  if (!created) return Vector3(); 

  const dReal* vel = dBodyGetLinearVel(bodyID);
  return Vector3(vel[0], vel[1], vel[2]);
}


Vector3 ODESolid::getAngVelocity() const
{
  if (!created) return Vector3(); 

  const dReal* angVel = dBodyGetAngularVel(bodyID);
  return Vector3(angVel[0], angVel[1], angVel[2]);
}

void ODESolid::saveState()
{
  if (!created) return; 

  copyMemory(dBodyGetPosition(bodyID), savedPos, 3);
  copyMemory(dBodyGetQuaternion(bodyID), savedOrient, 4);
  copyMemory(dBodyGetLinearVel(bodyID), savedVel, 3);
  copyMemory(dBodyGetAngularVel(bodyID), savedAngVel, 3);
}

void ODESolid::restoreState()
{
  if (!created) return; 

  dBodySetPosition(bodyID, savedPos[0], savedPos[1], savedPos[2]);
  dBodySetQuaternion(bodyID, savedOrient);
  dBodySetLinearVel(bodyID, savedVel[0], savedVel[1], savedVel[2]);
  dBodySetAngularVel(bodyID, savedAngVel[0], savedAngVel[1], savedAngVel[2]);
}

Point3  ODESolid::getSavedPosition() const
{
  if (!created) return Point3(); 

  return Point3(savedPos[0],savedPos[1],savedPos[2]);
}

Orient  ODESolid::getSavedOrientation() const
{
  if (!created) return Orient(); 

  return Orient(Quat4(savedOrient[1], savedOrient[2], savedOrient[2], savedOrient[0]));
}

Vector3 ODESolid::getSavedVelocity() const
{
  if (!created) return Vector3(); 

  return Vector3(savedVel[0], savedVel[1], savedVel[2]);
}

Vector3 ODESolid::getSavedAngVelocity() const
{
  if (!created) return Vector3(); 

  return Vector3(savedAngVel[0], savedAngVel[1], savedAngVel[2]);
}


Point3 ODESolid::getRelPointPos(const Point3& p)
{
  if (!created) return p; 

  dVector3 result;
  dBodyGetRelPointPos(bodyID, p.x, p.y, p.z, result);
  return Point3(result[0], result[1], result[2]);
}

Vector3 ODESolid::getRelPointVel(const Point3& p)
{
  if (!created) return p; 

  dVector3 result;
  dBodyGetRelPointVel(bodyID, p.x, p.y, p.z, result);
  return Vector3(result[0], result[1], result[2]);
}


Point3 ODESolid::getGlobalPointRelPos(const Point3& p)
{
  Point3 local(p - getPosition());
  getOrientation().invert().rotatePoint(local);
  return local;
}



void ODESolid::addForce(const Vector3& f)
{
  if (!created) return; 
  dBodyAddForce(bodyID, f.x, f.y, f.z);
}

void ODESolid::addTorque(const Vector3& t)
{
  if (!created) return; 
  dBodyAddTorque(bodyID, t.x, t.y, t.z);
}

void ODESolid::addRelForce(const Vector3& f)
{
  if (!created) return; 
  dBodyAddRelForce(bodyID, f.x, f.y, f.z);
}

void ODESolid::addRelTorque(const Vector3& t)
{
  if (!created) return; 
  dBodyAddRelTorque(bodyID, t.x, t.y, t.z);
}

void ODESolid::addForceAtPos(const Vector3& f, const Point3& p)
{
  if (!created) return; 
  dBodyAddForceAtPos(bodyID, f.x, f.y, f.z, p.x, p.y, p.z);
}

void ODESolid::addForceAtRelPos(const Vector3& f, const Point3& p)
{
  if (!created) return; 
  dBodyAddForceAtRelPos(bodyID, f.x, f.y, f.z, p.x, p.y, p.z);
}

void ODESolid::addRelForceAtPos(const Vector3& f, const Point3& p)
{
  if (!created) return; 
  dBodyAddRelForceAtPos(bodyID, f.x, f.y, f.z, p.x, p.y, p.z);
}

void ODESolid::addRelForceAtRelPos(const Vector3& f, const Point3& p)
{
  if (!created) return; 
  dBodyAddRelForceAtRelPos(bodyID, f.x, f.y, f.z, p.x, p.y, p.z);
}

Vector3 ODESolid::getForce() const
{
  if (!created) return Vector3(); 

  const dReal* f = dBodyGetForce(bodyID);
  return Vector3(f[0], f[1], f[32]);
}

Vector3 ODESolid::getTorque() const
{
  if (!created) return Vector3(); 

  const dReal* t = dBodyGetTorque(bodyID);
  return Vector3(t[0], t[1], t[2]);
}


ref<Collidable> ODESolid::createCollidable(ref<const Shape> collisionShape, CollidableFlags flags)
{
  if (!flags.test(SolidNotConnected)) {
    ref<Solid> self( this );
    return ref<CollidableBody>(NewObj ODESolidConnectedCollidableBody(self, collisionShape));
  }
  else
    return ref<CollidableBody>(NewObj ODECollidableBody(getShape()));
}

