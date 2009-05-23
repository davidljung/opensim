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

  $Id: SimulatedKinematicChain.cpp 1113 2004-09-27 22:07:47Z jungd $

****************************************************************************/

#include <robot/sim/SimulatedKinematicChain>

#include <base/Vector3>
#include <base/Orient>
#include <base/Transform>
#include <base/Externalizer>

#include <gfx/Color3>
#include <gfx/Line3>
#include <gfx/Segment3>
#include <physics/BoundingBox>
#include <physics/Material>
#include <physics/Box>
#include <physics/Sphere>
#include <physics/Cylinder>
#include <physics/Solid>
#include <physics/ConstraintGroup>
#include <physics/Joint>
#include <physics/HingeJoint>
#include <physics/SliderJoint>
#include <physics/Motor>
#include <physics/CollisionCuller>
#include <physics/CollisionDetector>
#include <physics/CollisionState>
#include <physics/NullCollisionResponseHandler>
#include <physics/SpatialTransform>




using robot::sim::SimulatedKinematicChain;

using base::Point3;
using base::Orient;
using base::Dimension3;
using base::Transform;
using gfx::Line3;
using gfx::Segment3;
using gfx::Color4;
using physics::BoundingBox;
using physics::Collidable;
using physics::CollidableBody;
using physics::CollidableGroup;
using physics::CollisionState;
using physics::CollisionCuller;
using physics::CollisionDetector;
using physics::CollisionResponseHandler;
using physics::NullCollisionResponseHandler;
using physics::Shape;
using physics::Box;
using physics::Sphere;
using physics::Cylinder;
using physics::Material;
using physics::Solid;
using physics::SpatialGroup;
using physics::SpatialTransform;
using physics::ConstraintGroup;
using physics::Joint;
using physics::HingeJoint;
using physics::SliderJoint;
using physics::Motor;


using robot::KinematicChain;




array<base::Dimension3> SimulatedKinematicChain::computeLinkDimensions(const array<Real>& linkRadii, Int firstLinkIndex)
{
  // If no explicit geometry is provided for links, a Capsule will be used
  //  The dimensions are based on the link lengths as calculated via
  //  a line segment joining the origins of consecutive links is
  //  calculated here for this purpose.  Link radii are used if supplied.

  Assert(chain.size() > 0);

  array<Dimension3> dim(chain.size()+1); // link dimensions (1..#links)
  linkLengths.resize(chain.size()+1);

  array<Vector> linkOrigin(chain.getLinkOrigins(zeroVector(chain.dof()))); //!!! use home pos?

  for (Int l=1; l<=chain.size(); l++) { // for each link
    Real len;

    if (l>1)
      len = base::toVector3(linkOrigin[l-1] - linkOrigin[l-2]).length();
    else
      len = base::toVector3(linkOrigin[0]).length();

    linkLengths[l] = len;

    Real zl=len*0.98;
    if (Math::equals(len,0)) zl=0.05; // some links are zero length so make them very small instead

    Real yz;
    if (linkRadii.size() >= l+firstLinkIndex)
      yz = linkRadii[ (l-1) + firstLinkIndex ]*2.0;
    else
      yz = Math::maximum(0.05,zl*0.14); // make x&y-dims smaller than length (z-dim)

    dim[l] = Dimension3(yz,yz,zl);
  }

  return dim;
}




// ProximityCollisionResponseHandler
SimulatedKinematicChain::ProximityCollisionResponseHandler::ProximityCollisionResponseHandler(ref<SimulatedKinematicChain> kc,
                                                                                              ref<CollisionDetector> collisionDetector)
  : CollisionResponseHandler(collisionDetector), kc(kc), resetCalled(false)
{
}


void SimulatedKinematicChain::ProximityCollisionResponseHandler::reset()
{
  Assert(kc->links.size() > 0);
  kc->linkProximity.clear().resize( kc->links.size() ); //  all elements now default
  Collider::reset(); // pass it on

  resetCalled = true;
}

const Real SimulatedKinematicChain::maxDist = Real(consts::maxInt);


void SimulatedKinematicChain::ProximityCollisionResponseHandler::collide(ref<const physics::Collidable> collidable1, ref<const physics::Collidable> collidable2)
{
  Assertm(resetCalled,"reset() called before collide()");

  if ((collidable1->getUserData() == kc) || (collidable2->getUserData() == kc)) {

    ref<const CollidableBody> body1( narrow_ref<const CollidableBody>(collidable1) );
    ref<const CollidableBody> body2( narrow_ref<const CollidableBody>(collidable2) );
    handleCollision(collisionDetector->getCollisionState(body1, body2));
  }
  else {
    notifyListeners(collidable1, collidable2); // pass through
  }
}



void SimulatedKinematicChain::ProximityCollisionResponseHandler::handleCollision(ref<physics::CollisionState> collisionState)
{
  kc->handleCollision(collisionState);
}

