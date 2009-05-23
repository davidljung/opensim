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

  $Id: ODESolidSystem.cpp 1154 2004-09-29 17:41:14Z jungd $

****************************************************************************/

#include <physics/CollisionState>

#include <physics/ODESolidSystem>
#include <physics/ODECollisionCuller>
#include <physics/ODECollisionDetector>
#include <physics/SolidCollisionResponseHandler>
#include <physics/NullCollisionResponseHandler>
#include <physics/ODECollidableGroup>
#include <physics/ODESolid>
#include <physics/ODESolidConnectedCollidableBody>

#include <physics/ODEConstraintGroup>
#include <physics/ODEFixedConstraint>
#include <physics/ODEContactConstraint>
#include <physics/ODEBallJoint>
#include <physics/ODEHingeJoint>
#include <physics/ODEDoubleHingeJoint>
#include <physics/ODESliderJoint>
#include <physics/ODEUniversalJoint>

#include <physics/ODEMotor>

using physics::ODESolidSystem;


using physics::Collider;
using physics::CollisionCuller;
using physics::CollisionDetector;
using physics::ODECollisionDetector;
using physics::Collidable;
using physics::CollidableGroup;
using physics::ODECollidableGroup;
using physics::ODECollisionCuller;
using physics::Solid;
using physics::ODESolid;
using physics::ODESolidConnectedCollidableBody;
using physics::ConstraintGroup;
using physics::ODEConstraintGroup;
using physics::CollisionResponseHandler;
using physics::SolidCollisionResponseHandler;

using physics::FixedConstraint;
using physics::ContactConstraint;
using physics::BallJoint;
using physics::HingeJoint;
using physics::DoubleHingeJoint;
using physics::SliderJoint;
using physics::UniversalJoint;
using physics::Motor;

using physics::ODEFixedConstraint;
using physics::ODEBallJoint;
using physics::ODEHingeJoint;
using physics::ODEDoubleHingeJoint;
using physics::ODESliderJoint;
using physics::ODEUniversalJoint;
using physics::ODEMotor;



ODESolidSystem::ODESolidSystem()
  : active(true), preSimulateCalled(false)
{
  worldID = dWorldCreate();

  collisionCuller = ref<CollisionCuller>(NewObj ODECollisionCuller());
  collisionDetector = ref<CollisionDetector>(NewObj ODECollisionDetector());
  collisionHandler = ref<CollisionResponseHandler>(NewObj SolidCollisionResponseHandler(ref<SolidSystem>(this), collisionDetector)); //!!!cyclic ref!

  collisionCuller->addPotentialCollisionListener(collisionDetector);
  collisionDetector->addPotentialCollisionListener(ref<ODESolidSystem>(this));

}

ODESolidSystem::ODESolidSystem(const ODESolidSystem& ss)
  : SolidSystem(ss), active(ss.active), preSimulateCalled(false)
{
  Unimplemented;
}


ODESolidSystem::~ODESolidSystem()
{
  // release the ConstraintGroups first
  //  (so that the Constraints may be destroyed before the world)
  ConstraintGroups::iterator g = constraintGroups.begin();
  ConstraintGroups::iterator end = constraintGroups.end();
  while (g != end) {
    (*g)->clear();
    ++g;
  }
  constraintGroups.clear();

  setGround(ref<Solid>(0),Point3());

  solids.clear();

  collisionCuller = ref<CollisionCuller>(0);
  collisionDetector = ref<CollisionDetector>(0);
  collisionHandler = ref<CollisionResponseHandler>(0);

  dWorldDestroy(worldID);
}




// factories

ref<Solid> ODESolidSystem::createSolid(ref<const Shape> shape, ref<const Material> material)
{
  return ref<Solid>(NewNamedObj("ODESolid") ODESolid(shape, material));
}


ref<ConstraintGroup> ODESolidSystem::createConstraintGroup()
{
  return ref<ConstraintGroup>(NewNamedObj("ODEConstraintGroup") ODEConstraintGroup(worldID));
}


ref<BallJoint> ODESolidSystem::createBallJoint()
{
  return ref<BallJoint>(NewNamedObj("ODEBallJoint") ODEBallJoint());
}

ref<HingeJoint> ODESolidSystem::createHingeJoint()
{
  return ref<HingeJoint>(NewNamedObj("ODEHingeJoint") ODEHingeJoint());
}

ref<DoubleHingeJoint> ODESolidSystem::createDoubleHingeJoint()
{
  return ref<DoubleHingeJoint>(NewNamedObj("ODEDoubleHingeJoint") ODEDoubleHingeJoint());
}

ref<SliderJoint> ODESolidSystem::createSliderJoint()
{
  return ref<SliderJoint>(NewNamedObj("ODESliderJoint") ODESliderJoint());
}

ref<UniversalJoint> ODESolidSystem::createUniversalJoint()
{
  return ref<UniversalJoint>(NewNamedObj("ODEUniversalJoint") ODEUniversalJoint());
}


ref<FixedConstraint> ODESolidSystem::createFixedConstraint()
{
  return ref<FixedConstraint>(NewNamedObj("ODEFixedConstraint") ODEFixedConstraint());
}

ref<ContactConstraint> ODESolidSystem::createContactConstraint()
{
  return ref<ContactConstraint>(NewNamedObj("ODEContactConstraint") ODEContactConstraint());
}


ref<Motor> ODESolidSystem::createMotor()
{
  return ref<ODEMotor>(NewNamedObj("ODEMotor") ODEMotor());
}


void ODESolidSystem::setGround(ref<Solid> ground, const Point3& position)
{
  if (this->ground) {
    dJointDestroy(groundJoint);
    ref<Solid> cground = this->ground;
    this->ground = ref<Solid>(0);
    removeSolid(cground);
  }

  if (ground) {
    addSolid(ground);
    this->ground = ground;
    ground->setPosition(position);
    groundJoint = dJointCreateFixed(worldID,0);
    ref<ODESolid> osolid = narrow_ref<ODESolid>(ground);
    dJointAttach(groundJoint, osolid->bodyID, 0);
    dJointSetFixed(groundJoint);
  }

}


ref<Solid> ODESolidSystem::getGround() const
{
  return ground;
}


void ODESolidSystem::setGravity(const Vector3& v)
{
  dWorldSetGravity(worldID, v.x, v.y, v.z);
}


void ODESolidSystem::addSolid(ref<Solid> solid)
{
 if ((!solid) || (solid && (solid == ground)))
   throw std::invalid_argument(Exception("solid must be non-null (and cannot be the ground Solid)"));

  ref<ODESolid> osolid = narrow_ref<ODESolid>(solid);
  osolid->create(worldID);
  solids.push_back(solid);

  updateVisual();
}


void ODESolidSystem::removeSolid(ref<const Solid> solid)
{
 if ((!solid) || (solid && (solid == ground)))
   throw std::invalid_argument(Exception("solid must be non-null (and cannot be the ground Solid)"));

  ref<const ODESolid> osolid = narrow_ref<const ODESolid>(solid);
  if (osolid->worldID != worldID)
    throw std::invalid_argument(Exception("Solid doesn't belong to this SolidSystem"));
  osolid->destroy();
  solids.remove(solid);

  updateVisual();
}



void ODESolidSystem::addConstraintGroup(ref<ConstraintGroup> group)
{
  ref<ODEConstraintGroup> ogroup = narrow_ref<ODEConstraintGroup>(group);
  if (ogroup->getWorldID() != worldID)
    throw std::invalid_argument(Exception("ConstraintGroup doesn't belong to this SolidSystem"));
  constraintGroups.push_back(ogroup);
}


void ODESolidSystem::removeConstraintGroup(ref<const ConstraintGroup> group)
{
  ref<const ODEConstraintGroup> ogroup = narrow_ref<const ODEConstraintGroup>(group);
  if (ogroup->getWorldID() != worldID)
    throw std::invalid_argument(Exception("ConstraintGroup doesn't belong to this SolidSystem"));

  constraintGroups.ConstraintGroups::remove(ogroup);
}


void ODESolidSystem::setCollisionCuller(ref<CollisionCuller> collisionCuller)
{
  // first disconnect the current culler from the detector
  this->collisionCuller->removePotentialCollisionListener(collisionDetector);

  // now connect up the new one
  this->collisionCuller = collisionCuller;
  collisionCuller->addPotentialCollisionListener(collisionDetector);
}


ref<CollisionCuller> ODESolidSystem::getCollisionCuller() const
{
  return collisionCuller;
}


void ODESolidSystem::setCollisionDetector(ref<CollisionDetector> collisionDetector)
{
  // first disconnect the current detector from the culler & us
  collisionCuller->removePotentialCollisionListener(this->collisionDetector);
  this->collisionDetector->removePotentialCollisionListener(ref<ODESolidSystem>(this));

  // now connect up the new one
  this->collisionDetector = collisionDetector;
  collisionCuller->addPotentialCollisionListener(collisionDetector);
  collisionDetector->addPotentialCollisionListener(ref<ODESolidSystem>(this));
}


ref<CollisionDetector> ODESolidSystem::getCollisionDetector() const
{
  return collisionDetector;
}


void ODESolidSystem::setCollisionResponseHandler(ref<CollisionResponseHandler> collisionResponseHandler)
{
  collisionHandler = collisionResponseHandler;
}

ref<CollisionResponseHandler> ODESolidSystem::getCollisionResponseHandler() const
{
  return collisionHandler;
}



void ODESolidSystem::setParameter(const String& name, Real value)
{
  if (name == String("ERP"))
    dWorldSetERP(worldID, value);
  else if (name == String("CFM"))
    dWorldSetCFM(worldID, value);
  else
    SolidSystem::setParameter(name, value);
}


// PotentialCollisionListener

void ODESolidSystem::reset()
{
  collisionHandler->reset();
}

void ODESolidSystem::potentialCollision(ref<const Collidable> collidable1, ref<const Collidable> collidable2)
{
  if (collisionListenMode == HandleCollisions) { // pass collisions on to handler
    collisionHandler->potentialCollision(collidable1, collidable2);
  }
  else { // just initialy checking for interpenetrations

    if (!collidable1->isInterpenetrationNormal() && !collidable2->isInterpenetrationNormal())
      initiallyPenetratingPairs.push_back( std::make_pair<ref<const Collidable>, ref<const Collidable> >(collidable1, collidable2) );
  }
}


void ODESolidSystem::preSimulate()
{
  // check that the current configurations for Solids is a valid one
  //  before simulating (i.e. check there are no penetrations)
  initialPenetrations = false;
  collisionListenMode = RecordInterpenetrations;
  initiallyPenetratingPairs.clear();

  if (collisionCuller != 0) {
    collisionCuller->collisionEnable(true);
    if (collidable == 0)
      collidable = createCollidable();
    collisionCuller->reset();
    collisionCuller->collide(collidable);
  }

  if (initialPenetrations) {
    Consoleln("The initial configuration of some Solids is one of inter-penetration (see log) - system inactivated.");
    active=false;
    collisionCuller->collisionEnable(false);
  }

  // permanently disable collisions between all initially penetrating pairs
  PenetratingPairs::const_iterator ppair = initiallyPenetratingPairs.begin();
  PenetratingPairs::const_iterator end = initiallyPenetratingPairs.end();
  while (ppair != end) {
    const CollidablePair& pair(*ppair);
    collisionCuller->collisionEnable(false, pair.first, pair.second);

    // log that we're disabling collisions
    String name1("unnamed"), name2("unnamed");
    if ( instanceof(*pair.first, const ODESolidConnectedCollidableBody))
      name1 = narrow_ref<const ODESolidConnectedCollidableBody>(pair.first)->getName();
    if ( instanceof(*pair.second, const ODESolidConnectedCollidableBody))
      name2 = narrow_ref<const ODESolidConnectedCollidableBody>(pair.second)->getName();

    Logln("Warning: Disabling collisions between CollidableBodies '" << name1 << "' and '" << name2 << "' as they are initiallly penetrating.");

    ++ppair;
  }


  initiallyPenetratingPairs.clear();
  collisionListenMode = HandleCollisions;

  preSimulateCalled = true;
}


void ODESolidSystem::simulateForSimTime(const base::Time& dt)
{
  Assertm(preSimulateCalled,"preSimulate() called before simulateForSimTime()");

  // Do Collision detection
  if (collisionCuller != 0) {

    if (collidable != 0) {
//!Debugln(Tmp,"\n\nTOP LEVEL COLLISION START");//!!!
      collisionCuller->reset();
      collisionCuller->collide(collidable);
//!Debugln(Tmp,"\n\nTOP LEVEL COLLISION END\n\n\n");
    }
  }

  if (active && (dt>0))
    dWorldStep(worldID, dt.seconds());

  if (node != 0) { // is there a Visual?
    // update Visual
    Solids::iterator s = solids.begin();
    Solids::iterator end = solids.end();
    while (s != end) {
      ref<ODESolid> solid = narrow_ref<ODESolid>(*s);
      solid->updateVisual();

      ++s;
    }
  }
}





osg::Node* ODESolidSystem::createOSGVisual(Visual::Attributes visualAttributes) const
{
  if ((node!=0) && (attributes==visualAttributes))
    return &(*node);

  node = new osg::Group();
  node->setName("ODESolidSystem");

  attributes = visualAttributes;

  updateVisual();

  return &(*node);
}


void ODESolidSystem::updateVisual() const
{
  if (node==0) return; // no Visual to update

  // remove all Solids from the top-level group and re-add them (to account for
  //  additions and removals)
  while (node->getNumChildren() > 0)
    node->removeChild(node->getChild(0));

  // add solids
  Solids::const_iterator_const s = solids.const_begin();
  Solids::const_iterator_const end = solids.const_end();
  while (s != end) {
    ref<const Solid> solid = *s;
    if (solid->visualTypeSupported(Visual::OSGVisual))
      node->addChild(solid->createOSGVisual(attributes));
    ++s;
  }
//!!! and collider visuals etc.
  // add collision detector visual (if any)
  if (collisionDetector)
    if (collisionDetector->visualTypeSupported(Visual::OSGVisual))
      node->addChild(collisionDetector->createOSGVisual(attributes));

}


ref<Collidable> ODESolidSystem::createCollidable(CollidableFlags flags)
{
  // just create CollidableBodys for all the Solids and put them in
  //  a single group
  ref<ODECollidableGroup> group(NewObj ODECollidableGroup());
  group->setName("ODESolidSystem group");

  Solids::const_iterator s = solids.begin();
  Solids::const_iterator end = solids.end();
  while (s != end) {
    ref<Solid> solid = *s;
    group->addCollidable( solid->createCollidable(flags) );
    ++s;
  }

  return group;
}

