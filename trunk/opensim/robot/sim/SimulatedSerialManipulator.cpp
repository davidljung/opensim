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

  $Id: SimulatedSerialManipulator.cpp 1107 2004-09-27 22:00:48Z jungd $

****************************************************************************/

#include <robot/sim/SimulatedSerialManipulator>

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
#include <physics/Capsule>
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

#include <physics/VisualDebugUtil>

#include <robot/KinematicChain>


using robot::sim::SimulatedSerialManipulator;

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
using physics::Capsule;
using physics::Material;
using physics::Solid;
using physics::SpatialGroup;
using physics::SpatialTransform;
using physics::ConstraintGroup;
using physics::Joint;
using physics::HingeJoint;
using physics::SliderJoint;
using physics::Motor;

using physics::VisualDebugUtil;

using robot::KinematicChain;
using robot::sim::SimulatedManipulatorDescription;
using robot::sim::SimulatedKinematicChain;
using robot::sim::SimulatedTool;

//!!!
#include <physics/ODESolidConnectedCollidableBody>
using physics::SolidConnectedCollidableBody;
using physics::ODESolidConnectedCollidableBody;


#include <ode/ode.h>
#include <ode/rotation.h>


SimulatedSerialManipulator::SimulatedSerialManipulator(bool dynamic)
  : SimulatedKinematicChain(dynamic), proximityDistance(0.05), proximityAngle(Math::degToRad(3)), toolGrasped(false)
{
}

SimulatedSerialManipulator::SimulatedSerialManipulator(ref<physics::SolidSystem> solidSystem, bool dynamic)
  : SimulatedKinematicChain(solidSystem, dynamic), proximityDistance(0.05), proximityAngle(Math::degToRad(3)), toolGrasped(false)
{
}

SimulatedSerialManipulator::SimulatedSerialManipulator(ref<const robot::ManipulatorDescription> manipulatorDescription, ref<physics::SolidSystem> solidSystem, bool dynamic)
  : SimulatedKinematicChain(solidSystem,dynamic),
    proximityDistance(0.05), proximityAngle(Math::degToRad(3)), toolGrasped(false)
{
  if (instanceof(*manipulatorDescription, const SimulatedManipulatorDescription))
    manipulatorDescr = narrow_ref<const SimulatedManipulatorDescription>(manipulatorDescription);
  else
    manipulatorDescr = ref<SimulatedManipulatorDescription>(NewObj SimulatedManipulatorDescription(*manipulatorDescription));

  chain = manipulatorDescr->getKinematicChain();
  q.reset(manipulatorDescr->initialConfiguration());
}



bool SimulatedSerialManipulator::formatSupported(String format, Real version, ExternalizationType type) const
{
  return false;
}

void SimulatedSerialManipulator::externalize(base::Externalizer& e, String format, Real version)
{
  if (format == "") format = "xml";

  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" v"+base::realToString(version)+" unsupported"));

  // just externalize the description
  manipulatorDescr->externalize(e,format,version);
}



void SimulatedSerialManipulator::setJointForce(Int j, Real f)
{
  Assertm((j>=1)&&(j<=chain.dof()), "joint index in range");

  if (!dynamic) return;

  ref<Motor> motor(joints[j-1]->getMotor(1));
  motor->setTargetVel( (f>0)?10:-10 );
  motor->setMaxForce(f);
}

void SimulatedSerialManipulator::setJointVel(Int j, Real v, Real maxForce)
{
  Assertm((j>=1)&&(j<=chain.dof()), "joint index in range");

  if (!dynamic) return;

  ref<Motor> motor(joints[j-1]->getMotor(1));
  motor->setTargetVel(v);
  motor->setMaxForce(maxForce);
}


void SimulatedSerialManipulator::setJointPos(Int j, Real p)
{
  Assertm((j>=1)&&(j<=chain.dof()), "joint index in range");

  q[j-1] = p;

  //... actually move the links
  //hack!!! (expensive - only need to update the joint in question)

  Transform baseTransform(manipulatorDescr->getBaseTransform());
  mountConfiguration = inverse(baseTransform)*linkGroups[0]->getConfiguration();

  TransformInfo transformInfo( computeLinkTransforms(mountConfiguration, q) );
  positionLinks(transformInfo);

  // re-position the tool at the ee-end
  if (toolGrasped) {
    Transform eeConfig( getEEPosition(), getEEOrientation() );
    proximityTool->setConfiguration( eeConfig );
  }
  //hack!!!
}


Real SimulatedSerialManipulator::getJointPos(Int j) const
{
  Assertm((j>=1)&&(j<=chain.dof()), "joint index in range");

  if (dynamic) { // get joint pos from physics::Joint

    // NB: physics::Joints have their 0 pos at the position/orientation
    //  at which the two bodies were initially attached.  That was
    //  computed from the manipulator joint parameter home position.

    if (chain[j-1].type() == KinematicChain::Link::Revolute) {

      ref<HingeJoint> hinge(narrow_ref<HingeJoint>(joints[j-1]));

      q[j-1] = hinge->getAngle();

    } else { // prismatic
      ref<SliderJoint> slider(narrow_ref<SliderJoint>(joints[j-1]));
      q[j-1] = slider->getPosition();
    }

  } else { // static
    /*
    Transform relConfig = inverse( links[j]->getConfiguration() )*links[j-1]->getConfiguration();
    Real rotAboutZ = Math::normalizeAngle(relConfig.getRotation().getVector(Orient::EulerZYX)[0]);
    Debugln(Tmp,"rotAboutZ=" << rotAboutZ);
    return rotAboutZ;
    */
  }

  return q[j-1];

}


Real SimulatedSerialManipulator::getJointVel(Int j) const
{
  Assertm((j>=1)&&(j<=chain.dof()), "joint index in range");

  if (!dynamic) return 0;

  if (chain[j-1].type() == KinematicChain::Link::Revolute) {

    ref<HingeJoint> hinge(narrow_ref<HingeJoint>(joints[j-1]));
    return hinge->getAngleRate();
  }
  else { // prismatic

    ref<SliderJoint> slider(narrow_ref<SliderJoint>(joints[j-1]));
    return slider->getPositionRate();
  }

}






array<base::Dimension3> SimulatedSerialManipulator::computeLinkDimensions(const array<Real>& linkRadii, Int firstLinkIndex)
{
  // call super and then add the dim of the base link as link 0
  array<Dimension3> dim( SimulatedKinematicChain::computeLinkDimensions(linkRadii, firstLinkIndex) );

  Matrix4 baseTransform(manipulatorDescr->getBaseTransform());
  Real len = baseTransform.column(4).toVector3().length();

  linkLengths[0] = len;

  Real zl=len*0.98;
  if (Math::equals(len,0)) zl=0.05; // some links are zero length so make them very small instead

  Real yz;
  if (linkRadii.size() >= 1)
    yz = linkRadii[0]*2.0;
  else
    yz = Math::maximum(0.05,zl*0.14); // make x&y-dims smaller than length (z-dim)

  dim[0] = Dimension3(yz,yz,zl);

  return dim;
}





SimulatedSerialManipulator::TransformInfo SimulatedSerialManipulator::computeLinkTransforms(const Transform& mountTransform, const base::Vector& q) const
{
  TransformInfo transformInfo;
  transformInfo.mountTransform = mountTransform;
  transformInfo.q.reset(q);

  array<Transform>& A(transformInfo.A); // convenient aliases
  array<Transform>& T(transformInfo.T);
  array<Transform>& SLT(transformInfo.SLT);
  A.resize(chain.size()+1);
  T.resize(A.size());
  SLT.resize(A.size());

  Transform baseTransform(manipulatorDescr->getBaseTransform());

  A[0] = transformInfo.mountTransform * baseTransform; // link 0
  T[0] = A[0];
  Int j=0; // joint variable index

  // handle base link (i.e. link joining mount to first joint origin) as a special case first

  SLT[0] = Transform(Vector3(0,0,-linkLengths[0]/2.0));
  // for the base, we want the Solid to have it's z-axis co-incident with
  //  the line joining the mount origin and the first joint's origin
  // (so we construct a rotation that rotates the z-axis of the base Solid
  //  into the joining line)
  Vector3 localZAxis(Vector3(0,0,1)); // untransformed z-axis
  Vector3 baseOffset(baseTransform.getTranslation());
  if (!Math::equals(baseOffset.norm(),0)) {
    // mount and base origins are not co-incident, need to orient Solid
    Vector3 baseZAxis( baseOffset.normalize()); // vector we want the z-axis be coincide with
    Vector3 rotAxis = cross(localZAxis, baseZAxis); // rotate about this vector
    if (!rotAxis.equals(Vector3(0,0,0))) { // not co-linear, transformation necessary
      Real rotAngle = Math::acos(dot(localZAxis, baseZAxis)); // by this amount
      //!!! do we need to account for when baseZAxis.y < 0 (and do rotAngle=360-rotAngle)??!!!
      Transform R = Quat4(rotAxis, rotAngle);
      SLT[0] = R * SLT[0];
    }
  }

  transformInfo.mountToBaseSolid = inverse(SLT[0]);



  // now the rest of the links
  for (Int l=1; l<=chain.size(); l++) { // for each link

    const KinematicChain::Link& link(chain[l-1]);

    //
    // ask KinematicChain to compute the transform for each link
    Vector lq(link.dof());
    if (lq.size()>0)
      lq[0] = transformInfo.q[j]; // this currently only handles 1-dof links (!!!)
    A[l] = base::toMatrix4(link.kinematicTransform(lq));

    //
    // accumulate inter-link transforms
    T[l] = T[l-1]*A[l];

    //
    // finally compute the relative configuration of each link's Solid from its origin

    // Rotate the z-axis into the x-axis, as the link coord. frame has the x-axis along
    //  the link length, but the Solid's z-axis is the long one.
    // Also translate back 1/2 the Solid length along the long axis (z)
    //  (as the origin of the Solid is at its center of mass, but the origin of a link
    //   is at its end)
    SLT[l] = Quat4(Vector3(0,1,0),consts::Pi/2.0) * Transform(Vector3(0,0,-linkLengths[l]/2.0));

    if (link.isDHType()) {
//!!! fix comment & and make analogous changes (re linkLength==0) to SimulatedTool if necessary!!!
      // We also want to rotate the Solid so that the ends meet even if there
      //  is a displacement due to DH-parameter d
      //  (this will be a rotation about the y-axis of link l-1 to make the Solid's z-axis
      //   point to the previous link's origin rather than being parallel with
      //   the link x-axis.  To achieve this we bring link Solid l's y&z-axes parallel to
      //   those of link l-1 by reversing the skew DH-parameter alpha, then rotate about the x-axis,
      //   and then redo the skew)
      Real d = (link.type() == KinematicChain::Link::Prismatic)?lq[0]:link.getD();
      Real incline = ( !Math::equals(d,0) && !Math::equals(linkLengths[l],0) )?
                                                                Math::asin(d/linkLengths[l]) : 0;
      if (!Math::equals(incline,0)) {
        Transform rotYl    = Quat4(Vector3(0,1,0),-incline);
        Transform rotXl    = Quat4(Vector3(1,0,0), link.getAlpha());
        Transform rotXlInv = Quat4(Vector3(1,0,0),-link.getAlpha());
        Transform rotYlm1 = rotXlInv * rotYl * rotXl;

        SLT[l] = rotYlm1 * SLT[l];
      }

    }

    if (l==chain.size())
      transformInfo.eeToEESolid = inverse(SLT[l]); //!!! check

    j += link.dof();
  } // for each link


  return transformInfo;
}





void SimulatedSerialManipulator::createLinks(const array<base::Dimension3>& linkDims)
{
  // We create a Solid for each link and place it in SolidSystem
  // Unless the geometry is specified, the Shape will be a Box who's dimensions
  // are specified in linkDims[]
  // The links are also chained together using SpatialGroups
  Assert(chain.size() > 0);

  links.resize(chain.size()+1);

  for (Int l=0; l<=chain.size(); l++) { // for each link
    const Dimension3& dim(linkDims[l]);
    //ref<Box> box(NewObj Box(dim.x,dim.y,dim.z));
    ref<Capsule> cap(NewObj Capsule(dim.z, dim.x/2.0));
    ref<Material> material(NewObj physics::Material());

    // colour
    switch (l%10) {
    case 0: material->setBaseColor(gfx::Color3(0.6,0.6,0.6)); break;
    case 1: material->setBaseColor(gfx::Color3(0,1,0)); break;
    case 2: material->setBaseColor(gfx::Color3(0+0.1,1-0.1,0+0.1)); break;
    case 3: material->setBaseColor(gfx::Color3(0+0.2,1-0.2,0+0.2)); break;
    case 4: material->setBaseColor(gfx::Color3(0+0.3,1-0.3,0+0.3)); break;
    case 5: material->setBaseColor(gfx::Color3(0+0.4,1-0.4,0+0.4)); break;
    case 6: material->setBaseColor(gfx::Color3(0+0.5,1-0.5,0+0.5)); break;
    case 7: material->setBaseColor(gfx::Color3(0+0.6,1-0.6,0+0.6)); break;
    case 8: material->setBaseColor(gfx::Color3(1,1,1)); break;
    case 9: material->setBaseColor(gfx::Color3(1,0,0)); break;
    default:
      material->setBaseColor(gfx::Color3(0,0,1));
    }

    // add it to the system
    links[l] = solidSystem->createSolid(cap,material);
    links[l]->setName(getManipulatorDescription()->getName()+" "
                      +String("link ")+base::intToString(l));
    solidSystem->addSolid(links[l]);

    // create a SpatialGroup which will hold a SpatialTransform warpping this link's Solid
    //  (as the link Solid may be transformed from the link origin), and the next link's
    //  group.
    ref<SpatialTransform> linkTransform( NewObj SpatialTransform() );
    linkTransform->setChild( links[l] ); // wrap link in a SpatialTransform
    ref<SpatialGroup> linkGroup( NewObj SpatialGroup() );
    linkGroup->add(linkTransform); // add it to the link group
    linkGroup->setImplicitConfiguration(linkTransform); // the configuration of the link group is synonymous with the linkTransform
    if (l > 0) linkGroups[l-1]->add(linkGroup); // add the linkGroup to the previous link's linkGroup
    linkGroups.push_back(linkGroup); // stash it into the link group array

    if (l == 0)
      baseSolid = links[l];
    else
      if (l == chain.size())
        endSolid = links[l];
  }
}



void SimulatedSerialManipulator::positionLinks(const SimulatedSerialManipulator::TransformInfo& transformInfo)
{
  Assert(links.size() == chain.size()+1); // links must already have been created

//  const Transform& mountTransform(transformInfo.mountTransform);

  mountToBaseSolid = transformInfo.mountToBaseSolid;
  eeToEESolid = transformInfo.eeToEESolid;


  // some convenient aliases
  const array<Transform>& SLT( transformInfo.SLT );   // transforms from the link's Solid coord. frame to the link frame
  const array<Transform>& T( transformInfo.T );

  //Transform baseTransform(manipulatorDescr.getBaseTransform());

  for (Int l=0; l<=chain.size(); l++) { // for each link

    ref<SpatialGroup> linkGroup(linkGroups[l]);
    SpatialGroup::iterator firstChild = linkGroup->begin();

    // set the transform from link origin to Solid origin first,
    // so that the Solid is properly configured when SpatialTransform configuration is set
    ref<SpatialTransform> linkSpatialTransform( narrow_ref<SpatialTransform>( *firstChild ) );
    linkSpatialTransform->setTransform( SLT[l] );

    // set SpatialTransform configuration (hence configuring its Solid child, and implicitly the linkGroup)
    linkSpatialTransform->setConfiguration( T[l] );

  } // for each link

}



void SimulatedSerialManipulator::disableCollisions(const array<ref<physics::Collidable> >& collidables,
                                                   const array<ref<physics::Collidable> >& proximityCollidables)
{
  Assert(solidSystem);
  Assert(links.size() == chain.size()+1); // links must already have been created

  ref<CollisionCuller> cc(solidSystem->getCollisionCuller());
  bool hasProxSensors = manipulatorDescr->hasLinkProximitySensors();

  bool disableNextWithPrev = false;   // if true, disabled collisions between link's l & l-2 (in addition to the
                                      // usual disable between l & l-1.  This occurs when link l-1 has a length of 0 (coincident origins)

  for (Int l=1; l<=chain.size(); l++) { // for each link
    // disable collision detection between consecutive links
    cc->collisionEnable(false,collidables[l-1],collidables[l]);

    // and between link Solid Collidables & their corresponding proximity sensor Collidable
    if (hasProxSensors) {
      cc->collisionEnable(false,collidables[l], proximityCollidables[l]);

      cc->collisionEnable(false,collidables[l-1],proximityCollidables[l]);
      cc->collisionEnable(false,proximityCollidables[l-1],collidables[l]);

//!!!debug reduce link-link prox hits
/*    if (l>=2 && l < proximityCollidables.size()-2) {
      cc->collisionEnable(false,collidables[l],proximityCollidables[l+1]);
      cc->collisionEnable(false,collidables[l],proximityCollidables[l+2]);
      cc->collisionEnable(false,collidables[l-1],proximityCollidables[l+1]);
      cc->collisionEnable(false,collidables[l-1],proximityCollidables[l+2]);
      cc->collisionEnable(false,collidables[l-2],proximityCollidables[l]);
      cc->collisionEnable(false,collidables[l-2],proximityCollidables[l+1]);
      cc->collisionEnable(false,collidables[l-2],proximityCollidables[l+2]);
    }*/
//!!!
//!!! debug - eliminate link-link prox hits
      for(Int pl=0; pl<proximityCollidables.size(); pl++)
        cc->collisionEnable(false,collidables[l], proximityCollidables[pl]);
//!!!
    }

    if (disableNextWithPrev) {
      cc->collisionEnable(false,collidables[l-2],collidables[l]);
      if (hasProxSensors) cc->collisionEnable(false,collidables[l-2],proximityCollidables[l]);
      disableNextWithPrev = false;
    }

    if (linkLengths[l]<=0.05)
       disableNextWithPrev = true; // don't collide link l+1 with l-1 as they will be close on account of this link l being 0 length

  }

//!!! eliminate base prox hits
if (hasProxSensors)
  for(Int pl=0; pl<proximityCollidables.size(); pl++)
    cc->collisionEnable(false,collidables[0], proximityCollidables[pl]);
//!!!
}


void SimulatedSerialManipulator::attachJoints(const SimulatedSerialManipulator::TransformInfo& transformInfo)
{
  if (links.size() < 2) return; // no joints

  // alias
  const array<Transform>& T( transformInfo.T );     // accumulated transformations for each link (transforms from frame l to base frame)
  const array<Transform>& SLT( transformInfo.SLT );

  ref<ConstraintGroup> cgroup = solidSystem->createConstraintGroup();
  solidSystem->addConstraintGroup(cgroup);
  ref<Motor> motor;
  joints.resize(links.size()-1);

  for (Int l=1; l<links.size(); l++) { // for each link (except the first)
    // connect it to the previous link in the chain

    const KinematicChain::Link& link(chain[l-1]);
    ref<Joint> joint;

    // Connect using a HingeJoint for revolute joints and a SliderJoint for prismatic joints
    if (link.isDHType()) {
//!!! check this is correct - is the link Zaxis == Solid -xaxis?? !!!
      // All DH type joints have an axis of Z - which is the Solid's -ve X-axis
      //  The axis is the z-axis of the previous link (l-1)
      // (they are specified relative to link l - hence we must transform z-axis
      //  of link l-1 into link l's coord. frame.)
      Vector3 axis( Vector3(-1,0,0) ); // untransformed Solid -ve x-axis (link z-axis)
      if (l>1) {
        axis = (T[l-1]*SLT[l-1]).rotate(axis);  // -ve x-axis of link Solid l-1
        axis = inverse(T[l]*SLT[l]).rotate(axis);  // -ve x-axis of link Solid l-1 in frame of link Solid l
      } else { // connection to base is special case
        axis = T[0].rotate(Vector3(0,0,1)); // z-axis in base frame
        axis = inverse(T[1]*SLT[1]).rotate(axis);  // z-axis of base frame in frame of link Solid l
      }


      if (link.type() == KinematicChain::Link::Revolute) {

        ref<HingeJoint> hinge = solidSystem->createHingeJoint();
        joint = hinge;
        cgroup->addConstraint(hinge);
        joints[l-1] = joint;
        Assert(links[l-1]);
        joint->attach(links[l], links[l-1]);

        // the anchor is at the end of the link (toward the base - not the origin end)
        hinge->setAnchor( Point3(0,0,-linkLengths[l]/2) );
        hinge->setAxis( axis );

        // set limits
        if (! ((link.minLimit() <= -consts::Pi)&&(link.maxLimit() >= consts::Pi)) ) {
          hinge->setLowStop(link.minLimit() - Math::degToRad(1) );
          hinge->setHighStop(link.maxLimit() + Math::degToRad(1) );
          hinge->setStopRestitution(0.2);
        }

        // now attach a Motor to drive the joint
        motor = solidSystem->createMotor();
        hinge->attachMotor(1, motor);

      }
      else if (link.type() == KinematicChain::Link::Prismatic) {

        ref<SliderJoint> slider = solidSystem->createSliderJoint();
        joint = slider;
        cgroup->addConstraint(slider);
        joints[l-1] = joint;
        Assert(links[l-1]);
        joint->attach(links[l], links[l-1]);

        slider->setAxis( axis );

        // set limits
        slider->setLowStop(link.minLimit());
        slider->setHighStop(link.maxLimit());
        slider->setStopRestitution(0.2);

        // now attach a Motor to drive the joint
        motor = solidSystem->createMotor();
        slider->attachMotor(1, motor);

        Logln("Warning: Prismatic joints not fully implemented");
      }
    }
    else
      throw std::invalid_argument(Exception("unknown/unsupported joint type: " + link.type()));

    motor->setTargetVel(0);
    motor->setMaxForce(0.001); // provide a little damping

  } // for each link

}



void SimulatedSerialManipulator::construct(const base::Point3& initialPosition,
                                           const base::Orient& initialOrientation)
{
  mountConfiguration = Transform(initialPosition, initialOrientation);

  linkGroups.clear();
  createLinks(computeLinkDimensions(manipulatorDescr->getLinkRadii()));


  // first construct the joint parameter vector q, based on the home positions of
  //  each joint represented in the chain
  Assert(chain.size() > 0);
  Vector q(chain.dof());

  // note: this only works for links with 1-dof (!!!)
  for(Int j=0; j<chain.dof(); j++) { // for each joint
    const KinematicChain::Link& link(chain.linkOfVariable(j)); // get link for this joint
    q[j] = link.variable();
  }

  // compute the various transform matrices we need
  TransformInfo transformInfo( computeLinkTransforms(mountConfiguration, q) );

  positionLinks(transformInfo);

  if (dynamic)
    attachJoints(transformInfo);

//!!! fix up this mess with CollisionCuller/Detector/ResponseHandler  !!!!
// (mostly in [ODE]SoliidSystem).  There is a need for a CollisionSystem to hold everything together
//   (including perhaps Collidables too)
//!!!

  if (!dynamic) {
    // replace the default CollisionResponseHandler with one that
    // doesn't create contacts
//!!! this doesn't seem to work - comment it out for now
//    ref<CollisionResponseHandler> nullHandler(NewObj NullCollisionResponseHandler(solidSystem->getCollisionDetector()));
//    solidSystem->setCollisionResponseHandler( nullHandler );
  }

  // now create a CollisionResponseHandler to handle collisions between
  //  the proximity CollidableBodies and oher objects and insert it into
  //  the collision chain before the final handler
  ref<ProximityCollisionResponseHandler> proximityHandler(NewObj ProximityCollisionResponseHandler(ref<SimulatedSerialManipulator>(this),
                                                                                                   solidSystem->getCollisionDetector() ));
  proximityHandler->addPotentialCollisionListener( solidSystem->getCollisionResponseHandler() );
  solidSystem->setCollisionResponseHandler( proximityHandler );

}




// Spatial

void SimulatedSerialManipulator::setPosition(const Point3& pos)
{
  Transform baseTransform(manipulatorDescr->getBaseTransform());
  mountConfiguration = inverse(baseTransform)*linkGroups[0]->getConfiguration();
  mountConfiguration.setTranslationComponent(pos);
  linkGroups[0]->setConfiguration(mountConfiguration);

  // re-position the tool at the ee-end, if necessary (static simulation only)
  if (!dynamic && toolGrasped) {
    Transform eeConfig( getEEPosition(), getEEOrientation() );
    proximityTool->setConfiguration( eeConfig );
  }

}

Point3 SimulatedSerialManipulator::getPosition() const
{
  Transform baseTransform(manipulatorDescr->getBaseTransform());
  mountConfiguration = inverse(baseTransform)*linkGroups[0]->getConfiguration();

  return mountConfiguration.getTranslation();
}

void SimulatedSerialManipulator::setOrientation(const Orient& orient)
{
  Transform baseTransform(manipulatorDescr->getBaseTransform());
  mountConfiguration = inverse(baseTransform)*linkGroups[0]->getConfiguration();
  mountConfiguration.setRotationComponent(orient);
  linkGroups[0]->setConfiguration(mountConfiguration);

  // re-position the tool at the ee-end, if necessary (static simulation only)
  if (!dynamic && toolGrasped) {
    Transform eeConfig( getEEPosition(), getEEOrientation() );
    proximityTool->setConfiguration( eeConfig );
  }
}

Orient SimulatedSerialManipulator::getOrientation() const
{
  Transform baseTransform(manipulatorDescr->getBaseTransform());
  mountConfiguration = inverse(baseTransform)*linkGroups[0]->getConfiguration();

  return mountConfiguration.getRotation();
}

void SimulatedSerialManipulator::setConfiguration(const base::Transform& configuration)
{
  Transform baseTransform(manipulatorDescr->getBaseTransform());
  mountConfiguration = configuration;
  linkGroups[0]->setConfiguration(mountConfiguration*baseTransform);

  // re-position the tool at the ee-end, if necessary (static simulation only)
  if (!dynamic && toolGrasped) {
    Transform eeConfig( getEEPosition(), getEEOrientation() );
    proximityTool->setConfiguration( eeConfig );
  }
}

base::Transform SimulatedSerialManipulator::getConfiguration() const
{
  Transform baseTransform(manipulatorDescr->getBaseTransform());
  mountConfiguration = inverse(baseTransform)*linkGroups[0]->getConfiguration();
  return mountConfiguration;
}




base::Point3 SimulatedSerialManipulator::getEEPosition() const
{
  // the end-effector frame is co-incident with the end of the ee solid

  // get the vector offset from ee origin to ee solid origin
  Vector3 eeToEESolidOffset( eeToEESolid.getTranslation() );
  // now express it in the world frame
  Vector3 eeToEESolidOffsetWorld( endSolid->getOrientation().rotate(eeToEESolidOffset) );
  // add it to the end solid origin to get the ee origin

  return endSolid->getPosition() + eeToEESolidOffsetWorld;
}


base::Orient SimulatedSerialManipulator::getEEOrientation() const
{
  Matrix3 eeToEESolidOrient(eeToEESolid.getRotation());
  return base::Orient(  endSolid->getOrientation() * Orient(eeToEESolidOrient).getQuat4() );
}



bool SimulatedSerialManipulator::checkProximity(ref<SimulatedTool> tool)
{
  if (toolGrasped)
    return (tool == proximityTool);

  // compare the end-effector position/orient with that of the tool
  //  If it is within tolerance, return true and record the tool in proximityTool
  Point3 eePos(getEEPosition());
  if ( (eePos- tool->getPosition()).length() < proximityDistance ) {
    Vector3 eeOrient(getEEOrientation().getVector3(Orient::EulerRPY));
    Vector3 toolOrient( tool->getOrientation().getVector3(Orient::EulerRPY));
    Vector3 eeAngDiff( Math::angleDifference(toolOrient.x,eeOrient.x),
                       Math::angleDifference(toolOrient.y,eeOrient.y),
                       Math::angleDifference(toolOrient.z,eeOrient.z) );
    if (    (Math::abs(eeAngDiff.x) < proximityAngle)
         && (Math::abs(eeAngDiff.y) < proximityAngle)
         && (Math::abs(eeAngDiff.z) < proximityAngle) ) {
      proximityTool = tool;
      return true;
    }

  }

  return false;
}




bool SimulatedSerialManipulator::graspTool()
{
  if (toolGrasped) return true; // tool already in grasp

  if (!proximityTool) return false; // no tool to grasp

  // recheck that the last tool indicated in proximity is still there
  if (!checkProximity(proximityTool)) return false; // no longer in proximity

  // first position the tool exactly in position/orient to be grasped
  proximityTool->setPositionOrientation( getEEPosition(), getEEOrientation() );

  // now attach it
  proximityTool->attachTo( endSolid );

  linkGroups[linkGroups.size()-1]->add(proximityTool);

  // disable collision detection between the first link of the tool and the ee
  //                                     the ee link and the tool first link sensor
  //                                     the ee link sensor and the tool first link
  ref<CollisionCuller> cc(solidSystem->getCollisionCuller());
  ref<Collidable> eeCollidable( collidables[collidables.size()-1] );
  ref<Collidable> eeProxCollidable( proximityCollidables[proximityCollidables.size()-1] );
  ref<Collidable> toolLinkCollidable( proximityTool->getFirstLinkCollidable() );
  ref<Collidable> toolLinkProxCollidable( proximityTool->getFirstLinkProximitySensorCollidable() );
  Assert(eeCollidable && eeProxCollidable && toolLinkCollidable && toolLinkProxCollidable);
  cc->collisionEnable(false, eeCollidable, toolLinkCollidable );
  cc->collisionEnable(false, eeProxCollidable, toolLinkCollidable );
  cc->collisionEnable(false, eeCollidable, toolLinkProxCollidable );
  toolGrasped = true;

  return true;
}


void SimulatedSerialManipulator::releaseGrasp()
{
  if (!toolGrasped) return;

  proximityTool->detatch();
  linkGroups[linkGroups.size()-1]->remove(proximityTool);

  // re-enable collisions between the first link of the tool and the ee
  Assert(solidSystem); Assert(collidables.size()>0);
  ref<CollisionCuller> cc(solidSystem->getCollisionCuller());
  ref<Collidable> eeCollidable( collidables[collidables.size()-1] );
  cc->collisionEnable(true, eeCollidable, proximityTool->getFirstLinkCollidable() );


  toolGrasped = false;
}



ref<physics::Collidable> SimulatedSerialManipulator::createCollidable(CollidableFlags flags)
{
  ref<CollisionCuller> cc(solidSystem->getCollisionCuller());
  ref<CollisionDetector> cd(solidSystem->getCollisionDetector());

  linkProximitySurfPosition.resize(links.size());

  collidables.resize(links.size()); // collidables for each link
  proximityCollidables.resize(links.size());  // proximity sensor for each link
  ref<CollidableGroup> linkCollidableGroup = cc->createCollidableGroup();
  linkCollidableGroup->setName(className()+" Links Group");
  proximityCollidableGroup = cc->createCollidableGroup();
  proximityCollidableGroup->setName(className()+" Link proximity sensors Group");

  const array<Real>& linkRadii( manipulatorDescr->getLinkRadii() );

  // create Collidables for each link (one that is connected to the Solid
  //  and optionally one for the proximity sensor)
  for(Int l=0; l<links.size(); l++) {
    // Collidable for link Solid
    collidables[l] = links[l]->createCollidable(flags);
    linkCollidableGroup->addCollidable( collidables[l] );
    if (l==0) collidables[l]->setName("baseLink");

    if (manipulatorDescr->hasLinkProximitySensors()) {

      // Collidable for proximity around each link

      // get link geometry info
      ref<const Shape> linkShape( links[l]->getShape() );
      Real linkHeight;
      Real linkRadius;
      if (manipulatorDescr->hasGeometry()) {
        linkHeight = narrow_ref<const Capsule>(linkShape)->height(); // capsule!!!
        linkRadius = linkRadii[l];
      }
      else {
        linkHeight = narrow_ref<const Capsule>(linkShape)->height(); // capsule!!!
        linkRadius = narrow_ref<const Capsule>(linkShape)->radius(); // capsule!!!
      }

      Real proxHeight = linkHeight;
//!!!! just for video
      Real proxRadius = /*linkRadius +*/ manipulatorDescr->linkProximitySensorRange();
      ref<const Capsule> collisionShape(NewObj Capsule(proxHeight, proxRadius));
      linkProximitySurfPosition[l] = linkRadius;

      ref<Collidable> pbody( links[l]->createCollidable(collisionShape, flags) );
      pbody->setInterpenetrationIsNormal(true); // the link solid is normally inside the proximity Collidable
      pbody->setUserClass(SensorCollidableClass); // cull sensor-sensor collision checks

      if (l!=0)
        pbody->setName( pbody->getName()+" Sensor" );
      else
        pbody->setName("baseLink Sensor");

//VisualDebugUtil::addDebugObject(linkShape, links[l]->getName(), links[l]->getConfiguration(),Color4("yellow",0.2));//!!!
//disable colision capsule display: VisualDebugUtil::addDebugObject(collisionShape, pbody->getName(), links[l]->getConfiguration(),Color4("lime green",0.15));//!!!
    // ref self so we can identify our Collidable quickly in ProximityCollisionResponseHandler
      pbody->setUserData(ref<SimulatedSerialManipulator>(this));
      proximityCollidables[l] = pbody;
      proximityCollidableGroup->addCollidable( proximityCollidables[l] );
    }

  }

  proximityCollidableGroup->setChildIntercollisionEnabled(false);
  proximityCollidableGroup->setUserClass(SensorCollidableClass);

  disableCollisions(collidables, proximityCollidables);

  ref<CollidableGroup> collidableGroup = cc->createCollidableGroup();
  collidableGroup->setName(className()+" collidables");
  collidableGroup->addCollidable( linkCollidableGroup );
  if (manipulatorDescr->hasLinkProximitySensors())
    collidableGroup->addCollidable( proximityCollidableGroup );

  return collidableGroup;
}



Real SimulatedSerialManipulator::getClosestObjectDistance(Int link) const
{
  if (linkProximity.size()>0) {
    Assert(link < linkProximity.size());
    return linkProximity[link].dist;
  }
  else
    return maxDist+1.0;
}

base::Vector3 SimulatedSerialManipulator::getClosestObjectDirection(Int link) const
{
  if (linkProximity.size()>0) {
    Assert(link < linkProximity.size());
    return linkProximity[link].dir;
  }
  else
    return Vector3();
}

Real SimulatedSerialManipulator::getClosestObjectSensorPosition(Int link) const
{
  if (linkProximity.size()>0) {
    Assert(link < linkProximity.size());
    return linkProximity[link].intersect;
  }
  else
    return 0;
}




void SimulatedSerialManipulator::handleCollision(ref<physics::CollisionState> collisionState)
{
  typedef SimulatedSerialManipulator::ProximityData ProximityData;

  // figure out proximity info for ControlInterface

  bool firstIsSensor = (collisionState->collidable1->getUserData() == ref<SimulatedSerialManipulator>(this));

  // find the corresponding link
  Int l=0;
  bool found = false;
  CollidableGroup::const_iterator sc( proximityCollidableGroup->begin() );
  CollidableGroup::const_iterator send( proximityCollidableGroup->end() );
  while ((sc != send) && !found) {
    if (*sc == (firstIsSensor?collisionState->collidable1:collisionState->collidable2) )
      found = true;
    else {
      ++sc; ++l;
    }
  }

//!!! handle base problem
if (l < 3) return;

  ref<const CollidableBody> body1( narrow_ref<const CollidableBody>(collisionState->collidable1) );
  ref<const CollidableBody> body2( narrow_ref<const CollidableBody>(collisionState->collidable2) );

  //!!!!
//  String name1(body1->getName());
//  String name2(body2->getName());
//  if ((name1.find("Multi") != String::npos) || (name2.find("Multi") != String::npos)) {
//    Debugcln(DJ,"SimSerManip::handleCollision():" << body1->getName() << " and " << body2->getName());
//  }
  //!!!!

  // now we compute the distance from the axis of the Solid (actually a segment
  //  running along the axis) and the obstacle
  ref<const Solid> linkSolid( narrow_ref<const Solid>( links[l] ) );
  Transform T( linkSolid->getConfiguration() );
  Real length( linkLengths[l] );
  Segment3 linkSeg( T*Point3(0,0,-length/2.0), T*Point3(0,0,length/2.0) );

  //  now determine distance between the obstacle and the link segment
  ref<const Shape> obstacleShape( (firstIsSensor?body2:body1)->getShape() );
  Transform obstacleT( (firstIsSensor?body2:body1)->getConfiguration() );
  // shortest degment between them
  Segment3 seg( obstacleShape->shortestSegmentBetween(obstacleT, linkSeg) );

  Real dist = seg.length();
  Vector3 direction( seg.s - seg.e ); if (dist > 0) direction.normalize();
  Real intersect = (seg.e - linkSeg.s).length();
  ProximityData proxData(dist, intersect,  direction);
  linkProximity[l] = proxData;


  if (!firstIsSensor) base::swap(body1, body2);

//!!! debug
Debugln(DJ,"proximity between " << body1->getName() << " and " << body2->getName());
//Debugcln(DJ,"seg between " << body1->getName() << " and " << body2->getName() << " is " << seg << " len=" << seg.length());

  // try to visualize the proximity data
  Transform c;
  c.setToTranslation( seg.s + 0.5*(seg.e-seg.s) );
  Matrix3 R; R.setIdentity();
  if (!seg.e.equals(seg.s)) {
    Vector3 z( seg.e - seg.s ); z.normalize();
    Vector3 x(1,0,0);
    if (x.equals(z) || x.equals(-z)) x = Vector3(0,1,0);
    if (x.equals(z) || x.equals(-z)) x = Vector3(0,0,1);
    if (x.equals(z) || x.equals(-z)) x = Vector3(1,1,0);
    Vector3 y( cross(x,z) ); y.normalize();
    x = cross( y,z ); x.normalize();
    R.setColumn(1,x);
    R.setColumn(2,y);
    R.setColumn(3,z);
  }
  c.setRotationComponent( R );
  VisualDebugUtil::addDebugCylinderObject(dist, 0.005, body1->getName()+"proxseg", c, gfx::Color4("yellow"));

/*
  // try to visualize the push-away direction
  {
    Real L = 0.00611899; // push away dist
    Point3 p(0.824874,1.36352,0.540932); // push away point
    Vector3 n(0.893771,0.448523,-0); // push away direction
    n.normalize();
    c = Transform();
    c.setToTranslation( p - 0.5*L*n);
    Matrix3 R; R.setIdentity();
    Vector3 z( n ); z.normalize();
    Vector3 x(1,0,0);
    if (x.equals(z) || x.equals(-z)) x = Vector3(0,1,0);
    if (x.equals(z) || x.equals(-z)) x = Vector3(0,0,1);
    if (x.equals(z) || x.equals(-z)) x = Vector3(1,1,0);
    Vector3 y( cross(x,z) ); y.normalize();
    x = cross( y,z ); x.normalize();
    R.setColumn(1,x);
    R.setColumn(2,y);
    R.setColumn(3,z);
    c.setRotationComponent( R );
    VisualDebugUtil::addDebugCapsuleObject(L, 0.01, "pushpoint1", c, gfx::Color4("cyan"));
  }
*/
//disable collision capsule red display:  VisualDebugUtil::setColor(body1->getName(), Color4("red",0.15));
//!!!
}

