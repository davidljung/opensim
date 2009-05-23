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

  $Id: SimulatedTool.cpp 1115 2004-09-27 22:09:45Z jungd $

****************************************************************************/

#include <robot/sim/SimulatedTool>

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
#include <physics/Capsule>
#include <physics/Solid>
#include <physics/ConstraintGroup>
#include <physics/FixedConstraint>
#include <physics/Joint>
#include <physics/HingeJoint>
#include <physics/SliderJoint>
#include <physics/Motor>
#include <physics/CollisionCuller>
#include <physics/CollisionDetector>
#include <physics/CollisionState>
#include <physics/NullCollisionResponseHandler>

#include <physics/VisualDebugUtil>

#include <robot/KinematicChain>


using robot::sim::SimulatedTool;

using base::Point3;
using base::Orient;
using base::Externalizer;
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
using physics::Capsule;
using physics::Material;
using physics::Solid;
using physics::SpatialGroup;
using physics::SpatialTransform;
using physics::ConstraintGroup;
using physics::FixedConstraint;
using physics::Joint;
using physics::HingeJoint;
using physics::SliderJoint;
using physics::Motor;

using physics::VisualDebugUtil;

using robot::KinematicChain;
using robot::sim::SimulatedKinematicChain;
using robot::sim::SimulatedTool;

//!!!
#include <physics/ODESolidConnectedCollidableBody>
using physics::SolidConnectedCollidableBody;
using physics::ODESolidConnectedCollidableBody;


#include <ode/ode.h>
#include <ode/rotation.h>


/*
SimulatedTool::SimulatedTool(ref<base::VFile> toolSpecification,
                             const base::Point3& initialPosition, const base::Orient& initialOrientation,
                             ref<physics::SolidSystem> solidSystem, bool dynamic)
  : SimulatedKinematicChain(solidSystem, dynamic), attached(false)
{
  Assert(solidSystem!=0);

  chain = toolDescr.getKinematicChain();
  q.reset(zeroVector(chain.dof()));
  spatialGroup = ref<SpatialGroup>(NewObj SpatialGroup()); // a group to put all the Solids in

  // read in supported formats
  if (toolSpecification->extension() == "xml") {

    // read in parameters
    try {
      Externalizer e(Externalizable::Input, toolSpecification);
      externalize(e,"xml",1.0);
    } catch (std::exception&) {
      throw std::invalid_argument(Exception("not a valid tool .xml file."));
    }

  }
  else
    throw std::invalid_argument(Exception("file format unsupported."));

  construct(initialPosition, initialOrientation);
}
*/

SimulatedTool::SimulatedTool(ref<const robot::ToolDescription> toolDescription,
                             const base::Point3& initialPosition, const base::Orient& initialOrientation,
                             ref<physics::SolidSystem> solidSystem, bool dynamic)
  : SimulatedKinematicChain(solidSystem, dynamic), attached(false)
{
  Assert(solidSystem!=0);

  if (instanceof(*toolDescription, const SimulatedToolDescription))
    toolDescr = narrow_ref<const SimulatedToolDescription>(toolDescription);
  else
    toolDescr = ref<SimulatedToolDescription>(NewObj SimulatedToolDescription(*toolDescription));


  chain = toolDescr->getKinematicChain();
  q.resize(chain.dof());
  spatialGroup = ref<SpatialGroup>(NewObj SpatialGroup()); // a group to put all the Solids in

  construct(initialPosition, initialOrientation);
}




bool SimulatedTool::formatSupported(String format, Real version, ExternalizationType type) const
{
  return false;
}

void SimulatedTool::externalize(base::Externalizer& e, String format, Real version)
{
  if (format == "") format = "xml";

  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" v"+base::realToString(version)+" unsupported"));

}



void SimulatedTool::setJointForce(Int j, Real f)
{
  Assertm((j>=1)&&(j<=chain.dof()), "joint index in range");

  if (!dynamic) return;

  Int l = chain.linkIndexOfVariable(j-1);

  ref<Motor> motor(joints[l]->getMotor(1));
  motor->setTargetVel( (f>0)?10:-10 );
  motor->setMaxForce(f);
}

void SimulatedTool::setJointVel(Int j, Real v, Real maxForce)
{
  Assertm((j>=1)&&(j<=chain.dof()), "joint index in range");

  if (!dynamic) return;

  Int l = chain.linkIndexOfVariable(j-1);

  ref<Motor> motor(joints[l]->getMotor(1));
  motor->setTargetVel(v);
  motor->setMaxForce(maxForce);
}


void SimulatedTool::setJointPos(Int j, Real p)
{
  Assertm((j>=1)&&(j<=chain.dof()), "joint index in range");

  q[j-1] = p;

  //... actually move the links
  //hack!!! (expensive - only need to update the joint in question)

  Transform mountConfiguration = mountSpatial->getConfiguration();

  TransformInfo transformInfo( computeLinkTransforms(mountConfiguration, q) );
  positionLinks(transformInfo);

  //hack!!!
}


Real SimulatedTool::getJointPos(Int j) const
{
  Assertm((j>=1)&&(j<=chain.dof()), "joint index in range");

  if (dynamic) { // get joint pos from physics::Joint

    // NB: physics::Joints have their 0 pos at the position/orientation
    //  at which the two bodies were initially attached.  That was
    //  computed from the tool joint parameter home position.
    Int l = chain.linkIndexOfVariable(j-1);

    if (chain[l].type() == KinematicChain::Link::Revolute) {

      ref<HingeJoint> hinge(narrow_ref<HingeJoint>(joints[l]));

      q[j-1] = hinge->getAngle();

    } else { // prismatic
      ref<SliderJoint> slider(narrow_ref<SliderJoint>(joints[l]));
      q[j-1] = slider->getPosition();
    }

  } else { // static
    /*// !!!fix indices for tool (copied from manip)
    Transform relConfig = inverse( links[j]->getConfiguration() )*links[j-1]->getConfiguration();
    Real rotAboutZ = Math::normalizeAngle(relConfig.getRotation().getVector(Orient::EulerZYX)[0]);
    Debugln(Tmp,"rotAboutZ=" << rotAboutZ);
    return rotAboutZ;
    */
  }

  return q[j-1];

}


Real SimulatedTool::getJointVel(Int j) const
{
  Assertm((j>=1)&&(j<=chain.dof()), "joint index in range");

  if (!dynamic) return 0;

  Int l = chain.linkIndexOfVariable(j-1);

  if (chain[l].type() == KinematicChain::Link::Revolute) {

    ref<HingeJoint> hinge(narrow_ref<HingeJoint>(joints[l]));
    return hinge->getAngleRate();
  }
  else { // prismatic

    ref<SliderJoint> slider(narrow_ref<SliderJoint>(joints[l]));
    return slider->getPositionRate();
  }

}










SimulatedTool::TransformInfo SimulatedTool::computeLinkTransforms(const Transform& mountTransform, const base::Vector& q) const
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

  A[0] = transformInfo.mountTransform; // link 0
  T[0] = A[0];
  Int j=0; // joint variable index

  // transform from mount point to first link - translate origin to (other) link end
  transformInfo.mountToBaseSolid = Transform(Vector3(linkLengths[1],0,0));

  for (Int l=1; l<=chain.size(); l++) { // for each link

    const KinematicChain::Link& link(chain[l-1]);

    //
    // ask KinematicChain to compute the transform for each link
    Vector lq(link.dof());
    if (lq.size()>0) // this currently only handles 0 or 1-dof links (!!!)
      lq[0] = transformInfo.q[j];
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
//!!! fix comment
      // We also want to rotate the Solid so that the ends meet even if there
      //  is a displacement due to DH-parameter d
      //  (this will be a rotation about the y-axis of link l-1 to make the Solid's z-axis
      //   point to the previous link's origin rather than being parallel with
      //   the link x-axis.  To achieve this we bring link Solid l's y&z-axes parallel to
      //   those of link l-1 by reversing the skew DH-parameter alpha, then rotate about the x-axis,
      //   and then redo the skew)
//!!!check use of link.d is OK here (need to use q[0] if Prismatic??)
      Real incline = (!Math::equals(link.getD(),0))? Math::asin(link.getD()/linkLengths[l]) : 0;
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





void SimulatedTool::createLinks(const array<base::Dimension3>& linkDims)
{
  // We create a Solid for each link and place it in SolidSystem
  // Unless the geometry is specified, the Shape will be a Box who's dimensions
  // are specified in linkDims[]
  // The links are also chained together using SpatialGroups
  Assert(chain.size() > 0);

  links.resize(chain.size()+1);
  linkGroups.resize(chain.size()+1);


  for (Int l=1; l<=chain.size(); l++) { // for each link
    const Dimension3& dim(linkDims[l]);
    ref<Capsule> cap(NewObj Capsule(dim.z,dim.x/2.0));
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
    links[l]->setName(getToolDescription()->getName()+" "
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
    if (l > 1) linkGroups[l-1]->add(linkGroup); // add the linkGroup to the previous link's linkGroup
    linkGroups[l] = linkGroup; // stash it into the link group array

    if (l == 1)
      firstSolid = links[l];
    else
      if (l == chain.size())
        endSolid = links[l];
  }


  // the mount point is a SpatialTransform that wraps the first link such that the mount is at the
  //  appropriate point (opposite end to the end-effector)
  mountSpatial = ref<SpatialTransform>( NewObj SpatialTransform() );
  mountSpatial->setChild( linkGroups[1] );

}



void SimulatedTool::positionLinks(const SimulatedTool::TransformInfo& transformInfo)
{
  Assert(links.size() == chain.size()+1); // links must already have been created

  // set mount transform
  firstLinkSolidToMount = transformInfo.mountToBaseSolid;
  mountSpatial->setTransform(firstLinkSolidToMount);

  // some convenient aliases
  const array<Transform>& SLT( transformInfo.SLT );   // transforms from the link's Solid coord. frame to the link frame
  const array<Transform>& T( transformInfo.T );

  for (Int l=1; l<=chain.size(); l++) { // for each link

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



void SimulatedTool::disableCollisions(const array<ref<physics::Collidable> >& collidables,
                                      const array<ref<physics::Collidable> >& proximityCollidables)
{
  Assert(solidSystem);
  Assert(links.size() == chain.size()+1); // links must already have been created

  ref<CollisionCuller> cc(solidSystem->getCollisionCuller());

  bool disableNextWithPrev = false;   // if true, disabled collisions between link's l & l-2 (in addition to the
                                      // usual disable between l & l-1.  This occurs when link l-1 has a length of 0 (coincident origins)

  for (Int l=2; l<=chain.size(); l++) { // for each link
    // disable collision detection between consecutive links
    cc->collisionEnable(false,collidables[l-1],collidables[l]);

    // and between link Solid Collidables & their corresponding proximity sensor Collidable
    cc->collisionEnable(false,collidables[l], proximityCollidables[l]);

    cc->collisionEnable(false,collidables[l-1],proximityCollidables[l]);
    cc->collisionEnable(false,proximityCollidables[l-1],collidables[l]);


    if (disableNextWithPrev) {
      cc->collisionEnable(false,collidables[l-2],collidables[l]);
      cc->collisionEnable(false,collidables[l-2],proximityCollidables[l]);
      disableNextWithPrev = false;
    }

    if (linkLengths[l]<=0.05)
       disableNextWithPrev = true; // don't collide link l+1 with l-1 as they will be close on account of this link l being 0 length

  }


}


void SimulatedTool::attachJoints(const SimulatedTool::TransformInfo& transformInfo)
{
  if (links.size() < 2) return; // no joints

  // alias
  const array<Transform>& T( transformInfo.T );     // accumulated transformations for each link (transforms from frame l to base frame)
  const array<Transform>& SLT( transformInfo.SLT );

  ref<ConstraintGroup> cgroup = solidSystem->createConstraintGroup();
  solidSystem->addConstraintGroup(cgroup);
  ref<Motor> motor;
  joints.resize(links.size()-1); // should this be link.size()???!!!

  for (Int l=2; l<links.size(); l++) { // for each link (except the first)
    // connect it to the previous link in the chain

    const KinematicChain::Link& link(chain[l-1]);
    ref<Joint> joint;

    if (link.isDHType()) {    // Connect using a HingeJoint for revolute joints and a SliderJoint for prismatic joints

      // All DH type joints have an axis of Z - which is the Solid's X-axis
      //  The axis is the z-axis of the previous link (l-1)
      // (they are specified relative to link l - hence we must transform z-axis
      //  of link l-1 into link l's coord. frame.)
      Vector3 axis( Vector3(1,0,0) ); // untransformed Solid x-axis (link z-axis)
      if (l>1) {
        axis = (T[l-1]*SLT[l-1]).rotate(axis);  // x-axis of link Solid l-1
        axis = inverse(T[l]*SLT[l]).rotate(axis);  // x-axis of link Solid l-1 in frame of link Solid l
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



void SimulatedTool::construct(const base::Point3& initialPosition,
                              const base::Orient& initialOrientation)
{
  Transform mountConfiguration = Transform(initialPosition, initialOrientation);

  linkGroups.clear();
  createLinks(computeLinkDimensions( toolDescr->getLinkRadii(), 0 ));


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

  if (!dynamic) {
//!!! appropriate place for this?? (note another one in SimulatedSerialManipulator)
    // replace the default CollisionResponseHandler with one that
    // doesn't create contacts
    ref<CollisionResponseHandler> nullHandler(NewObj NullCollisionResponseHandler(solidSystem->getCollisionDetector()));
    solidSystem->setCollisionResponseHandler( nullHandler );
  }

  // now create a CollisionResponseHandler to handle collisions between
  //  the proximity CollidableBodies and oher objects and insert it into
  //  the collision chain before the final handler
  ref<ProximityCollisionResponseHandler> proximityHandler(NewObj ProximityCollisionResponseHandler(ref<SimulatedTool>(this),
                                                                                                   solidSystem->getCollisionDetector() ));
  proximityHandler->addPotentialCollisionListener( solidSystem->getCollisionResponseHandler() );
  solidSystem->setCollisionResponseHandler( proximityHandler );

}




// Spatial

void SimulatedTool::setPosition(const Point3& pos)
{
  Transform mountConfiguration = mountSpatial->getConfiguration();
  mountConfiguration.setTranslationComponent(pos);
  mountSpatial->setConfiguration(mountConfiguration);
}

Point3 SimulatedTool::getPosition() const
{
  Transform mountConfiguration = mountSpatial->getConfiguration();
  return mountConfiguration.getTranslation();
}

void SimulatedTool::setOrientation(const Orient& orient)
{
  Transform mountConfiguration = mountSpatial->getConfiguration();
  mountConfiguration.setRotationComponent(orient);
  mountSpatial->setConfiguration(mountConfiguration);
}

Orient SimulatedTool::getOrientation() const
{
  Transform mountConfiguration = mountSpatial->getConfiguration();
  return mountConfiguration.getRotation();
}

void SimulatedTool::setConfiguration(const base::Transform& configuration)
{
  Transform mountConfiguration = configuration;
  mountSpatial->setConfiguration(mountConfiguration);
}

base::Transform SimulatedTool::getConfiguration() const
{
  Transform mountConfiguration = mountSpatial->getConfiguration();
  return mountConfiguration;
}





void SimulatedTool::attachTo(ref<physics::Solid> manipEESolid)
{
  KinematicChain::Link link( chain[0] ); // first joint params

  Assert(firstSolid);
  Assert(manipEESolid);

  if (dynamic) {

    // attach using a FixedConstraint
    mountConstraintGroup = solidSystem->createConstraintGroup();
    Assert(!link.isActive());
    ref<FixedConstraint> fixed = solidSystem->createFixedConstraint();
    mountConstraintGroup->addConstraint(fixed);
    links[0] = manipEESolid;
    fixed->attach(manipEESolid, firstSolid);

  } // if dynamic

  attached = true;
}



void SimulatedTool::detatch()
{
  if (attached) {
    const KinematicChain::Link& link(chain[0]);

    if (dynamic) {

      Assert(!link.isActive());

      mountConstraintGroup->clear();
      mountConstraintGroup = ref<ConstraintGroup>(0);

    }

    attached=false;
  }

}







/*
base::Point3 SimulatedTool::getEEPosition() const
{
  // the end-effector frame is co-incident with the end of the ee solid

  // get the vector offset from ee origin to ee solid origin
  Vector3 eeToEESolidOffset( eeToEESolid.getTranslation() );
  // now express it in the world frame
  Vector3 eeToEESolidOffsetWorld( endSolid->getOrientation().rotate(eeToEESolidOffset) );
  // add it to the end solid origin to get the ee origin

  return endSolid->getPosition() + eeToEESolidOffsetWorld;
}


base::Orient SimulatedTool::getEEOrientation() const
{
  Matrix3 eeToEESolidOrient(eeToEESolid.getRotation());
  return base::Orient(  endSolid->getOrientation() * Orient(eeToEESolidOrient).getQuat4() );
}
*/


ref<physics::Collidable> SimulatedTool::createCollidable(CollidableFlags flags)
{
  Assert(solidSystem);
  ref<CollisionCuller> cc(solidSystem->getCollisionCuller());
  ref<CollisionDetector> cd(solidSystem->getCollisionDetector());
  Assert(cc); Assert(cd);

  linkProximitySurfPosition.resize(links.size());

  collidables.resize(links.size()); // collidables for each link
  proximityCollidables.resize(links.size());  // proximity sensor for each link
  ref<CollidableGroup> linkCollidableGroup = cc->createCollidableGroup();
  linkCollidableGroup->setName(className()+"Links Group");
  proximityCollidableGroup = cc->createCollidableGroup();
  proximityCollidableGroup->setName(className()+" Link proximity sensors Group");

  const array<Real>& linkRadii( toolDescr->getLinkRadii() );

  // create Collidables for each link (one that is connected to the Solid
  //  and optionally one for the proximity sensor)
  for(Int l=1; l<=chain.size(); l++) {
    // Collidable for link Solid
    Assert(links[l]);
    collidables[l] = links[l]->createCollidable(flags);
    linkCollidableGroup->addCollidable( collidables[l] );
    if (l==1) collidables[l]->setName("firstLink");

    // Collidable for proximity around each link
    ref<const Shape> linkShape( links[l]->getShape() );
    Real linkHeight;
    Real linkRadius;
    if (toolDescr->hasGeometry()) {
      linkHeight = narrow_ref<const Capsule>(linkShape)->height(); // capsule!!!
      linkRadius = linkRadii[l-1];
    }
    else {
      linkHeight = narrow_ref<const Capsule>(linkShape)->height(); // capsule!!!
      linkRadius = narrow_ref<const Capsule>(linkShape)->radius(); // capsule!!!
    }


    Real proxHeight = linkHeight;

//!!!! just for video
    Real proxRadius = /*linkRadius +*/ toolDescr->linkProximitySensorRange();
    ref<const Capsule> collisionShape(NewObj Capsule(proxHeight, proxRadius));
    linkProximitySurfPosition[l] = linkRadius;

    ref<Collidable> pbody( links[l]->createCollidable(collisionShape, flags) );
    pbody->setInterpenetrationIsNormal(true); // the link solid is normally inside the proximity Collidable
    pbody->setUserClass(SensorCollidableClass); // cull sensor-sensor collision checks

    if (l!=1)
      pbody->setName( pbody->getName()+" Sensor" );
    else
      pbody->setName("firstLink Sensor");
//disable colision capsule display: VisualDebugUtil::addDebugObject(collisionShape, pbody->getName(), links[l]->getConfiguration(),Color4("lime green",0.2));//!!!
//VisualDebugUtil::addDebugObject(linkShape, links[l]->getName(), links[l]->getConfiguration(),Color4("yellow",0.2));//!!!

    // ref self so we can identify our Collidable quickly in ProximityCollisionResponseHandler
    pbody->setUserData(ref<SimulatedTool>(this));
    proximityCollidables[l] = pbody;
    proximityCollidableGroup->addCollidable( proximityCollidables[l] );

  }

  proximityCollidableGroup->setChildIntercollisionEnabled(false);
  proximityCollidableGroup->setUserClass(SensorCollidableClass);

  disableCollisions(collidables, proximityCollidables);

  ref<CollidableGroup> collidableGroup = cc->createCollidableGroup();
  collidableGroup->setName(className()+" collidables");
  collidableGroup->addCollidable( linkCollidableGroup );
  collidableGroup->addCollidable( proximityCollidableGroup );

  return collidableGroup;
}



Real SimulatedTool::getClosestObjectDistance(Int link) const
{
//  Debugln(Tmp,"here lp.size=" << linkProximity.size());
  if (linkProximity.size()>0) {
    Assert(link < linkProximity.size());
    return linkProximity[link].dist;
  }
  else
    return maxDist+1.0;
}

base::Vector3 SimulatedTool::getClosestObjectDirection(Int link) const
{
  if (linkProximity.size()>0) {
    Assert(link < linkProximity.size());
    return linkProximity[link].dir;
  }
  else
    return Vector3();
}

Real SimulatedTool::getClosestObjectSensorPosition(Int link) const
{
  if (linkProximity.size()>0) {
    Assert(link < linkProximity.size());
    return linkProximity[link].intersect;
  }
  else
    return 0;
}




void SimulatedTool::handleCollision(ref<physics::CollisionState> collisionState)
{
  typedef SimulatedTool::ProximityData ProximityData;

  // figure out proximity info for ControlInterface

  bool firstIsSensor = (collisionState->collidable1->getUserData() == ref<SimulatedTool>(this));

  // find the corresponding link
  Int l=1;
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

  ref<const CollidableBody> body1( narrow_ref<const CollidableBody>(collisionState->collidable1) );
  ref<const CollidableBody> body2( narrow_ref<const CollidableBody>(collisionState->collidable2) );
//!!!Debugln(DJ,"collision: " << body1->getName() << " and " << body2->getName());


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
  Vector3 direction( seg.e - seg.s ); if (dist > 0) direction.normalize();
  Real intersect = (seg.e - linkSeg.s).length();
  ProximityData proxData(dist, intersect,  direction);
  linkProximity[l] = proxData;


  if (!firstIsSensor) base::swap(body1, body2);

//!!! debug
  VisualDebugUtil::setColor(body1->getName(), Color4("red",0.4));

}

