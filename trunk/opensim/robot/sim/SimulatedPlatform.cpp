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

  $Id: SimulatedPlatform.cpp 1102 2004-09-27 21:55:35Z jungd $
  $Revision: 1.12 $
  $Date: 2004-09-27 17:55:35 -0400 (Mon, 27 Sep 2004) $
  $Author: jungd $

****************************************************************************/

#include <robot/sim/SimulatedPlatform>

#include <base/Vector3>
#include <base/PathName>
#include <base/Externalizer>

#include <gfx/Color3>
#include <physics/Material>
#include <physics/Box>
#include <physics/Cylinder>
#include <physics/Solid>
#include <physics/HingeJoint>
#include <physics/DoubleHingeJoint>
#include <physics/Motor>
#include <physics/CollisionCuller>
#include <physics/CollisionDetector>


using robot::sim::SimulatedPlatform;

using base::Point3;
using base::Orient;
using base::PathName;
using physics::Box;
using physics::Cylinder;
using physics::ConstraintGroup;
using physics::SpatialGroup;
using physics::Solid;
using physics::HingeJoint;
using physics::DoubleHingeJoint;
using physics::Motor;
using physics::Collidable;
using physics::CollidableGroup;
using physics::CollisionCuller;
using physics::CollisionDetector;




SimulatedPlatform::SimulatedPlatform(ref<const robot::PlatformDescription> platformDescription, ref<physics::SolidSystem> solidSystem, bool dynamic)
  : solidSystem(solidSystem), platformDescr(platformDescription), dynamic(true)
{
  spatialGroup = ref<SpatialGroup>(NewObj SpatialGroup()); // a group to put all the Solids in
  platformSolids.resize(1);
  platformSolids[Platform] = ref<Solid>(0);
}


SimulatedPlatform::~SimulatedPlatform()
{
  if (platformSolids[Platform] && platformDescr->isMobile() && !platformDescr->isHolonomic()) {
    if (dynamic) {
      leftDriveHingeJoint->attachMotor(1,ref<Motor>(0));
      rightDriveHingeJoint->attachMotor(1,ref<Motor>(0));
      steeringHingeJoint->attachMotor(1,ref<Motor>(0));
      leftDriveHingeJoint = rightDriveHingeJoint = steeringHingeJoint = ref<HingeJoint>(0);
      wheelConstraintGroup->clear();
    }
  }
}



void SimulatedPlatform::setLeftBackWheelTorque(Real t)
{
  if (!dynamic) return;
  if (platformSolids[Platform] && platformDescr->isMobile() && !platformDescr->isHolonomic()) {
    leftDriveMotor->setMaxForce(t);
    leftDriveMotor->setTargetVel( (t>0)?10:-10 );
  }
}


void SimulatedPlatform::setLeftBackWheelVel(Real v, Real maxTorque)
{
  if (!dynamic) return;
  if (platformSolids[Platform] && platformDescr->isMobile() && !platformDescr->isHolonomic()) {
    leftDriveMotor->setTargetVel(v);
    leftDriveMotor->setMaxForce(maxTorque);
  }
}

void SimulatedPlatform::setRightBackWheelTorque(Real t)
{
  if (!dynamic) return;
  if (platformSolids[Platform] && platformDescr->isMobile() && !platformDescr->isHolonomic()) {
    rightDriveMotor->setMaxForce(t);
    rightDriveMotor->setTargetVel( (t>0)?10:-10 );
  }
}


void SimulatedPlatform::setRightBackWheelVel(Real v, Real maxTorque)
{
  if (!dynamic) return;
  if (platformSolids[Platform] && platformDescr->isMobile() && !platformDescr->isHolonomic()) {
    rightDriveMotor->setTargetVel(v);
    rightDriveMotor->setMaxForce(maxTorque);
  }
}


void SimulatedPlatform::setSteeringTorque(Real t)
{
  if (!dynamic) return;
  if (platformSolids[Platform] && platformDescr->isMobile() && !platformDescr->isHolonomic()) {
    steeringMotor->setMaxForce(t);
    steeringMotor->setTargetVel( (t>0)?10:-10 );
  }
}


void SimulatedPlatform::setSteeringVel(Real v, Real maxTorque)
{
  if (!dynamic) return;
  if (platformSolids[Platform] && platformDescr->isMobile() && !platformDescr->isHolonomic()) {
    steeringMotor->setTargetVel(v);
    steeringMotor->setMaxForce(maxTorque);
  }
}


Real SimulatedPlatform::getLeftBackWheelVel() const
{
  if (!dynamic) return 0;
  if (platformSolids[Platform] && platformDescr->isMobile() && !platformDescr->isHolonomic()) {
    return leftDriveHingeJoint->getAngleRate();
  }
  return 0;
}

Real SimulatedPlatform::getRightBackWheelVel() const
{
  if (!dynamic) return 0;
  if (platformSolids[Platform] && platformDescr->isMobile() && !platformDescr->isHolonomic()) {
    return rightDriveHingeJoint->getAngleRate();
  }
  return 0;
}


Real SimulatedPlatform::getSteeringAngle() const
{
  if (!dynamic) return 0; //!!! fix me (or not? this isn't a 'position' control, is it?)
  if (platformSolids[Platform] && platformDescr->isMobile() && !platformDescr->isHolonomic()) {
    return steeringHingeJoint->getAngle();
  }
  return 0;
}





// Spatial

void SimulatedPlatform::setPosition(const Point3& pos)
{
  spatialGroup->setPosition(pos);
}

Point3 SimulatedPlatform::getPosition() const
{
  return spatialGroup->getPosition();
}

void SimulatedPlatform::setOrientation(const Orient& orient)
{
  spatialGroup->setOrientation(orient);
}

Orient SimulatedPlatform::getOrientation() const
{
  return spatialGroup->getOrientation();
}

void SimulatedPlatform::setConfiguration(const base::Transform& configuration)
{
  spatialGroup->setConfiguration(configuration);
}

base::Transform SimulatedPlatform::getConfiguration() const
{
  return spatialGroup->getConfiguration();
}









void SimulatedPlatform::construct(const base::Point3& initialPosition, const base::Orient& initialOrientation)
{
  Assert(solidSystem);
  spatialGroup->clear();
  spatialGroup->setPositionOrientation(initialPosition, initialOrientation);

  ref<const robot::PlatformDescription> pd(platformDescr);
  platformSolids.resize(pd->isMobile()?MaxPlatformSolids:1);

  // make a platform Solid
  base::Dimension3 dim(pd->dimensions());

  ref<Box> pBox(NewObj Box(dim.x,dim.y,dim.z));
  ref<physics::Material> pMat(new physics::Material());
  pMat->setBaseColor(gfx::Color3(0.5,0.5,0.8));
  if (!pd->isMobile())
    pMat->setDensity(6); // make the platform heavy
  else
    pMat->setDensity(0.25);
  platformSolids[Platform] = ref<Solid>(solidSystem->createSolid(pBox, pMat));
  platformSolids[Platform]->setName(pd->getName());
  solidSystem->addSolid(platformSolids[Platform]);
  platformSolids[Platform]->setEnabled(dynamic);

  Quat4 orient(initialOrientation.getQuat4());
  Point3 spos( initialPosition - orient.rotate(pd->originOffset()) );
  platformSolids[Platform]->setPosition(spos);
  platformSolids[Platform]->setOrientation(orient);
  spatialGroup->add(platformSolids[Platform]);


  if (pd->isMobile()) { // add wheels

    const Real wheelWidth = 0.25;
    const Real wheelRadius = 0.40;
    const Real wheelOffsetDown = 0.65;
    const Real wheelPivotOffset = 0.22; // distance beween wheel origin and the steering pivot point (toward platform)
    const Real wheelMountLength = 0.4; // length of the left & right wheel mounts

    const Real L = pd->L(); // distance of drive axle back from platform origin
    const Real W = pd->W(); // distance between steering axle and drive axle


    if (pd->isHolonomic()) {

      throw std::runtime_error(Exception("holonomic mobile platform not yet implemented - sorry."));

    } else { // non-holonomic

      // add two drive wheels at the back and two steering wheels at the front
      ref<physics::Material> wMat(new physics::Material());
      wMat->setBaseColor(gfx::Color3(0.1,0.1,0.1));// dark grey
      //wMat->setSurfaceAppearance(PathName("images/basn2c16.png"));
      ref<Cylinder> wCyl(NewObj Cylinder(wheelWidth, wheelRadius));

      // create solids for each wheel
      platformSolids[LeftFrontWheel] = ref<Solid>(solidSystem->createSolid(wCyl, wMat));
      platformSolids[LeftFrontWheel]->setName(pd->getName()+":leftFrontWheel");
      solidSystem->addSolid(platformSolids[LeftFrontWheel]);
      platformSolids[LeftFrontWheel]->setEnabled(dynamic);
      spatialGroup->add(platformSolids[LeftFrontWheel]);

      platformSolids[RightFrontWheel] = ref<Solid>(solidSystem->createSolid(wCyl, wMat));
      platformSolids[RightFrontWheel]->setName(pd->getName()+":rightFrontWheel");
      solidSystem->addSolid(platformSolids[RightFrontWheel]);
      platformSolids[RightFrontWheel]->setEnabled(dynamic);
      spatialGroup->add(platformSolids[RightFrontWheel]);

      platformSolids[LeftBackWheel] = ref<Solid>(solidSystem->createSolid(wCyl, wMat));
      platformSolids[LeftBackWheel]->setName(pd->getName()+":leftBackWheel");
      solidSystem->addSolid(platformSolids[LeftBackWheel]);
      platformSolids[LeftBackWheel]->setEnabled(dynamic);
      spatialGroup->add(platformSolids[LeftBackWheel]);

      platformSolids[RightBackWheel] = ref<Solid>(solidSystem->createSolid(wCyl, wMat));
      platformSolids[RightBackWheel]->setName(pd->getName()+":rightBackWheel");
      solidSystem->addSolid(platformSolids[RightBackWheel]);
      platformSolids[RightBackWheel]->setEnabled(dynamic);
      spatialGroup->add(platformSolids[RightBackWheel]);

      // position them relative to the platform
      Point3 leftFrontWheelPos(initialPosition);
      Quat4 leftFrontWheelOrient(initialOrientation.getQuat4());

      Point3 rightFrontWheelPos(initialPosition);
      Quat4 rightFrontWheelOrient(initialOrientation.getQuat4());

      Point3 leftBackWheelPos(initialPosition);
      Quat4 leftBackWheelOrient(initialOrientation.getQuat4());

      Point3 rightBackWheelPos(initialPosition);
      Quat4 rightBackWheelOrient(initialOrientation.getQuat4());

      // rotate left wheels upright (Z points out of the side of platform)
      leftFrontWheelOrient *= Quat4(Vector3(1,0,0),-consts::Pi/2.0);
      leftBackWheelOrient *= Quat4(Vector3(1,0,0),-consts::Pi/2.0);
      // rotate right wheels upright (the other way)
      rightFrontWheelOrient *= Quat4(Vector3(1,0,0),consts::Pi/2.0);
      rightBackWheelOrient *= Quat4(Vector3(1,0,0),consts::Pi/2.0);

      // translate to platform sides
      Real yoffset = dim.y/2.0;
      leftFrontWheelPos += leftFrontWheelOrient.rotate(Vector3(0,0,yoffset));
      leftBackWheelPos += leftFrontWheelOrient.rotate(Vector3(0,0,yoffset));
      rightFrontWheelPos += rightFrontWheelOrient.rotate(Vector3(0,0,yoffset));
      rightBackWheelPos += rightFrontWheelOrient.rotate(Vector3(0,0,yoffset));

      // translate to front/back of platfom
      leftFrontWheelPos += leftFrontWheelOrient.rotate(Vector3(W-L,0,0));
      rightFrontWheelPos += rightFrontWheelOrient.rotate(Vector3(W-L,0,0));
      leftBackWheelPos += leftFrontWheelOrient.rotate(Vector3(-L,0,0));
      rightBackWheelPos += rightFrontWheelOrient.rotate(Vector3(-L,0,0));

      // translate down
      leftFrontWheelPos += leftFrontWheelOrient.rotate(Vector3(0,wheelOffsetDown,0));
      leftBackWheelPos += leftFrontWheelOrient.rotate(Vector3(0,wheelOffsetDown,0));
      rightFrontWheelPos += rightFrontWheelOrient.rotate(Vector3(0,-wheelOffsetDown,0));
      rightBackWheelPos += rightFrontWheelOrient.rotate(Vector3(0,-wheelOffsetDown,0));


      // set pos/orient
      platformSolids[LeftFrontWheel]->setPosition( leftFrontWheelPos );
      platformSolids[LeftFrontWheel]->setOrientation( leftFrontWheelOrient );
      platformSolids[RightFrontWheel]->setPosition( rightFrontWheelPos );
      platformSolids[RightFrontWheel]->setOrientation( rightFrontWheelOrient );
      platformSolids[LeftBackWheel]->setPosition( leftBackWheelPos );
      platformSolids[LeftBackWheel]->setOrientation( leftBackWheelOrient );
      platformSolids[RightBackWheel]->setPosition( rightBackWheelPos );
      platformSolids[RightBackWheel]->setOrientation( rightBackWheelOrient );


      // the front wheels need a crossbar steering system to connect them
      //  and keep them facing the same direction
      wheelConstraintGroup = solidSystem->createConstraintGroup();

      // first the wheel mounts
      ref<Box> mBox(NewObj Box(wheelMountLength,wheelMountLength/4.0,wheelMountLength/4.0));
      ref<physics::Material> mMat(new physics::Material());
      mMat->setBaseColor(gfx::Color3(0.7,0.7,0.7));

      platformSolids[LeftFrontWheelMount] = ref<Solid>(solidSystem->createSolid(mBox, mMat));
      platformSolids[LeftFrontWheelMount]->setName(pd->getName()+":leftFrontWheelMount");
      solidSystem->addSolid(platformSolids[LeftFrontWheelMount]);
      platformSolids[LeftFrontWheelMount]->setEnabled(dynamic);
      spatialGroup->add(platformSolids[LeftFrontWheelMount]);

      platformSolids[RightFrontWheelMount] = ref<Solid>(solidSystem->createSolid(mBox, mMat));
      platformSolids[RightFrontWheelMount]->setName(pd->getName()+":rightFrontWheelMount");
      solidSystem->addSolid(platformSolids[RightFrontWheelMount]);
      platformSolids[RightFrontWheelMount]->setEnabled(dynamic);
      spatialGroup->add(platformSolids[RightFrontWheelMount]);

      platformSolids[LeftFrontWheelMount]->setPosition( leftFrontWheelPos + leftFrontWheelOrient.rotate(Vector3(0,0,-wheelPivotOffset)) );
      platformSolids[LeftFrontWheelMount]->setPosition( platformSolids[LeftFrontWheelMount]->getPosition() + initialOrientation.getQuat4().rotate(Vector3(wheelMountLength/2.0,0,0)) );
      platformSolids[LeftFrontWheelMount]->setOrientation( initialOrientation.getQuat4() );
      platformSolids[RightFrontWheelMount]->setPosition( rightFrontWheelPos + rightFrontWheelOrient.rotate(Vector3(0,0,-wheelPivotOffset)) );
      platformSolids[RightFrontWheelMount]->setOrientation( initialOrientation.getQuat4() );
      platformSolids[RightFrontWheelMount]->setPosition( platformSolids[RightFrontWheelMount]->getPosition() + initialOrientation.getQuat4().rotate(Vector3(wheelMountLength/2.0,0,0)) );

      // create the steering crossbar to connect the right & left wheel mounts
      ref<Box> xBox(NewObj Box(0.1,dim.y - 2.0*wheelPivotOffset,0.1));
      platformSolids[Crossbar] = ref<Solid>(solidSystem->createSolid(xBox, mMat));
      platformSolids[Crossbar]->setName(pd->getName()+":steeringCrossbar");
      solidSystem->addSolid(platformSolids[Crossbar]);
      platformSolids[Crossbar]->setEnabled(dynamic);
      spatialGroup->add(platformSolids[Crossbar]);
      // position it
      Point3 crossbarPos( leftFrontWheelPos );
      Quat4  crossbarOrient( initialOrientation.getQuat4() );
      crossbarPos += crossbarOrient.rotate(Vector3(0,-dim.y/2.0,0));
      crossbarPos += crossbarOrient.rotate(Vector3(wheelMountLength,0,0));
      platformSolids[Crossbar]->setPosition( crossbarPos );
      platformSolids[Crossbar]->setOrientation( crossbarOrient );


      // only connect everything up with constraints & motors if we're doing dynamic simulation.
      //  That way, for static simulation the Solids can be explicitly positioned without
      //  worrying about maintaining constraints
      if (dynamic) {

        // connect the mounts to the wheels
        ref<HingeJoint> leftMountHinge = solidSystem->createHingeJoint();
        wheelConstraintGroup->addConstraint(leftMountHinge);
        leftMountHinge->attach(platformSolids[LeftFrontWheelMount], platformSolids[LeftFrontWheel]);
        leftMountHinge->setAxis(Vector3(0,1,0));
        leftMountHinge->setAnchor(Point3(-wheelMountLength/2.0,0,0));

        ref<HingeJoint> rightMountHinge = solidSystem->createHingeJoint();
        wheelConstraintGroup->addConstraint(rightMountHinge);
        rightMountHinge->attach(platformSolids[RightFrontWheelMount], platformSolids[RightFrontWheel]);
        rightMountHinge->setAxis(Vector3(0,1,0));
        rightMountHinge->setAnchor(Point3(-wheelMountLength/2.0,0,0));

        // connect the mounts to the crossbar
        ref<HingeJoint> leftCrossbarHinge = solidSystem->createHingeJoint();
        wheelConstraintGroup->addConstraint(leftCrossbarHinge);
        leftCrossbarHinge->attach(platformSolids[LeftFrontWheelMount], platformSolids[Crossbar]);
        leftCrossbarHinge->setAxis(Vector3(0,0,1));
        leftCrossbarHinge->setAnchor(Point3(wheelMountLength/2.0,0,0));

        ref<HingeJoint> rightCrossbarHinge = solidSystem->createHingeJoint();
        wheelConstraintGroup->addConstraint(rightCrossbarHinge);
        rightCrossbarHinge->attach(platformSolids[RightFrontWheelMount], platformSolids[Crossbar]);
        rightCrossbarHinge->setAxis(Vector3(0,0,1));
        rightCrossbarHinge->setAnchor(Point3(wheelMountLength/2.0,0,0));


        // now connect the mounts to the platform
        ref<HingeJoint> leftSteeringHinge = solidSystem->createHingeJoint();
        wheelConstraintGroup->addConstraint(leftSteeringHinge);
        leftSteeringHinge->attach(platformSolids[LeftFrontWheelMount], platformSolids[Platform]);
        leftSteeringHinge->setAxis(Vector3(0,0,-1));
        leftSteeringHinge->setAnchor(Point3(-wheelMountLength/2.0,0,0));
        leftSteeringHinge->setLowStop(-Math::degToRad(35));
        leftSteeringHinge->setHighStop(Math::degToRad(35));

        ref<HingeJoint> rightSteeringHinge = solidSystem->createHingeJoint();
        wheelConstraintGroup->addConstraint(rightSteeringHinge);
        rightSteeringHinge->attach(platformSolids[RightFrontWheelMount], platformSolids[Platform]);
        rightSteeringHinge->setAxis(Vector3(0,0,-1));
        rightSteeringHinge->setAnchor(Point3(-wheelMountLength/2.0,0,0));



        // Back wheels with HingeJoints
        ref<HingeJoint> hingeLeft = solidSystem->createHingeJoint();
        wheelConstraintGroup->addConstraint(hingeLeft);
        hingeLeft->attach(platformSolids[Platform], platformSolids[LeftBackWheel]);
        hingeLeft->setAxis(Vector3(0,-1,0));
        hingeLeft->setAnchor( leftBackWheelPos - platformSolids[Platform]->getPosition() );

        ref<HingeJoint> hingeRight = solidSystem->createHingeJoint();
        wheelConstraintGroup->addConstraint(hingeRight);
        hingeRight->attach(platformSolids[Platform], platformSolids[RightBackWheel]);
        hingeRight->setAxis(Vector3(0,-1,0));
        hingeRight->setAnchor( rightBackWheelPos - platformSolids[Platform]->getPosition() );



        // now attach drive motors
        leftDriveMotor = solidSystem->createMotor();
        hingeLeft->attachMotor(1, leftDriveMotor);
        leftDriveMotor->setTargetVel(0);
        leftDriveMotor->setMaxForce(0);
        leftDriveHingeJoint = hingeLeft;

        rightDriveMotor = solidSystem->createMotor();
        hingeRight->attachMotor(1, rightDriveMotor);
        rightDriveMotor->setTargetVel(0);
        rightDriveMotor->setMaxForce(0.005);
        rightDriveHingeJoint = hingeRight;


        // and a steering motor on the left
        steeringMotor = solidSystem->createMotor();
        leftSteeringHinge->attachMotor(1, steeringMotor);
        steeringMotor->setTargetVel(0);
        steeringMotor->setMaxForce(0.005);
        steeringHingeJoint = leftSteeringHinge;


        // add some motors to the front wheels, just to provide some
        //  'friction-like' damping.
        ref<Motor> leftFrontWheelMotor = solidSystem->createMotor();
        leftMountHinge->attachMotor(1, leftFrontWheelMotor);
        leftFrontWheelMotor->setTargetVel(0);
        leftFrontWheelMotor->setMaxForce(0.02);

        ref<Motor> rightFrontWheelMotor = solidSystem->createMotor();
        rightMountHinge->attachMotor(1, rightFrontWheelMotor);
        rightFrontWheelMotor->setTargetVel(0);
        rightFrontWheelMotor->setMaxForce(0.02);
      }

    }

  }

}



ref<physics::Collidable> SimulatedPlatform::createCollidable(CollidableFlags flags)
{
  Assert(solidSystem);
  Assert(platformSolids.size() > 0);

  ref<CollisionCuller> cc(solidSystem->getCollisionCuller());
  ref<const robot::PlatformDescription> pd(platformDescr);

  array<ref<Collidable> > collidables(platformSolids.size());
  collidables[Platform] = platformSolids[Platform]->createCollidable(flags);
  collidables[Platform]->setName("platformBody");

  if (pd->isMobile()) {
    // Create a group with the platform and all the rest in a subgroup
    ref<CollidableGroup> platformGroup = cc->createCollidableGroup();
    platformGroup->setName(className()+" collidables");

    platformGroup->addCollidable( collidables[Platform] );

    ref<CollidableGroup> partsGroup = cc->createCollidableGroup();
    partsGroup->setName(className()+" parts Group");
    partsGroup->setChildIntercollisionEnabled(false);

    for(PlatformSolids p=PlatformSolids(Platform+1); p<PlatformSolids(platformSolids.size()); p = PlatformSolids(p+1)) {
      collidables[p] = platformSolids[p]->createCollidable(flags);
      partsGroup->addCollidable( collidables[p] );
    }

    platformGroup->addCollidable( partsGroup );

    disableCollisions(collidables);
    return platformGroup;
  }
  else {
    return collidables[Platform];
  }

}



void SimulatedPlatform::disableCollisions(const array<ref<Collidable> >& collidables)
{
  ref<const robot::PlatformDescription> pd(platformDescr);
  if (!pd->isMobile()) return;

  // don't collide any of the wheels, mounts or crossbar with the platform
  //  !!! should relax this for the real wheels once the collision model
  //      uses a  cylinder instead of a capped cylinder (??)

  ref<CollisionCuller> cc(solidSystem->getCollisionCuller());
  cc->collisionEnable(false,collidables[LeftFrontWheel], collidables[Platform]);
  cc->collisionEnable(false,collidables[RightFrontWheel], collidables[Platform]);
  cc->collisionEnable(false,collidables[LeftFrontWheelMount], collidables[Platform]);
  cc->collisionEnable(false,collidables[LeftFrontWheelMount], collidables[LeftFrontWheel]);
  cc->collisionEnable(false,collidables[LeftFrontWheelMount], collidables[Crossbar]);
  cc->collisionEnable(false,collidables[RightFrontWheelMount], collidables[Platform]);
  cc->collisionEnable(false,collidables[RightFrontWheelMount], collidables[RightFrontWheel]);
  cc->collisionEnable(false,collidables[RightFrontWheelMount], collidables[Crossbar]);
  cc->collisionEnable(false,collidables[LeftBackWheel], collidables[Platform]);
  cc->collisionEnable(false,collidables[RightBackWheel], collidables[Platform]);
  cc->collisionEnable(false,collidables[Crossbar], collidables[Platform]);
  cc->collisionEnable(false,collidables[Crossbar], collidables[LeftFrontWheel]);
  cc->collisionEnable(false,collidables[Crossbar], collidables[RightFrontWheel]);

}



bool SimulatedPlatform::formatSupported(String format, Real version, ExternalizationType type) const
{
  return false; // none supported (yet)
}

void SimulatedPlatform::externalize(base::Externalizer& e, String format, Real version)
{
  //if (format=="") format = String("dh");

  if (!formatSupported(format,version))
    throw std::invalid_argument(Exception(String("format ")+format+" "+base::realToString(version)+" unsupported"));

}
