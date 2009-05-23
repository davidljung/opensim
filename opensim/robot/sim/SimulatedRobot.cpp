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

  $Id: SimulatedRobot.cpp 1101 2004-09-27 21:54:27Z jungd $
  $Revision: 1.23 $
  $Date: 2004-09-27 17:54:27 -0400 (Mon, 27 Sep 2004) $
  $Author: jungd $

****************************************************************************/

#include <robot/sim/SimulatedRobot>

#include <sstream>

#include <base/SimpleXMLSerializer>
#include <base/Externalizer>
#include <base/Orient>

#include <physics/Box>
#include <physics/Solid>
#include <physics/ConstraintGroup>
#include <physics/FixedConstraint>
#include <physics/CollisionCuller>
#include <physics/CollisionDetector>

#include <robot/sim/SimulatedPlatform>
#include <robot/sim/SimulatedSerialManipulator>

#include <robot/sim/SimulatedManipulatorDescription>
#include <robot/sim/SimulatedToolDescription>


using robot::sim::SimulatedRobot;

using base::array;
using base::Point3;
using base::VFile;
using base::Externalizer;
using base::Orient;
using physics::Box;
using physics::Solid;
using physics::Spatial;
using physics::SpatialGroup;
using physics::ConstraintGroup;
using physics::FixedConstraint;
using physics::Collidable;
using physics::CollidableGroup;
using physics::CollisionCuller;
using physics::CollisionDetector;
using robot::sim::SimulatedManipulatorDescription;
using robot::sim::SimulatedToolDescription;
using robot::sim::SimulatedPlatform;
using robot::sim::SimulatedSerialManipulator;




SimulatedRobot::SimulatedRobot(ref<base::VFile> robotSpecification,
                               const base::Point3& initialPosition, const base::Orient& initialOrientation,
                               ref<physics::SolidSystem> solidSystem, bool dynamic)
  : solidSystem(solidSystem), dynamic(dynamic)
{
  Assert(solidSystem!=0);

  // read in supported formats
  if (robotSpecification->extension() == "xml") {

    // read in parameters
    try {
      Externalizer e(Externalizable::Input, robotSpecification);
      externalize(e,"xml",1.0);
    } catch (std::exception&) {
      throw std::invalid_argument(Exception("not a valid robot .xml file."));
    }

  }
  else
    throw std::invalid_argument(Exception("file format unsupported."));

  construct(initialPosition, initialOrientation);
}


SimulatedRobot::SimulatedRobot(ref<const robot::RobotDescription> robotDescription,
                               const base::Point3& initialPosition, const base::Orient& initialOrientation,
                               ref<physics::SolidSystem> solidSystem, bool dynamic)
  : solidSystem(solidSystem), dynamic(dynamic)
{
  Assert(solidSystem!=0);

  setRobotDescription( robotDescription );

  platform = ref<SimulatedPlatform>(NewObj SimulatedPlatform(robotDescription->platform(), solidSystem, dynamic));

  const array<ref<const ManipulatorDescription> > manips(robotDescription->manipulators());
  for (Int i=0; i<manips.size(); i++) {
    ref<SimulatedSerialManipulator> m(NewObj SimulatedSerialManipulator(manips[i], solidSystem, dynamic));
    manipulators.push_back(m);
  }

  construct(initialPosition, initialOrientation);
}




// factory methods for descriptions

ref<robot::PlatformDescription> SimulatedRobot::newPlatformDescription() const
{
  return ref<PlatformDescription>(NewObj PlatformDescription());
}


ref<robot::ManipulatorDescription> SimulatedRobot::newManipulatorDescription() const
{
  return ref<ManipulatorDescription>(NewObj SimulatedManipulatorDescription());
}







void SimulatedRobot::setDynamic(bool enabled)
{
  platform->setDynamic(false);

  reflist<SimulatedSerialManipulator>::iterator m = manipulators.begin();
  reflist<SimulatedSerialManipulator>::iterator end = manipulators.end();
  while (m != end) {
    (*m)->setDynamic(enabled);
    ++m;
  }
}



array<std::pair<String,String> > SimulatedRobot::controlInterfaces() const
{
  array<std::pair<String,String> > a;
  if (dynamic) {
    a.push_back(std::make_pair<String,String>("platform","PlatformControl"));
    a.push_back(std::make_pair<String,String>("platformVelocity","PlatformVelocityControl"));
  }
  else
    a.push_back(std::make_pair<String,String>("platformPosition","PlatformPositionControl"));

  for(Int m=0; m<getRobotDescription()->manipulators().size();m++) {
    ref<const ManipulatorDescription> md( getRobotDescription()->manipulators()[m] );
    ref<const SimulatedManipulatorDescription> smd;
    if (instanceof(*md, const SimulatedManipulatorDescription))
      smd = narrow_ref<const SimulatedManipulatorDescription>(md);
    else
      smd = ref<SimulatedManipulatorDescription>(NewObj SimulatedManipulatorDescription(*md));

    String n(base::intToString(m+1));
    if (dynamic) {
      a.push_back(std::make_pair<String,String>("manipulator"+n,"JointForceControl"));
      a.push_back(std::make_pair<String,String>("manipulatorVelocity"+n,"JointVelocityControl"));
      a.push_back(std::make_pair<String,String>("tool"+n,"JointForceControl"));
      a.push_back(std::make_pair<String,String>("toolVelocity"+n,"JointVelocityControl"));
    }
    else {
      a.push_back(std::make_pair<String,String>("manipulatorPosition"+n,"JointPositionControl"));
      a.push_back(std::make_pair<String,String>("toolPosition"+n,"JointPositionControl"));
    }
    if (smd->hasLinkProximitySensors())
      a.push_back(std::make_pair<String,String>("manipulatorProximity"+n,"LinkProximitySensors"));
    a.push_back(std::make_pair<String,String>("manipulatorToolGrip"+n,"ToolGripControl"));
  }
  return a;
}





// Spatial

void SimulatedRobot::setPosition(const Point3& pos)
{
  spatialGroup->updateGroupConfiguration( platform->getConfiguration() );
  spatialGroup->setPosition(pos);
}

Point3 SimulatedRobot::getPosition() const
{
  spatialGroup->updateGroupConfiguration( platform->getConfiguration() );
  return spatialGroup->getPosition();
}

void SimulatedRobot::setOrientation(const Orient& orient)
{
  spatialGroup->updateGroupConfiguration( platform->getConfiguration() );
  spatialGroup->setOrientation(orient);
}

Orient SimulatedRobot::getOrientation() const
{
  spatialGroup->updateGroupConfiguration( platform->getConfiguration() );
  return spatialGroup->getOrientation();
}

void SimulatedRobot::setConfiguration(const base::Transform& configuration)
{
  spatialGroup->updateGroupConfiguration( platform->getConfiguration() );
  spatialGroup->setConfiguration(configuration);
}

base::Transform SimulatedRobot::getConfiguration() const
{
  spatialGroup->updateGroupConfiguration( platform->getConfiguration() );
  return spatialGroup->getConfiguration();
}






bool SimulatedRobot::formatSupported(String format, Real version, ExternalizationType type) const
{
  return ( ((format=="dh") || (format=="xml")) && (version==1.0) && (type==Input) );
}


void SimulatedRobot::externalize(base::Externalizer& e, String format, Real version)
{
  if (format=="") format = String("xml");

  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" "+base::realToString(version)+" unsupported"));

  if (format == "dh") {

    if (e.isInput()) {

      spatialGroup = ref<SpatialGroup>(NewObj SpatialGroup());

      // dh format doesn't specify anything about the platform, just instantiate the default
      ref<PlatformDescription> pd(NewObj PlatformDescription("platform",base::Dimension3(1,1,0.1), base::Vector3(0,0,0.05), false));
      platform = ref<SimulatedPlatform>(NewObj SimulatedPlatform(pd));
      spatialGroup->add(platform);

      // externalize SimulatedSerialManipulator
      //  (dh format only specifies a single manipulator)
      ref<SimulatedSerialManipulator> m(NewObj SimulatedSerialManipulator(solidSystem));
      m->externalize(e,format,version);
      manipulators.push_back(m);
      spatialGroup->add(m);

      // dh format doesn't specify the manipulator offset from the platform either
      array<Vector3> offsets(1);
      offsets[0] = Vector3(); // default to 0

      // construct RobotDescription
      array<ref<const ManipulatorDescription> > manipulatorDescrs(1);
      manipulatorDescrs[0] = m->getManipulatorDescription();

      setRobotDescription( ref<RobotDescription>(NewObj RobotDescription("robot",pd,manipulatorDescrs,offsets)) );

    }

  }
  else if (format == "xml") {

    if (e.isInput()) {
      // let the RobotDescription read the xml first,
      //  then look for simulation specific information

      ref<RobotDescription> rd(NewObj RobotDescription());
      rd->externalize(e, format, version);

      setRobotDescription( rd );

      spatialGroup = ref<SpatialGroup>(NewObj SpatialGroup());

      platform = ref<SimulatedPlatform>(NewObj SimulatedPlatform(rd->platform()));
      spatialGroup->add(platform);

      const array<ref<const ManipulatorDescription> >& manips(rd->manipulators());
      for (Int i=0; i<manips.size(); i++) {
        ref<SimulatedSerialManipulator> m(NewObj SimulatedSerialManipulator(manips[i], solidSystem));
        manipulators.push_back(m);
        spatialGroup->add(m);
      }

    } // end input

  }

}




void SimulatedRobot::construct(const base::Point3& initialPosition, const base::Orient& initialOrientation)
{
  Assert(solidSystem);
  spatialGroup = ref<SpatialGroup>(NewObj SpatialGroup());
  spatialGroup->setPositionOrientation(initialPosition, initialOrientation);

  // construct the platform
  Assert(platform);
  platform->setSolidSystem(solidSystem);
  platform->construct(initialPosition, initialOrientation);
  spatialGroup->add(platform);

  ref<const RobotDescription> rd(getRobotDescription());
  ref<ConstraintGroup> cgroup(solidSystem->createConstraintGroup());

  Int manipulatorIndex=0;
  // for each manipulator, construct it and attach it to the platform
  reflist<SimulatedSerialManipulator>::iterator m = manipulators.begin();
  reflist<SimulatedSerialManipulator>::iterator end = manipulators.end();
  while (m != end) {
    ref<SimulatedSerialManipulator> sm(*m);

    // construct (initial position of the manipulator is the position of the platform/robot + manip offset translation)
    Matrix4 offsetTransform; offsetTransform.setToTranslation(rd->manipulatorOffsets()[manipulatorIndex]);
    Matrix4 platformTransform(initialOrientation.getRotationMatrix3()); platformTransform.setTranslationComponent(initialPosition);
    Matrix4 manipTransform(platformTransform * offsetTransform);
    Vector3 manipPosition = manipTransform.column(4).toVector3();
    Orient  manipOrient = Matrix3(manipTransform);

    sm->setSolidSystem(solidSystem);
    sm->construct(manipPosition, manipOrient);
    spatialGroup->add(sm);

    // fix the manipulator base link to the platform
    ref<FixedConstraint> fixed(solidSystem->createFixedConstraint());
    cgroup->addConstraint(fixed);
    fixed->attach(platform->getPlatformSolid(), sm->getBaseSolid());

    ++m; ++manipulatorIndex;
  }

  if (manipulatorIndex > 0)
    solidSystem->addConstraintGroup(cgroup);

}



bool SimulatedRobot::checkProximity(ref<SimulatedTool> tool)
{
  // ask each manipulator to check
  bool anyInProximity = false;
  reflist<SimulatedSerialManipulator>::iterator m = manipulators.begin();
  reflist<SimulatedSerialManipulator>::iterator end = manipulators.end();
  while (m != end) {
    ref<SimulatedSerialManipulator> sm(*m);
    if (sm->checkProximity(tool))
      anyInProximity=true;
    ++m;
  }
  return anyInProximity;
}


void SimulatedRobot::placeToolInProximity(ref<SimulatedTool> tool, Int manipIndex)
{
  ref<SimulatedSerialManipulator> sm( base::elementAt(manipulators, manipIndex) );

  // new tool pos/orient
  tool->setPositionOrientation( sm->getEEPosition(), sm->getEEOrientation() );
  sm->setToolInProximity(tool);
}


bool SimulatedRobot::graspTool(Int manipIndex)
{
  ref<SimulatedSerialManipulator> sm( base::elementAt(manipulators, manipIndex) );
  return sm->graspTool();
}


void SimulatedRobot::releaseGrasp(Int manipIndex)
{
  ref<SimulatedSerialManipulator> sm( base::elementAt(manipulators, manipIndex) );
  sm->releaseGrasp();
}


ref<physics::Collidable> SimulatedRobot::createCollidable(CollidableFlags flags)
{
  ref<CollisionCuller> cc(solidSystem->getCollisionCuller());

  ref<CollidableGroup> collidableGroup = cc->createCollidableGroup();
  if (isDescriptionProvided())
    collidableGroup->setName(className()+":"+getRobotDescription()->getName()+" collidables");
  else
    collidableGroup->setName(className()+" collidables");
  ref<Collidable> platformCollidable( platform->createCollidable(flags) );
  collidableGroup->addCollidable(platformCollidable);
  ref<Collidable> platform( platformCollidable->findNamed("platformBody") );

  // for each manipulator, create a collidable
  reflist<SimulatedSerialManipulator>::iterator m = manipulators.begin();
  reflist<SimulatedSerialManipulator>::iterator end = manipulators.end();
  while (m != end) {
    ref<SimulatedSerialManipulator> sm(*m);
    ref<Collidable> smCollidable( sm->createCollidable(flags) );
    collidableGroup->addCollidable( smCollidable );

    // disable collision detection between platform and base link (& its proximity sensor Collidable)
    ref<Collidable> base( smCollidable->findNamed("baseLink") );
    if (platform && base)
      cc->collisionEnable(false,platform,base);
    ref<Collidable> baseSensor( smCollidable->findNamed("baseLink Sensor") );
    if (platform && baseSensor)
      cc->collisionEnable(false,platform,baseSensor);

    ++m;
  }

  return collidableGroup;
}







ref<robot::ControlInterface> SimulatedRobot::getControlInterface(String interfaceName) throw(std::invalid_argument)
{
  if (interfaceName == "") interfaceName = "platform";

  // these if's test the name string prefix, so don't forget to put the longer names before shorter ones
  //  with a common prefix.

  //  if (interfaceName.substr(0,20) == "manipulatorProximity")
  //    return proximitySensorControlInterface;


  ref<SimulatedRobot> self(this);
  if (interfaceName == "platformVelocity")
    return ref<robot::ControlInterface>(NewObj PlatformControlInterface(interfaceName, "PlatformVelocityControl", self, platform, VelControl));

  if (interfaceName == "platformPosition")
    return ref<robot::ControlInterface>(NewObj PlatformControlInterface(interfaceName, "PlatformPositionControl", self, platform, PosControl));

  if (interfaceName == "platform")
    return ref<robot::ControlInterface>(NewObj PlatformControlInterface(interfaceName, "PlatformControl", self, platform, ForceControl));


  if (interfaceName.substr(0,19) == "manipulatorVelocity") {
    Int n = base::stringToInt(interfaceName.substr(19));
    if ((n<1) || (n > manipulators.size()))
      throw std::invalid_argument(Exception("manipulator index out of range"));
    ref<SimulatedSerialManipulator> manip( base::elementAt(manipulators,n-1) );
    return ref<robot::ControlInterface>(NewObj ManipulatorControlInterface(interfaceName,"JointVelocityControl",manip,VelControl));
  }

  if (interfaceName.substr(0,19) == "manipulatorPosition") {
    Int n = base::stringToInt(interfaceName.substr(19));
    if ((n<1) || (n > manipulators.size()))
      throw std::invalid_argument(Exception("manipulator index out of range"));
    ref<SimulatedSerialManipulator> manip( base::elementAt(manipulators,n-1) );
    return ref<robot::ControlInterface>(NewObj ManipulatorControlInterface(interfaceName,"JointPositionControl",manip,PosControl));
  }


  if (interfaceName.substr(0,19) == "manipulatorToolGrip") {
    Int n = base::stringToInt(interfaceName.substr(19));
    if ((n<1) || (n > manipulators.size()))
      throw std::invalid_argument(Exception("manipulator index out of range"));
    ref<SimulatedSerialManipulator> manip( base::elementAt(manipulators,n-1) );
    return ref<robot::ControlInterface>(NewObj ToolControlInterface(interfaceName,"ToolGripControl", manip, true));
  }

  if (interfaceName.substr(0,20) == "manipulatorProximity") {
    Int n = base::stringToInt(interfaceName.substr(20));
    if ((n<1) || (n > manipulators.size()))
      throw std::invalid_argument(Exception("manipulator index out of range"));
    ref<SimulatedSerialManipulator> manip( base::elementAt(manipulators,n-1) );
    if (manip->getManipulatorDescription()->hasLinkProximitySensors()) {
      return ref<robot::ControlInterface>(NewObj ProximitySensorInterface(interfaceName, "LinkProximitySensors", manip));
    }
    else
      throw std::invalid_argument(Exception(String("manipulator has no proximity sensors; invalid interface ")+interfaceName));
  }

  if (interfaceName.substr(0,11) == "manipulator") {
    Int n = base::stringToInt(interfaceName.substr(11));
    if ((n<1) || (n > manipulators.size()))
      throw std::invalid_argument(Exception(String("manipulator index ")+base::intToString(n)+" out of range"));
    ref<SimulatedSerialManipulator> manip( base::elementAt(manipulators,n-1) );
    return ref<robot::ControlInterface>(NewObj ManipulatorControlInterface(interfaceName,"JointForceControl",manip));
  }


  if (interfaceName.substr(0,12) == "toolVelocity") {
    Int n = base::stringToInt(interfaceName.substr(12));
    if ((n<1) || (n > manipulators.size()))
      throw std::invalid_argument(Exception("manipulator index out of range"));
    ref<SimulatedSerialManipulator> manip( base::elementAt(manipulators,n-1) );
    return ref<robot::ControlInterface>(NewObj ToolControlInterface(interfaceName,"JointVelocityControl", manip, false, VelControl));
  }

  if (interfaceName.substr(0,12) == "toolPosition") {
    Int n = base::stringToInt(interfaceName.substr(12));
    if ((n<1) || (n > manipulators.size()))
      throw std::invalid_argument(Exception("manipulator index out of range"));
    ref<SimulatedSerialManipulator> manip( base::elementAt(manipulators,n-1) );
    return ref<robot::ControlInterface>(NewObj ToolControlInterface(interfaceName,"JointPositionControl", manip, false, PosControl));
  }

  if (interfaceName.substr(0,4) == "tool") {
    Int n = base::stringToInt(interfaceName.substr(4));
    if ((n<1) || (n > manipulators.size()))
      throw std::invalid_argument(Exception("manipulator index out of range"));
    ref<SimulatedSerialManipulator> manip( base::elementAt(manipulators,n-1) );
    return ref<robot::ControlInterface>(NewObj ToolControlInterface(interfaceName,"JointForceControl", manip, false, ForceControl));
  }


  throw std::invalid_argument(Exception(String("unknown interface ")+interfaceName));
}

