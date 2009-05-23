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
  
  $Id: TestRobot.cpp 1039 2004-02-11 20:50:52Z jungd $
  $Revision: 1.9 $
  $Date: 2004-02-11 15:50:52 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <robot/TestRobot>

#include <base/Application>

using robot::TestRobot;

using base::Vector;
using base::IVector;
using base::Matrix4;
using base::VFile;
using base::Application;


using consts::Pi;


TestRobot::TestRobot()
{
  base::IVector jt(6); // joint type
  base::Vector alpha(6), a(6), d(6), theta(6);

  for(Int j=0; j<6; j++) jt[j] = Revolute;
  // alpha,a,d,theta
  alpha[0] = -Pi/2.0; a[0] = 0;             d[0] = 0;             theta[0] = Pi/2.0; 
  alpha[1] = 0;       a[1] = 431.8/1000.0;  d[1] = 149.09/1000.0; theta[1] = 0; 
  alpha[2] = Pi/2.0;  a[2] = -20.32/1000.0; d[2] = 0;             theta[2] = Pi/2.0;
  alpha[3] = -Pi/2.0; a[3] = 0;             d[3] = 433.07/1000.0; theta[3] = 0;
  alpha[4] = Pi/2.0;  a[4] = 0;             d[4] = 0;             theta[4] = 0;
  alpha[5] = 0;       a[5] = 0;             d[5] = 56.25/1000.0;  theta[5] = 0;

  create("Puma",jt,alpha,a,d,theta);
  initManipulators();
}


TestRobot::TestRobot(ref<VFile> robotSpecification,
		     const base::Point3& initialPosition, const base::Orient& initialOrientation)
  : position(initialPosition), orientation(initialOrientation)
{
  // Read in specification
  ref<RobotDescription> robotDescription(NewObj RobotDescription());

  if (robotSpecification->extension() == "xml")
    robotDescription->load(robotSpecification, "xml", 1.0);
  else
    throw std::invalid_argument(Exception("unsupported file type"));

  setRobotDescription(robotDescription);
  initManipulators();
}


TestRobot::TestRobot(ref<const robot::RobotDescription> robotDescription,
		     const base::Point3& initialPosition, const base::Orient& initialOrientation)
  : position(initialPosition), orientation(initialOrientation)
{
  setRobotDescription(robotDescription);
  initManipulators();
}		     



TestRobot::TestRobot(const IVector& jointType, const Vector& alpha, const Vector& a, const Vector& d, const Vector& theta)
{
  create("manipulator", jointType, alpha, a, d, theta);
  initManipulators();
}



array<std::pair<String,String> > TestRobot::controlInterfaces() const
{
  array<std::pair<String, String> > a;
  a.push_back(std::make_pair<String,String>("platformPosition","PlatformPositionControl"));
  
  for(Int m=0; m<getRobotDescription()->manipulators().size();m++) {
    String n(base::intToString(m+1));
    a.push_back(std::make_pair<String,String>("manipulatorPosition"+n,"JointPositionControl"));
    a.push_back(std::make_pair<String,String>("manipulatorProximity"+n,"LinkProximitySensors"));
    a.push_back(std::make_pair<String,String>("manipulatorToolGrip"+n,"ToolGripControl"));
    a.push_back(std::make_pair<String,String>("toolPosition"+n,"JointPositionControl"));
  }
  return a; 
}



void TestRobot::create(String manipName, const IVector& jointType, const Vector& alpha, const Vector& a, const Vector& d, const Vector& theta)
{
  const Int dof = jointType.size();
  Assert( alpha.size() == dof );
  Assert( a.size() == dof );
  Assert( d.size() == dof );
  Assert( theta.size() == dof );

  // create RobotDescription
  
  ref<PlatformDescription> pd = ref<PlatformDescription>(NewObj PlatformDescription());
  
  KinematicChain kc;
  for(Int j=0; j<dof; j++) 
    kc.push_back(KinematicChain::Link(KinematicChain::Link::LinkType(jointType[j]),
				      alpha[j], a[j], d[j], theta[j],
				      Math::degToRad(-160.0),Math::degToRad(160.0)));

  Matrix4 I; I.setIdentity();

  ref<ManipulatorDescription>  md = ref<ManipulatorDescription>(NewObj ManipulatorDescription(manipName, I, kc));
  array<ref<const ManipulatorDescription> > mds(1);
  array<Vector3> offsets(1);
  mds[0]=md;
  offsets[0]=Vector3();
  
  setRobotDescription( ref<RobotDescription>(NewObj RobotDescription("TestRobot", pd, mds, offsets)) );
  
}


void TestRobot::initManipulators()
{
  // manipulator joint parameters array
  ref<const RobotDescription> rd(getRobotDescription());
  Int numManpis = rd->manipulators().size();
  qa.resize(numManpis);
  for (Int i=0; i<numManpis; i++) {
    ref<const ManipulatorDescription> md(rd->manipulators()[i]);
    Int dof = md->getKinematicChain().dof();
    qa[i].reset( zeroVector(dof) );
  }

  // tool arrays
  tqa.resize(numManpis);
  toolGrasped.resize(numManpis);
  toolProximity.resize(numManpis);
  for(Int i=0; i<numManpis; i++) {
    toolGrasped[i] = false;
    toolProximity[i] = false;
  }
  tools.resize(numManpis);
}


void TestRobot::placeToolInProximity(ref<const ToolDescription> toolDescription, Int manipIndex)
{
  if (toolGrasped[manipIndex]) {
    if (!tools[manipIndex]->equals(toolDescription))
      throw std::logic_error(Exception(String("tool '") + tools[manipIndex]->getName()
				       + "' must be ungrasped before placing tool '" 
				       + toolDescription->getName() + "' in proximity"));
    return; // already in proximity (& grasped)
  }

  // nothing grasped
  toolProximity[manipIndex] = true;
  tools[manipIndex] = toolDescription;

}


void TestRobot::removeToolFromProximity(Int manipIndex)
{
  if (toolGrasped[manipIndex])
    throw std::logic_error(Exception(String("cannot remove tool '") + tools[manipIndex]->getName()
				     + "' from proximity while is it being grasped"));
  toolProximity[manipIndex] = false;
}



ref<robot::ControlInterface> TestRobot::getControlInterface(String interfaceName) throw(std::invalid_argument)
{
  if (interfaceName == "platformPosition") {
    ref<TestRobot> self(this);
    bool mobilePlatform = getRobotDescription()->platform()->isMobile();
    return ref<robot::ControlInterface>(NewObj PlatformControlInterface(interfaceName,"PlatformPositionControl",self,mobilePlatform));
  }
  
  if (interfaceName.substr(0,19) == "manipulatorPosition") {
    Int n = base::stringToInt(interfaceName.substr(19));
    if ((n<1) || (n > getRobotDescription()->manipulators().size())) 
      throw std::invalid_argument(Exception(String("manipulator index ")+base::intToString(n)+" out of range"));
    ref<TestRobot> self(this);
    return ref<robot::ControlInterface>(NewObj ManipulatorControlInterface(interfaceName,"JointPositionControl",self,n-1));
  }

  if (interfaceName.substr(0,19) == "manipulatorToolGrip") {
    Int n = base::stringToInt(interfaceName.substr(19));
    if ((n<1) || (n > getRobotDescription()->manipulators().size())) 
      throw std::invalid_argument(Exception(String("manipulator index ")+base::intToString(n)+" out of range"));
    ref<TestRobot> self(this);
    return ref<robot::ControlInterface>(NewObj ToolControlInterface(interfaceName,"ToolGripControl",self,n-1,true));
  }

  if (interfaceName.substr(0,12) == "toolPosition") {
    Int n = base::stringToInt(interfaceName.substr(12));
    if ((n<1) || (n > getRobotDescription()->manipulators().size())) 
      throw std::invalid_argument(Exception(String("manipulator index ")+base::intToString(n)+" out of range"));
    ref<TestRobot> self(this);
    return ref<robot::ControlInterface>(NewObj ToolControlInterface(interfaceName,"JointPositionControl",self,n-1,false));
  }

  if (interfaceName.substr(0,20) == "manipulatorProximity") {
    Int n = base::stringToInt(interfaceName.substr(20));
    if ((n<1) || (n > getRobotDescription()->manipulators().size())) 
      throw std::invalid_argument(Exception(String("manipulator index ")+base::intToString(n)+" out of range"));
     
  }
  
  throw std::invalid_argument(Exception(String("unknown interface ")+interfaceName));
}
