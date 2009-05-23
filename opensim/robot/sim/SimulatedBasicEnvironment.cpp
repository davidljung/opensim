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

  $Id: SimulatedBasicEnvironment.cpp 1112 2004-09-27 22:06:45Z jungd $

****************************************************************************/

#include <robot/sim/SimulatedBasicEnvironment>

// OSG
#include <osg/Vec4>
#include <osg/Group>
#include <osg/Notify>
#include <osg/Fog>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/Material>

#include <base/Externalizer>
#include <base/externalization_error>
#include <base/Application>
#include <base/VFile>
#include <physics/Shape>
#include <physics/Box>
#include <physics/Sphere>
#include <physics/Material>
#include <physics/Solid>
#include <physics/ODESolidSystem>
#include <physics/ConstraintGroup>
#include <physics/FixedConstraint>


using robot::sim::SimulatedBasicEnvironment;

using base::Externalizer;
using base::externalization_error;
using base::Orient;
using base::Dimension3;
using base::Application;
using base::VFile;
using base::PathName;
using base::dom::DOMNode;
using base::dom::DOMElement;
using physics::Shape;
using physics::Box;
using physics::Sphere;
using physics::Material;
using physics::Solid;
using physics::SolidSystem;
using physics::ODESolidSystem;
using physics::ConstraintGroup;
using physics::FixedConstraint;
using physics::Collidable;
using physics::CollisionCuller;
using robot::sim::BasicEnvironment;
using robot::sim::SimulatedRobot;
using robot::sim::SimulatedToolDescription;



SimulatedBasicEnvironment::SimulatedBasicEnvironment(ref<base::VFileSystem> fs, ref<base::Cache> cache, const String& name, bool dynamic)
  : BasicEnvironment(fs, cache, name), OSGWorld(fs, cache, name), dynamic(dynamic)
{
  construct();
}


/// \todo clone anchored state of robot platform
SimulatedBasicEnvironment::SimulatedBasicEnvironment(const SimulatedBasicEnvironment& e)
  : BasicEnvironment(e), OSGWorld(e), dynamic(e.dynamic)
{
  construct();

  // duplicate robots
  RobotList::const_iterator r = e.robots.begin();
  RobotAnchoredList::const_iterator ra = robotsAnchored.begin();
  RobotList::const_iterator rend = e.robots.end();
  while (r != rend) {
    ref<SimulatedRobot> robot(*r);
    addRobot(robot->getRobotDescription(),
             robot->getPosition(), robot->getOrientation(),
             *ra);
    ++r;
    ++ra;
  }

  // duplicate tools
  ToolList::const_iterator t = e.tools.begin();
  ToolList::const_iterator tend = e.tools.end();
  while (t != tend) {
    ref<const SolidTool> solidTool(narrow_ref<const SolidTool>(*t));
    addTool(solidTool->getToolDescription(), solidTool->getPosition(), solidTool->getOrientation());
    ++t;
  }


  // duplicate obstacles
  //  (should be an easier way sometime...)
  ObstacleList::const_iterator o = e.obstacles.begin();
  ObstacleList::const_iterator oend = e.obstacles.end();
  while (o != oend) {
    ref<SolidObstacle> obst(*o);
    ref<const Solid> solid(obst->solid);
    ref<const Shape> shape(solid->getShape());

    if (instanceof(*shape, const Box)) {
      ref<const Box> box(narrow_ref<const Box>(shape));
      addBoxObstacle(box->dimensions(), solid->getPosition(), solid->getOrientation(),
                     solid->getMaterial(), solid->getName());
    }
    else if (instanceof(*shape, const Sphere)) {
      ref<const Sphere> sphere(narrow_ref<const Sphere>(shape));
      addSphereObstacle(sphere->radius(), solid->getPosition(), solid->getOrientation(),
                        solid->getMaterial(), solid->getName());
    }
    else
      throw std::runtime_error(Exception("unhandled obstacle shape"));

    ++o;
  }

}


void SimulatedBasicEnvironment::setDynamic(bool enabled)
{
  dynamic = enabled;
  RobotList::const_iterator r = robots.begin();
  RobotList::const_iterator rend = robots.end();
  while (r != rend) {
    (*r)->setDynamic(enabled);
    ++r;
  }

  ToolList::const_iterator t = tools.begin();
  ToolList::const_iterator tend = tools.end();
  while (t != tend) {
    ref<const SolidTool> solidTool(narrow_ref<const SolidTool>(*t));
    solidTool->simTool->setDynamic(enabled);
    ++t;
  }

}



ref<robot::Robot> SimulatedBasicEnvironment::addRobot(ref<const robot::RobotDescription> robotDescription,
                                                      const base::Point3& position,
                                                      const base::Orient& orientation,
                                                      bool anchored)
{
  ref<SimulatedRobot> robot(NewObj SimulatedRobot(robotDescription,
                                                  position, orientation,
                                                  system, dynamic));
  Assert(collidables);
  ref<Collidable> robotCollidable( robot->createCollidable() );
  collidables->addCollidable( robotCollidable );

  if (anchored) {
    ref<Solid> platformSolid(robot->getPlatformSolid()); // get robot's platform Solid
    ref<FixedConstraint> fixed(system->createFixedConstraint()); // create a fixed constraint
    cgroup->addConstraint(fixed);
    fixed->attach(ground, platformSolid); // attach platform to ground

    // tell collision detector not to collide them
    ref<Collidable> platformBase( robotCollidable->findNamed("body") );
    if (platformBase)
      collisionCuller->collisionEnable(false,groundCollidable,platformBase);
  }

  robots.push_back(robot);
  robotsAnchored.push_back(anchored);
  return robot;
}

/// \todo remove solid & collidable
void SimulatedBasicEnvironment::removeRobot(ref<robot::Robot> robot)
{
  if (instanceof(*robot, SimulatedRobot)) {
    ref<SimulatedRobot> simRobot( narrow_ref<SimulatedRobot>(robot));
    robots.remove(simRobot);
  }
  else
    throw std::invalid_argument(Exception("unknown robot"));
}



ref<robot::Robot> SimulatedBasicEnvironment::getRobot(String name)
{
  RobotList::const_iterator r = robots.begin();
  RobotList::const_iterator rend = robots.end();
  while (r != rend) {
    ref<Robot> robot(*r);
    if (robot->isDescriptionProvided()) {
      ref<const RobotDescription> rd(robot->getRobotDescription());
      if (rd->getName() == name)
        return robot;
    }
    ++r;
  }
  throw std::invalid_argument(Exception("no robot named "+name+" present in environment"));
}





ref<BasicEnvironment::Tool> SimulatedBasicEnvironment::addTool(ref<const robot::ToolDescription> toolDescription,
                                                               const base::Point3& position,
                                                               const base::Orient& orientation)
{
  ref<SimulatedTool> simTool(NewObj SimulatedTool(toolDescription,
                                                  position, orientation,
                                                  system, dynamic));
  Assert(simTool);
  Assert(collidables);

  ref<Collidable> toolCollidable( simTool->createCollidable() );
  collidables->addCollidable( toolCollidable );


  ref<SolidTool> tool(NewObj SolidTool(toolDescription->getName(),toolDescription,
                                       position, orientation, simTool));
  tools.push_back(tool);
  return tool;
}

/// \todo remove solid
void SimulatedBasicEnvironment::removeTool(ref<BasicEnvironment::Tool> tool)
{
  tools.remove(tool);
}


void SimulatedBasicEnvironment::placeToolInProximity(ref<Tool> tool, ref<Robot> robot, Int manipulatorIndex)
{
   if (instanceof(*robot, SimulatedRobot)) {
     if (instanceof(*tool, SolidTool)) {
       ref<SimulatedRobot> simRobot( narrow_ref<SimulatedRobot>(robot));
       ref<SolidTool> solidTool( narrow_ref<SolidTool>(tool) );
       ref<SimulatedTool> simTool( solidTool->simTool );

       simRobot->placeToolInProximity(simTool, manipulatorIndex);
     }
     else
       throw std::invalid_argument(Exception("unknown tool"));
  }
  else
    throw std::invalid_argument(Exception("unknown robot"));
}




ref<BasicEnvironment::Obstacle> SimulatedBasicEnvironment::addBoxObstacle(base::Dimension3 dim,
                                                                          const base::Point3& position,
                                                                          const base::Orient& orientation,
                                                                          const String& name)
{
  ref<Material> material(NewObj physics::Material());
  material->setBaseColor(gfx::Color3(0.7,0.4,0.8)); // default
  material->setDensity(1.0); // default

  return addBoxObstacle(dim,position,orientation,material,name);
}


ref<BasicEnvironment::Obstacle> SimulatedBasicEnvironment::addSphereObstacle(Real radius,
                                                                             const base::Point3& position,
                                                                             const base::Orient& orientation,
                                                                             const String& name)
{
  ref<Material> material(NewObj physics::Material());
  material->setBaseColor(gfx::Color3(0.7,0.4,0.8)); // default
  material->setDensity(1.0); // default

  return addSphereObstacle(radius,position,orientation,material,name);
}


ref<BasicEnvironment::Obstacle> SimulatedBasicEnvironment::addBoxObstacle(base::Dimension3 dim,
                                                                          const base::Point3& position,
                                                                          const base::Orient& orientation,
                                                                          ref<const physics::Material> material,
                                                                          const String& name)
{
  ref<Box> box(NewObj Box(dim.x,dim.y,dim.z));
  ref<Solid> solid(system->createSolid(box, material));
  solid->setName( (name=="")?String("box obstacle"):name);
  system->addSolid(solid);
  solid->setEnabled(dynamic);
  solid->setPosition(position);
  solid->setOrientation(orientation.getQuat4());

  ref<Collidable> obstCollidable( solid->createCollidable() );
  collidables->addCollidable( obstCollidable );

  ref<SolidObstacle> obstacle(NewObj SolidObstacle(dim,position,orientation,solid));
  obstacles.push_back(obstacle);
  return obstacle;
}


ref<BasicEnvironment::Obstacle> SimulatedBasicEnvironment::addSphereObstacle(Real radius,
                                                                             const base::Point3& position,
                                                                             const base::Orient& orientation,
                                                                             ref<const physics::Material> material,
                                                                             const String& name)
{
  ref<Sphere> sphere(NewObj Sphere(radius));
  ref<Solid> solid(system->createSolid(sphere, material));
  solid->setName( (name=="")?String("sphere obstacle"):name);
  system->addSolid(solid);
  solid->setEnabled(dynamic);
  solid->setPosition(position);
  solid->setOrientation(orientation.getQuat4());

  ref<Collidable> obstCollidable( solid->createCollidable() );
  collidables->addCollidable( obstCollidable );

  ref<SolidObstacle> obstacle(NewObj SolidObstacle(radius,position,orientation,solid));
  obstacles.push_back(obstacle);
  return obstacle;
}



void SimulatedBasicEnvironment::removeObstacle(ref<BasicEnvironment::Obstacle> obstacle)
{
  if (instanceof(*obstacle, SolidObstacle))
    obstacles.remove( narrow_ref<SolidObstacle>(obstacle) );
  else
    throw std::invalid_argument(Exception("unknown Obstacle"));
}

/// \TODO shouldn't this remove the old Solid from the system an
///  add the new one (and also create a new Collidable from the new Solid)???
void SimulatedBasicEnvironment::setObstacleColor(ref<BasicEnvironment::Obstacle> obstacle, const gfx::Color3& color)
{
  if (instanceof(*obstacle, SolidObstacle)) {

    ref<SolidObstacle> solidobst( narrow_ref<SolidObstacle>(obstacle) );
    ref<Solid> s( solidobst->solid );
    // create a Solid identical to the old but with a different material color
    //  and substitute one for the other
    ref<Material> newmat(NewObj Material(*s->getMaterial()));
    newmat->setBaseColor(color);
    ref<Solid> newsolid(system->createSolid(s->getShape(), newmat));
    newsolid->setName( s->getName() );
    solidobst->solid = newsolid;
  }
  else
    throw std::invalid_argument(Exception("unknown Obstacle"));
}


void SimulatedBasicEnvironment::setObstacleDensity(ref<BasicEnvironment::Obstacle> obstacle, Real density)
{
  if (instanceof(*obstacle, SolidObstacle)) {

    ref<SolidObstacle> solidobst( narrow_ref<SolidObstacle>(obstacle) );
    ref<Solid> s( solidobst->solid );
    // create a Solid identical to the old but with a different material density
    //  and substitute one for the other
    ref<Material> newmat(NewObj Material(*s->getMaterial()));
    newmat->setDensity(density);
    ref<Solid> newsolid(system->createSolid(s->getShape(), newmat));
    newsolid->setName( s->getName() );
    solidobst->solid = newsolid;
  }
  else
    throw std::invalid_argument(Exception("unknown Obstacle"));
}


ref<physics::Solid> SimulatedBasicEnvironment::findObstacle(ref<BasicEnvironment::Obstacle> obstacle)
{
  if (instanceof(*obstacle, SolidObstacle)) {
    ref<SolidObstacle> so( narrow_ref<SolidObstacle>(obstacle) );
    return so->solid;
  }
  else
    return ref<physics::Solid>(0);
}


void SimulatedBasicEnvironment::construct()
{
  // Rigid-body physics simulation system
  system = ref<SolidSystem>(NewNamedObj("BasicEnvironmentSolidSystem") ODESolidSystem());
  system->setGravity(base::Vector3(0,0,-9.8));
  //system->setGravity(base::Vector3(0,0,0));//!!!
  system->setParameter("ERP",0.2);
  system->setParameter("CFM",1e-7);

  collisionCuller = system->getCollisionCuller();
  collidables = collisionCuller->createCollidableGroup();
  collidables->setName(className()+" collidables");

  // create a constraint group for environment wide constraints (e.g. fixed constraints for fixing robots to the ground)
  cgroup = ref<ConstraintGroup>(system->createConstraintGroup());

  // make a big flat 'plane' for the 'ground'
  const Real t = 1.0; // thickness
  ref<Box> groundBox(NewObj Box(150,150,t));
  ref<physics::Material> groundMaterial(NewObj physics::Material());
  groundMaterial->setDensity(0.01);
  //groundMaterial->setBaseColor(gfx::Color3(244.0/256.0,164.0/256.0,96.0/256.0)); // 'brown'
  groundMaterial->setBaseColor(gfx::Color3(1.0,1.0,1.0)); // 'white'
  //!!!groundMaterial->setSurfaceAppearance(PathName("images/LlanoTex.jpg"));
  ground = system->createSolid(groundBox, groundMaterial);
  ground->setName("ground");
  system->setGround(ground, Point3(0,0,-(t/2.0))); // system treats the ground Solid as special - it's anchored to the world frame.

  groundCollidable = ground->createCollidable();
  collidables->addCollidable( groundCollidable );

  system->setCollidable(collidables);
}



void SimulatedBasicEnvironment::checkTools()
{
  // check each tool for proximity with each robot
  RobotList::const_iterator r = robots.begin();
  RobotList::const_iterator rend = robots.end();
  while (r != rend) {
    ref<SimulatedRobot> robot(*r);

    ToolList::const_iterator t = tools.begin();
    ToolList::const_iterator tend = tools.end();
    while (t != tend) {
      ref<SimulatedTool> simTool( narrow_ref<SolidTool>(*t)->simTool );

      robot->checkProximity(simTool);

      ++t;
    }

    ++r;
  }

}



void SimulatedBasicEnvironment::preSimulate()
{
  system->preSimulate();
  checkTools();
}


void SimulatedBasicEnvironment::simulateForSimTime(const base::Time& dt)
{
  if (dynamic)
    system->simulateForSimTime(dt);
  else
    system->simulateForSimTime(0); // just do collisions and update Visual (if any)
  checkTools();
}



osg::Node* SimulatedBasicEnvironment::createOSGVisual(Visual::Attributes visualAttributes) const
{
  if ((rootnode!=0) && (attributes==visualAttributes))
    return &(*rootnode);

  rootnode = NewObj osg::Group;
  rootnode->setName("root");

  // create physics system graphics
  osg::Node* g = system->createOSGVisual( visualAttributes
#ifdef DEBUG
//					 | gfx::Visual::ShowAxes
                                         //| gfx::Visual::ShowCollisionModel
                                         //| gfx::Visual::ShowNormals
#endif
                                        );

  rootnode->addChild(g);


  // Set state of rootnode (fog etc.)
  osg::StateSet* pState = NewObj osg::StateSet;
  //    pState->setMode(GL_LIGHTING,osg::StateAttribute::ON);
  osg::Vec4 fogColor(0.65f,0.65f,0.65f,1.0f);
  osg::Fog* fog = NewObj osg::Fog;
  fog->setMode(osg::Fog::LINEAR);
  fog->setDensity(0.1f);
  fog->setStart(100.0f);
  fog->setEnd(10000.0f - 100.0f); // (max view dist)
  fog->setColor(fogColor);
  pState->setAttributeAndModes(fog,osg::StateAttribute::ON);
  rootnode->setStateSet(pState);

  return &(*rootnode);
}




bool SimulatedBasicEnvironment::formatSupported(const String format, Real version, ExternalizationType type) const
{
  return ( (format=="xml") && (version==1.0) );
}


void SimulatedBasicEnvironment::externalize(base::Externalizer& e, String format, Real version)
{
  if (format=="") format="xml";

  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" v"+base::realToString(version)+" unsupported"));

  if (format == "xml") {

    if (e.isOutput()) {

      DOMElement* envElem = e.createElement("environment");
      e.setElementAttribute(envElem,"type","basic");

      e.pushContext(envElem);

      e.appendBreak(envElem);

      // externalize the robots
      RobotList::const_iterator r = robots.begin();
      RobotAnchoredList::const_iterator ra = robotsAnchored.begin();
      RobotList::const_iterator rend = robots.end();
      if (r != rend)
        e.appendComment(envElem,"robots   (description, position, and orientation (Quat) )");
        while (r != rend) {
          ref<SimulatedRobot> robot(*r);

          if (robot->isDescriptionProvided())
            robot->getRobotDescription()->externalize(e, format, version);
          else
            throw externalization_error(Exception("can't externalize a Robot with no description"));

          DOMElement* robotElem = e.lastAppendedElement();
          Point3 position(robot->getPosition());
          DOMElement* posElem = e.createElement("position",false);
          e.appendText(posElem, e.toString(position) );
          e.appendNode(robotElem, posElem);
          e.appendBreak(robotElem);

          Orient orientation(robot->getOrientation());
          DOMElement* orientElem = e.createElement("orientation",false);
          Vector v( orientation.getVector(Orient::Quat) );
          e.appendText(orientElem, e.toString(v,true));
          e.appendNode(robotElem, orientElem);
          e.appendBreak(robotElem);

          if (*ra)
            e.setElementAttribute(robotElem, "anchored", "true");

          ++r;
          ++ra;
          e.appendBreak(envElem);
      }
      e.appendBreak(envElem);


      // externalize tools
      ToolList::const_iterator t = tools.begin();
      ToolList::const_iterator tend = tools.end();
      if (t != tend)
        e.appendComment(envElem,"tools   (position, orientation (Quat) and description)");
        while (t != tend) {
          ref<Tool> tool(*t);
          tool->getToolDescription()->externalize(e, format, version);

          DOMElement* toolElem = e.lastAppendedElement();

          Point3 position(tool->getPosition());
          DOMElement* posElem = e.createElement("position",false);
          e.appendText(posElem, e.toString(position) );
          e.appendNode(toolElem, posElem);
          e.appendBreak(toolElem);

          Orient orientation(tool->getOrientation());
          DOMElement* orientElem = e.createElement("orientation",false);
          Vector v( orientation.getVector(Orient::Quat) );
          e.appendText(orientElem, e.toString(v,true));
          e.appendNode(toolElem, orientElem);
          e.appendBreak(toolElem);

          ++t;
          e.appendBreak(envElem);
      }
      e.appendBreak(envElem);


      // externalize obstacles
      ObstacleList::const_iterator o = obstacles.begin();
      ObstacleList::const_iterator oend = obstacles.end();
      if (o != oend)
        e.appendComment(envElem,"obstacles (position, orientation (Quat) and description)");
      while (o != oend) {
        ref<SolidObstacle> obstacle(*o);

        DOMElement* obstElem = e.createElement("obstacle");
        e.setElementAttribute(obstElem,"name",obstacle->solid->getName());

        Point3& position(obstacle->position);
        DOMElement* posElem = e.createElement("position",false);
        e.appendText(posElem, e.toString(position) );
        e.appendNode(obstElem, posElem);
        e.appendBreak(obstElem);

        Orient& orientation(obstacle->orientation);
        DOMElement* orientElem = e.createElement("orientation",false);
        Vector v( orientation.getVector(Orient::Quat) );
        e.appendText(orientElem, e.toString(v,true));
        e.appendNode(obstElem, orientElem);
        e.appendBreak(obstElem);

        if (obstacle->type == Obstacle::BoxObstacle) {
          e.setElementAttribute(obstElem,"type","box");
          DOMElement* dElem = e.createElement("dimensions",false);
          e.appendText( dElem, e.toString(obstacle->dims) );
          e.appendNode(obstElem,dElem);
        }
        else if (obstacle->type == Obstacle::SphereObstacle) {
          e.setElementAttribute(obstElem,"type","sphere");
          DOMElement* rElem = e.createElement("radius",false);
          e.appendText(rElem, base::realToString(obstacle->radius));
          e.appendNode(obstElem,rElem);
        }
        else
          throw externalization_error(Exception("unsupported obstacle type"));

        e.pushContext(obstElem);
        ref<Material> mat(NewObj Material(*obstacle->solid->getMaterial()));
        mat->externalize(e, format, version);
        e.popContext();


        e.appendNode(envElem, obstElem);

        ++o;
        e.appendBreak(envElem);
      }


      e.popContext();

      e.appendElement(envElem);

    }
    else { // input

      // first clear the current environment
      while (!robots.empty()) removeRobot( robots.front() );
      while (!tools.empty()) removeTool( tools.front() );
      while (!obstacles.empty()) removeObstacle( obstacles.front() );
      construct();

      // now load the new one
      DOMNode* context = e.context();

      DOMElement* envElem = e.getFirstElement(context, "environment");
      // handle link
      String link = e.getElementAttribute(envElem,"link",false);
      if (link != "") {

        ref<VFile> linkFile = Application::getInstance()->universe()->cache()->findFile(link,e.getArchivePath());
        load(linkFile,format,version);
      }
      else {

        if ( e.getElementAttribute(envElem, "type") != "basic" )
          throw externalization_error(Exception("unsupported environment type"));

        e.pushContext(envElem);

        // read in robots
        DOMElement* robotElem = e.getFirstChildElement(envElem, "robot",false);
        while (robotElem) {
          // get position & orientation
          DOMElement* posElem = e.getFirstChildElement(robotElem,"position");
          String posText = e.getContainedText(posElem);
          Point3 position( e.toVector3(posText) );

          DOMElement* orientElem = e.getFirstChildElement(robotElem, "orientation");
          String orientText = e.getContainedText(orientElem);
          Vector v( e.toVector(orientText,true) );
          Orient orientation;
          if (v.size() == 4)
            orientation = Orient(v, Orient::Quat);
          else if (v.size() == 3)
            orientation = Orient(v, Orient::EulerRPY);
          else
            throw externalization_error(Exception("robot orientation must have either 3(EulerRPY) or 4(Quaternion) elements)"));

          // get if robot anchored
          String anchoredString = e.getDefaultedElementAttribute(robotElem, "anchored", "false");
          bool anchored = (anchoredString == "true");
          if (!anchored && (anchoredString != "false"))
            throw externalization_error(Exception("'anchored' attribute of element 'robot' must be either 'true' or 'false'"));

          ref<RobotDescription> rd(newRobotDescription());
          rd->externalize(e, format, version);
          addRobot(rd, position, orientation, anchored);

          robotElem = e.getFirstChildElement(envElem, "robot",false);
        }


        // read in tools
        DOMElement* toolElem = e.getFirstChildElement(envElem, "tool",false);
        while (toolElem) {
          // get position & orientation
          DOMElement* posElem = e.getFirstChildElement(toolElem,"position");
          String posText = e.getContainedText(posElem);
          Point3 position( e.toVector3(posText) );

          DOMElement* orientElem = e.getFirstChildElement(toolElem, "orientation");
          String orientText = e.getContainedText(orientElem);
          Vector v( e.toVector(orientText,true) );
          Orient orientation;
          if (v.size() == 4)
            orientation = Orient(v, Orient::Quat);
          else if (v.size() == 3)
            orientation = Orient(v, Orient::EulerRPY);
          else
            throw externalization_error(Exception("tool orientation must have either 3(EulerRPY) or 4(Quaternion) elements)"));

          ref<SimulatedToolDescription> td(NewObj SimulatedToolDescription());
          td->externalize(e, format, version);

          addTool(td, position, orientation);

          toolElem = e.getFirstChildElement(envElem, "tool",false);
        }


        // read in obstacles
        DOMElement* obstElem = e.getFirstChildElement(envElem, "obstacle",false);
        while (obstElem) {
          String name( e.getDefaultedElementAttribute(obstElem, "name", "obstacle") );

          // read in position & orientation
          DOMElement* posElem = e.getFirstChildElement(obstElem, "position");
          String posText = e.getContainedText(posElem);
          Point3 position( e.toVector3(posText) );

          DOMElement* orientElem = e.getFirstChildElement(obstElem, "orientation");
          String orientText = e.getContainedText(orientElem);
          Vector v( e.toVector(orientText,true) );
          Orient orientation;
          if (v.size() == 4)
            orientation = Orient(v, Orient::Quat);
          else if (v.size() == 3)
            orientation = Orient(v, Orient::EulerRPY);
          else
            throw externalization_error(Exception("obstacle orientation must have either 3(EulerRPY) or 4(Quaternion) elements)"));

          // optional material
          DOMElement* matElem = e.getFirstChildElement(obstElem, "material",false);
          ref<Material> mat(NewObj Material());
          if (matElem) {
            e.pushContext(obstElem);
            mat->externalize(e, format, version);
            e.popContext();
          }

          // get obstacle parameters & construct obstacles
          ref<SolidObstacle> obstacle;
          Obstacle::ObstacleType type;
          if ( e.getElementAttribute(obstElem, "type") == "box" ) {
            type = Obstacle::BoxObstacle;

            DOMElement* dElem = e.getFirstChildElement(obstElem, "dimensions");
            String dimText = e.getContainedText(dElem);
            Dimension3 dims( e.toVector3( dimText ) );

            addBoxObstacle(dims,position,orientation,mat,name);
          }
          else if ( e.getElementAttribute(obstElem, "type") == "sphere" ) {
            type = Obstacle::SphereObstacle;

            DOMElement* rElem = e.getFirstChildElement(obstElem, "radius",false);
            Real radius=1.0;
            if (rElem)
              radius = base::stringToReal( e.getContainedText(rElem) );

            addSphereObstacle(radius, position,orientation,mat,name);
          }
          else
            throw externalization_error(Exception("unsupported obstacle type"));

          e.removeElement(obstElem);

          obstElem = e.getFirstChildElement(envElem, "obstacle",false);
        }

        e.popContext();
      }

      e.removeElement(envElem);

    }

  }

}

