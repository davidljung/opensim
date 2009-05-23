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

  $Id: TestBasicEnvironment.cpp 1110 2004-09-27 22:04:18Z jungd $

****************************************************************************/

#include <robot/sim/TestBasicEnvironment>

#include <base/Externalizer>
#include <base/externalization_error>
#include <base/Application>
#include <base/VFile>

using robot::sim::TestBasicEnvironment;

using base::Externalizer;
using base::externalization_error;
using base::Orient;
using base::Dimension3;
using base::Application;
using base::VFile;
using base::PathName;
using base::dom::DOMNode;
using base::dom::DOMElement;
using robot::TestRobot;
using robot::sim::BasicEnvironment;



TestBasicEnvironment::TestBasicEnvironment(ref<base::VFileSystem> fs, ref<base::Cache> cache, const String& name)
  : BasicEnvironment(fs, cache, name)
{
  Logln("Warning: class TestBasicEnvironment will be deprecated in favour of SimulatedBasicEnvironment in future");
}


ref<robot::Robot> TestBasicEnvironment::addRobot(ref<const robot::RobotDescription> robotDescription,
                                                 const base::Point3& position,
                                                 const base::Orient& orientation,
                                                 bool anchored)
{
  ref<TestRobot> robot(NewObj TestRobot(robotDescription,
                       position, orientation));
  robots.push_back(robot);
  return robot;
}


void TestBasicEnvironment::removeRobot(ref<robot::Robot> robot)
{
  if (instanceof(*robot, TestRobot)) {
    ref<TestRobot> testRobot( narrow_ref<TestRobot>(robot));
    robots.remove(testRobot);
  }
  else
    throw std::invalid_argument(Exception("unknown robot"));
}



ref<robot::Robot> TestBasicEnvironment::getRobot(String name)
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





ref<BasicEnvironment::Tool> TestBasicEnvironment::addTool(ref<const robot::ToolDescription> toolDescription,
                                                          const base::Point3& position,
                                                          const base::Orient& orientation)
{
  ref<Tool> tool(NewObj Tool(toolDescription->getName(),toolDescription,
                             position, orientation));
  tools.push_back(tool);
  return tool;
}


void TestBasicEnvironment::removeTool(ref<BasicEnvironment::Tool> tool)
{
  tools.remove(tool);
}


void TestBasicEnvironment::placeToolInProximity(ref<Tool> tool, ref<Robot> robot, Int manipulatorIndex)
{
   if (instanceof(*robot, TestRobot)) {
     ref<const ToolDescription> toolDescription(tool->getToolDescription());

    ref<TestRobot> testRobot( narrow_ref<TestRobot>(robot));
    testRobot->placeToolInProximity(toolDescription, manipulatorIndex);
  }
  else
    throw std::invalid_argument(Exception("unknown robot"));
}



ref<BasicEnvironment::Obstacle> TestBasicEnvironment::addBoxObstacle(base::Dimension3 dim,
                                                                     const base::Point3& position,
                                                                     const base::Orient& orientation,
                                                                     const String& name)
{
  ref<Obstacle> obstacle(NewObj Obstacle(dim,position,orientation));
  obstacles.push_back(obstacle);
  return obstacle;
}


ref<BasicEnvironment::Obstacle> TestBasicEnvironment::addSphereObstacle(Real radius,
                                                                        const base::Point3& position,
                                                                        const base::Orient& orientation,
                                                                        const String& name)
{
  ref<Obstacle> obstacle(NewObj Obstacle(radius,position,orientation));
  obstacles.push_back(obstacle);
  return obstacle;
}


void TestBasicEnvironment::removeObstacle(ref<BasicEnvironment::Obstacle> obstacle)
{
  obstacles.remove(obstacle);
}



void TestBasicEnvironment::preSimulate()
{
}


void TestBasicEnvironment::simulateForSimTime(const base::Time& dt)
{
}


bool TestBasicEnvironment::formatSupported(const String format, Real version, ExternalizationType type) const
{
  return ( (format=="xml") && (version==1.0) );
}


void TestBasicEnvironment::externalize(base::Externalizer& e, const String format, Real version)
{
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
      RobotList::const_iterator rend = robots.end();
     if (r != rend)
       e.appendComment(envElem,"robots   (description, position, and orientation (Quat) )");
      while (r != rend) {
        ref<TestRobot> robot(*r);

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

        ++r;
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

          Point3& position(tool->getPosition());
          DOMElement* posElem = e.createElement("position",false);
          e.appendText(posElem, e.toString(position) );
          e.appendNode(toolElem, posElem);
          e.appendBreak(toolElem);

          Orient& orientation(tool->getOrientation());
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
        ref<Obstacle> obstacle(*o);

        DOMElement* obstElem = e.createElement("obstacle");

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
          e.appendBreak(obstElem);
        }
        else if (obstacle->type == Obstacle::SphereObstacle) {
          e.setElementAttribute(obstElem,"type","sphere");
          DOMElement* rElem = e.createElement("radius",false);
          e.appendText(rElem, base::realToString(obstacle->radius));
          e.appendBreak(obstElem);
          e.appendNode(obstElem,rElem);
        }
        else
          throw externalization_error(Exception("unsupported obstacle type"));

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
          if (v.size() != 4)
            throw externalization_error(Exception("orientation must be a Quat (4 elements)"));
          Orient orientation(v, Orient::Quat);

          ref<RobotDescription> rd(NewObj RobotDescription());
          rd->externalize(e, format, version);

          addRobot(rd, position, orientation);

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
          if (v.size() != 4)
            throw externalization_error(Exception("orientation must be a Quat (4 elements)"));
          Orient orientation(v, Orient::Quat);

          ref<ToolDescription> td(NewObj ToolDescription());
          td->externalize(e, format, version);

          addTool(td, position, orientation);

          toolElem = e.getFirstChildElement(envElem, "tool",false);
        }


        // read in obstacles
        DOMElement* obstElem = e.getFirstChildElement(envElem, "obstacle",false);
        while (obstElem) {
          String name( e.getDefaultedElementAttribute(envElem, "name", "obstacle") );

          // read in position & orientation
          DOMElement* posElem = e.getFirstChildElement(obstElem, "position");
          String posText = e.getContainedText(posElem);
          Point3 position( e.toVector3(posText) );

          DOMElement* orientElem = e.getFirstChildElement(obstElem, "orientation");
          String orientText = e.getContainedText(orientElem);
          Vector v( e.toVector(orientText,true) );
          if (v.size() != 4)
            throw externalization_error(Exception("orientation must be a Quat (4 elements)"));
          Orient orientation(v, Orient::Quat);

          // get obstacle parameters & construct obstacles
          ref<Obstacle> obstacle;
          Obstacle::ObstacleType type;
          if ( e.getElementAttribute(obstElem, "type") == "box" ) {
            type = Obstacle::BoxObstacle;

            DOMElement* dElem = e.getFirstChildElement(obstElem, "dimensions");
            String dimText = e.getContainedText(dElem);
            Dimension3 dims( e.toVector3( dimText ) );

            addBoxObstacle(dims,position,orientation,name);
          }
          else if ( e.getElementAttribute(obstElem, "type") == "sphere" ) {
            type = Obstacle::SphereObstacle;

            DOMElement* rElem = e.getFirstChildElement(obstElem, "radius",false);
            Real radius=1.0;
            if (rElem)
              radius = base::stringToReal( e.getContainedText(rElem) );

            addSphereObstacle(radius, position,orientation,name);
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

