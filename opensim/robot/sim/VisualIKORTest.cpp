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

  $Id: VisualIKORTest.cpp 1116 2004-09-27 22:11:32Z jungd $

****************************************************************************/

#include <robot/sim/VisualIKORTest>

#include <base/Externalizer>
#include <base/externalization_error>
#include <base/Expression>
#include <base/Matrix>
#include <base/VDirectory>
#include <base/VFile>

#include <osg/Group>
#include <osg/Switch>
#include <osg/Geode>
#include <osg/Geometry>
#include <osgText/Font>
#include <osgText/Text>


using robot::sim::VisualIKORTest;

using base::Vector;
using base::Orient;
using base::Time;
using base::PathName;
using base::VFile;
using base::VDirectory;
using base::externalization_error;
using base::Path;
using base::Trajectory;
using base::dom::DOMNode;
using base::dom::DOMElement;
using gfx::Segment3;
using gfx::Color4;
using robot::sim::IKORTest;
using robot::sim::BasicEnvironment;
using robot::control::kinematics::FullSpaceSolver;
using robot::control::kinematics::InverseKinematicsSolver;
using robot::control::kinematics::FullSpaceSolver;
using robot::control::kinematics::IKOR;





VisualIKORTest::VisualIKORTest(ref<base::VFile> testSpecification,
                               ref<base::VFileSystem> fs, ref<base::Cache> cache)
  : IKORTest(testSpecification, fs, cache)
{
}



osg::Vec3Array* VisualIKORTest::newVertexArrayFromLines(const VisualIKORTest::LineSegArray& lines)
{
  osg::Vec3Array& v(*NewObj osg::Vec3Array(lines.size()*2));
  for(Int i=0; i<lines.size(); i++) {
    const Point3& s(lines[i].s);
    const Point3& e(lines[i].e);
    v[2*i].set(s.x,s.y,s.z);
    v[2*i+1].set(e.x,e.y,e.z);
  }
  return &v;
}


osg::Geometry* VisualIKORTest::newGeometryFromLines(const VisualIKORTest::LineSegArray& lines, const gfx::Color4& color)
{
  osg::Geometry* linesGeom = new osg::Geometry();
  osg::Vec3Array* verts(newVertexArrayFromLines(lines));
  linesGeom->setVertexArray(verts);
  osg::Vec4Array& colora(*new osg::Vec4Array(1));
  colora[0].set(color.r, color.g, color.b, color.a);
  linesGeom->setColorArray(&colora);
  linesGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
  linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,lines.size()*2));
  return linesGeom;
}



VisualIKORTest::LineSegArray VisualIKORTest::manipToolAsLines(const KinematicChain& chain, const Vector& q, Int numPlatformLinks)
{
  // create lines between each pair of link origins, except those for the platform
  LineSegArray lines;
//Debugcln(DJ,"q=" << q << "(chain size=" << chain.size() << ")" << std::flush);

  array<Vector> links( chain.getLinkOrigins(q) );
  for(Int l=numPlatformLinks+1; l<links.size(); l++) {
    const Vector& plink(links[l-1]);
    const Vector& link( links[l]);
    lines.push_back(Segment3(Point3(plink[0],plink[1],plink[2]), Point3(link[0],link[1],link[2])));
    // debug!!!!
    lines.push_back(Segment3(Point3(link[0],link[1],link[2]), Point3(link[0],link[1],link[2]+0.05)));
  }

  return lines;
}


VisualIKORTest::LineSegArray VisualIKORTest::platformAsLines(ref<const PlatformDescription> platfDescr,
                                                             const KinematicChain& platfChain,
                                                             const Vector& q, Real steerAngle)
{
  // draw the platform as a square with line-seg wheels
  LineSegArray lines;

  const base::Dimension3 dim(platfDescr->dimensions());
  Vector3 offset(platfDescr->originOffset());

  Vector3 c1(-dim.x/2.0, -dim.y/2.0, 0), c2(-dim.x/2.0, dim.y/2.0, 0),  // corners
          c3(dim.x/2.0, dim.y/2.0, 0), c4(dim.x/2.0, -dim.y/2.0, 0);
  c1-=offset; c2-=offset; c3-=offset; c4-=offset;

  Vector pf_q;
  if (platfChain.dof() > 0) pf_q = vectorRange(q,Range(0,platfChain.dof()));
  Matrix4 pT(base::toMatrix4(platfChain.getForwardKinematics(pf_q)));
  c1 = pT*c1; c2 = pT*c2; c3 = pT*c3; c4 = pT*c4; // transform to world frame

  lines.push_back(Segment3(c1,c2));
  lines.push_back(Segment3(c2,c3));
  lines.push_back(Segment3(c3,c4));
  lines.push_back(Segment3(c4,c1));
/*
  // the wheels
  const Real d = 0.4; // wheel diameter (length of 2D proj)
  const Real L = platfDescr.L();
  const Real W = platfDescr.W();

  // back
  Vector3 bwlb(-L-d/2.0, 0.8*(dim.y/2.0), -offset.z);  // back wheel left back point
  Vector3 bwlf(-L+d/2.0, 0.8*(dim.y/2.0), -offset.z);
  Vector3 bwrb(-L-d/2.0, 0.8*(-dim.y/2.0), -offset.z);
  Vector3 bwrf(-L+d/2.0, 0.8*(-dim.y/2.0), -offset.z);

  bwlb = pT*bwlb; bwlf = pT*bwlf; bwrb = pT*bwrb; bwrf = pT*bwrf; // to world
  lines.push_back(Segment3(bwlb,bwlf));
  lines.push_back(Segment3(bwrb,bwrf));


  // front
  Real dx = (d/2.0)*cos(steerAngle);
  Real dy = (d/2.0)*sin(steerAngle);
  Vector3 fwlb(-L+W-dx, 0.8*(dim.y/2.0)+dy, -offset.z); // front wheel left back point
  Vector3 fwlf(-L+W+dx, 0.8*(dim.y/2.0)-dy, -offset.z);
  Vector3 fwrb(-L+W-dx, 0.8*(-dim.y/2.0)+dy, -offset.z);
  Vector3 fwrf(-L+W+dx, 0.8*(-dim.y/2.0)-dy, -offset.z);

  fwlb = pT*fwlb; fwlf = pT*fwlf; fwrb = pT*fwrb; fwrf = pT*fwrf; // to world
  lines.push_back(Segment3(fwlb,fwlf));
  lines.push_back(Segment3(fwrb,fwrf));
*/

  return lines;
}


/// helper to translate a segment
static inline gfx::Segment3 translate(const gfx::Segment3& s, const base::Vector3& t)
{
  gfx::Segment3 ts(s); ts.s += t; ts.e += t;
  return ts;
}


VisualIKORTest::LineSegArray VisualIKORTest::obstaclesAsLines(ref<const BasicEnvironment> env)
{
  LineSegArray lines;

  for(Int i=0; i<env->numObstacles(); i++) {
    ref<const BasicEnvironment::Obstacle> obst(env->getObstacle(i));

    switch (obst->type) {

    case BasicEnvironment::Obstacle::BoxObstacle: {
      const base::Dimension3& dim(obst->dims);
      Vector3 c1(-dim.x/2.0, -dim.y/2.0, -dim.z/2.0), c2(-dim.x/2.0, dim.y/2.0, -dim.z/2.0),  // corners
              c3(dim.x/2.0, dim.y/2.0, -dim.z/2.0), c4(dim.x/2.0, -dim.y/2.0, -dim.z/2.0);
      Vector3 c5(-dim.x/2.0, -dim.y/2.0, dim.z/2.0), c6(-dim.x/2.0, dim.y/2.0, dim.z/2.0),
              c7(dim.x/2.0, dim.y/2.0, dim.z/2.0), c8(dim.x/2.0, -dim.y/2.0, dim.z/2.0);

      // transform
      Matrix4 T( obst->orientation.getRotationMatrix3() );
      T.setTranslationComponent( obst->position );
      c1 = T*c1; c2 = T*c2; c3 = T*c3; c4 = T*c4;
      c5 = T*c5; c6 = T*c6; c7 = T*c7; c8 = T*c8;

      // bottom
      lines.push_back(Segment3(c1,c2));
      lines.push_back(Segment3(c2,c3));
      lines.push_back(Segment3(c3,c4));
      lines.push_back(Segment3(c4,c1));

      // top
      lines.push_back(Segment3(c5,c6));
      lines.push_back(Segment3(c6,c7));
      lines.push_back(Segment3(c7,c8));
      lines.push_back(Segment3(c8,c5));

      // sides
      lines.push_back(Segment3(c1,c5));
      lines.push_back(Segment3(c2,c6));
      lines.push_back(Segment3(c3,c7));
      lines.push_back(Segment3(c4,c8));
    } break;

    case BasicEnvironment::Obstacle::SphereObstacle: {
      // compute segments for 6 circles; 3 in each of the xy, yz and xz planes & 3 rotated
      Transform rot(Orient(consts::Pi/4.0, consts::Pi/4.0, 0)); // roll 45, pitch 45

      Real r = obst->radius;
      Point3 c = obst->position;
      Real inc = consts::Pi/32.0;
      for(Real a=inc; a<=2*consts::Pi; a+=inc) {
        Real c1 = r*cos(a-inc);
        Real s1 = r*sin(a-inc);
        Real c2 = r*cos(a);
        Real s2 = r*sin(a);

        Point3 xy1( c1, s1, 0);
        Point3 xy2( c2, s2, 0);
        Segment3 xy(xy1, xy2);
        lines.push_back(translate(xy,c));
        xy.transform(rot);
        lines.push_back(translate(xy,c));
        Point3 yz1( 0, c1, s1);
        Point3 yz2( 0, c2, s2);
        Segment3 yz(yz1, yz2);
        lines.push_back(translate(yz,c));
        yz.transform(rot);
        lines.push_back(translate(yz,c));
        Point3 xz1( c1, 0, s1);
        Point3 xz2( c2, 0, s2);
        Segment3 xz(xz1,xz2);
        lines.push_back(translate(xz,c));
        xz.transform(rot);
        lines.push_back(translate(xz,c));

      }

    } break;

    default:
      Logfln("warning: unknown obstacle type (" << obst->type << ") not displayed");
    }

  }

  return lines;
}



VisualIKORTest::LineSegArray VisualIKORTest::trajectoryAsLines(const array<base::Vector>& xs)
{
  LineSegArray lines;

  for(Int i=0; i<xs.size()-1; i++) {
    const Vector& xi(xs[i]);
    const Vector& xip1(xs[i+1]);
    lines.push_back(Segment3(Point3(xi[0],xi[1],xi[2]), Point3(xip1[0],xip1[1],xip1[2])));
  }

  return lines;
}




osg::Node* VisualIKORTest::createOSGVisual(Attributes visualAttributes) const
{
  if ((node!=0) && (attributes==visualAttributes))
    return &(*node);

  osg::Switch* worldNode = new osg::Switch();
  worldNode->addChild( osgCreateObstacles() );
  worldNode->addChild( osgCreateAxes() );
  worldNode->addChild( osgCreateTrajectory() );

  worldNode->addChild( osgCreateManipulator() );

  worldNode->setAllChildrenOn();
  worldNode->setValue(0,displayObstacles);
  worldNode->setValue(1,displayAxes);
  worldNode->setValue(2,displayEEPath);

  worldNode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

  node = worldNode;
  return &(*node);
}


/// \TODO fix setting font
osg::Node*  VisualIKORTest::osgCreateAxes() const
{
  const Real l = 1.0; // axis length
  const Real charSize = 0.2;
  osg::Vec4 labelCol(0,0,0,1);

  osg::Geode* geode = new osg::Geode();

  LineSegArray axes;
  axes.push_back(Vector3(l,0,0));
  axes.push_back(Vector3(0,l,0));
  axes.push_back(Vector3(0,0,l));
  osg::Geometry* linesGeom = newGeometryFromLines(axes, Color4(0.5,0.5,0.5));
  geode->addDrawable(linesGeom);

  // labels
  String font("fonts/times.ttf");

  osgText::Text* xtext = new osgText::Text;
  xtext->setFont(font);
  xtext->setCharacterSize(charSize);
//!!! fix  xtext->setFontSize(20,20);
  xtext->setColor(labelCol);
  xtext->setPosition(osg::Vec3(1.05,0,0));
  xtext->setAxisAlignment(osgText::Text::SCREEN);
  xtext->setText("X");
  geode->addDrawable(xtext);

  osgText::Text* ytext = new osgText::Text;
  ytext->setFont(font);
  ytext->setCharacterSize(charSize);
//!!! fix  ytext->setFontSize(20,20);
  ytext->setColor(labelCol);
  ytext->setPosition(osg::Vec3(0,1.05,0));
  ytext->setAxisAlignment(osgText::Text::SCREEN);
  ytext->setText("Y");
  geode->addDrawable(ytext);

  osgText::Text* ztext = new osgText::Text;
  ztext->setFont(font);
  ztext->setCharacterSize(charSize);
//!!! fix  ztext->setFontSize(20,20);
  ztext->setColor(labelCol);
  ztext->setPosition(osg::Vec3(0,0,1.05));
  ztext->setAxisAlignment(osgText::Text::SCREEN);
  ztext->setText("Z");
  geode->addDrawable(ztext);

  return geode;
}


osg::Node* VisualIKORTest::osgCreateObstacles() const
{
  osg::Geode* geode = new osg::Geode();
  geode->addDrawable( newGeometryFromLines(obstaclesAsLines(env),Color4(0.3,0.3,0.9)) );
  return geode;
}


osg::Node* VisualIKORTest::osgCreateManipulator() const
{
  ref<const BasicEnvironment> env(getEnvironment());
  ref<const SimulatedRobot> robot( narrow_ref<const SimulatedRobot>(getRobot()) );
  Int testManipulatorIndex = getManipulatorIndex();

  ref<const RobotDescription> robotDescr(robot->getRobotDescription());
  ref<const PlatformDescription> platfDescr(robotDescr->platform() );

  Matrix4 platformTransform( robot->getOrientation().getRotationMatrix3() );
  platformTransform.setTranslationComponent( robot->getPosition() );

  KinematicChain robotChain(robotDescr->getKinematicChain(3,platformTransform,testManipulatorIndex,0));
  KinematicChain platfChain(platfDescr->getKinematicChain(3,platformTransform));
  Int numPlatfLinks = platfChain.size();

  // make each test a seperate child in a Group
  osg::Group* testsGroup = new osg::Group();

  Int stepCount=0;
  for(Int t=0; t<tests.size(); t++) { // for each test
    const Test& test(tests[t]);

    KinematicChain chain(robotChain); // kinematic chain from world frame to manip/tool end-effector

    // locate tool
    bool toolAttached = false;
    if (test.toolAttached) {
      for(Int i=0; (i<env->numTools()) && !toolAttached; i++) {
        ref<const ToolDescription> td( env->getTool(i)->getToolDescription() );

        if (td->getName() == test.toolName) {
          toolAttached = true;
          KinematicChain toolChain( td->getKinematicChain() );
          chain += toolChain;
        }
      }
    }

    // create a Switch with each test joint space state of the manipulator represented as a child
    //  and platform (independently)
    osg::Switch* testPlatfSwitch = new osg::Switch();
    osg::Switch* testManipSwitch = new osg::Switch();

    Vector pf_prevq(platfChain.dof());
    Vector pf_q(platfChain.dof());

    for(Int i=0; i<test.qs.size(); i++) {
      const Vector& q( test.qs[i] );

      osg::Geode* platfGeode = new osg::Geode();
      osg::Geode* manipGeode = new osg::Geode();

      pf_prevq = pf_q;
      pf_q = vectorRange(q, Range(0, platfChain.dof()) );
      if(i==0) pf_prevq = pf_q;

      Real steerAngle = platfDescr->requiredSteeringAngle(pf_prevq, pf_q);

      platfGeode->addDrawable( newGeometryFromLines(platformAsLines(platfDescr,platfChain,pf_q,steerAngle ),
                            Color4(0.4,0.4,0.4)) );

      manipGeode->addDrawable( newGeometryFromLines(manipToolAsLines(chain,q,numPlatfLinks),Color4(0.3,0.3,0.3)) );

      stepCount++;

      testPlatfSwitch->addChild(platfGeode);
      testManipSwitch->addChild(manipGeode);
      if (test.displayRangeSpecified) {
        testPlatfSwitch->setValue(i, ((i >= test.displayStartIndex) && (i < test.displayEndIndex) && ((stepCount%displayStepMod) == 0) && displayPlatform ) );
        testManipSwitch->setValue(i, ((i >= test.displayStartIndex) && (i < test.displayEndIndex) && ((stepCount%displayStepMod) == 0)  ) );
      }
      else {
        testPlatfSwitch->setValue(i, ((stepCount%displayStepMod) == 0) && displayPlatform );
        testManipSwitch->setValue(i, (stepCount%displayStepMod) == 0);
      }
    }

    testPlatfSwitches.push_back( testPlatfSwitch );
    testManipSwitches.push_back( testManipSwitch );
    testsGroup->addChild( testPlatfSwitch );
    testsGroup->addChild( testManipSwitch );
  }

  return testsGroup;
}


osg::Node* VisualIKORTest::osgCreateTrajectory() const
{
  osg::Geode* geode = new osg::Geode();

  for(Int t=0; t<tests.size(); t++) { // for each test
    const Test& test(tests[t]);
    geode->addDrawable( newGeometryFromLines(trajectoryAsLines(test.xs),Color4(0.9,0.3,0.3)) );
  }
  return geode;
}


void VisualIKORTest::updateVisuals() const
{
  if ((node == 0)  || (testPlatfSwitches.size()==0)) return;

  node->setValue(0, displayObstacles);
  node->setValue(1, displayAxes);
  node->setValue(2, displayEEPath);

  Int stepCount=0;
  for(Int t=0; t<tests.size(); t++) { // for each test
    const Test& test(tests[t]);

    osg::Switch* testPlatfSwitch = &(*testPlatfSwitches[t]);
    osg::Switch* testManipSwitch = &(*testManipSwitches[t]);
    for(Int i=0; i<test.times.size(); i++) {
      testPlatfSwitch->setValue(i, ((i >= test.displayStartIndex) && (i < test.displayEndIndex) && ((stepCount%displayStepMod)==0) && displayPlatform ));
      testManipSwitch->setValue(i, ((i >= test.displayStartIndex) && (i < test.displayEndIndex) && ((stepCount%displayStepMod)==0) ));
      stepCount++;
    }

  }

}



void VisualIKORTest::svgLine(base::Externalizer& e, base::dom::DOMElement* svgElem, const gfx::Segment3& line) const
{
  Point3 start(line.s);
  Point3 end(line.e);
  start = svgTransform*start;
  end = svgTransform*end;

  DOMElement* lineElem = e.createElement("line");
  e.setElementAttribute(lineElem, "x1", base::realToString(start.x));
  e.setElementAttribute(lineElem, "y1", base::realToString(start.y));
  e.setElementAttribute(lineElem, "x2", base::realToString(end.x));
  e.setElementAttribute(lineElem, "y2", base::realToString(end.y));
  //e.setElementAttribute(lineElem, "stroke", "stroke-width:0.5");
  e.appendNode(svgElem, lineElem);

}


void VisualIKORTest::svgLines(base::Externalizer& e, base::dom::DOMElement* svgElem, const LineSegArray& lines) const
{
  for(Int l=0; l<lines.size(); l++)
    svgLine(e, svgElem, lines[l]);
}


void VisualIKORTest::svgText(base::Externalizer& e, base::dom::DOMElement* svgElem, const Point3& pos, const String& text, Real size) const
{
  DOMElement* textElem = e.createElement("text");
  e.setElementAttribute(textElem, "x", base::realToString(pos.x));
  e.setElementAttribute(textElem, "y", base::realToString(pos.y));
  e.setElementAttribute(textElem, "style", "font-size:"+base::realToString(size)
                                           +"; font-family:Arial, sans-serif");
  e.appendText(textElem, text);
  e.appendNode(svgElem, textElem);
}



void VisualIKORTest::svgOutputAxes(base::Externalizer& e, base::dom::DOMElement* svgElem) const
{
  const Real l = 1.0; // axis length

  LineSegArray axes;
  axes.push_back(Vector3(l,0,0));
  axes.push_back(Vector3(0,l,0));
  axes.push_back(Vector3(0,0,l));

  svgLines(e, svgElem, axes);

  svgText(e, svgElem, Point3(1,0,0), "X");
  svgText(e, svgElem, Point3(0,1,0), "Y");
  svgText(e, svgElem, Point3(0,0,1), "Z");
}




void VisualIKORTest::externalize(base::Externalizer& e, String format, Real version)
{
  if (format == "") format = "xml";

  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" v"+base::realToString(version)+" unsupported"));

  if (format == "xml")
    IKORTest::externalize(e,format,version);
  else if (format == "svg") {

    if (e.isInput()) {
      Unimplemented;
    }
    else { // output

        svgTransform.setIdentity();

        // must set type before first element is created (ignored if e is not a new output Externalizer)
        e.setDocumentType("svg","-//W3C//DTD SVG 20010904//EN",
                          "http://www.w3.org/TR/2001/REC-SVG-20010904/DTD/svg10.dtd");

        // createElement must be called first, as it will create to document root element if e is a new output Externalizer
        DOMElement* svgElem = e.createElement("svg");

        e.setElementAttribute(svgElem, "xmlns", "http://www.w3.org/2000/svg");
        e.setElementAttribute(svgElem, "width", "100%");
        e.setElementAttribute(svgElem, "height", "100%");


        svgOutputAxes(e, svgElem);


        e.appendElement(svgElem);
        e.appendBreak(svgElem);


    }

  }

}
