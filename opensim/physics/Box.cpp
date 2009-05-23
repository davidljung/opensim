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

  $Id: Box.cpp 1140 2004-09-28 21:02:12Z jungd $

****************************************************************************/

#include <physics/Box>

#include <base/Externalizer>
#include <gfx/VisualTriangles>
#include <physics/Material>
#include <physics/Sphere>

#include <osg/Group>
#include <osg/Geode>
#include <osg/ShapeDrawable>


using physics::Box;

using base::Transform;
using base::Point3;
using base::dom::DOMNode;
using base::dom::DOMElement;
using gfx::Color3;
using gfx::VisualTriangles;
using gfx::Segment3;
using gfx::Triangle3;
using gfx::Quad3;
using physics::MassProperties;
using physics::CollisionModel;

using osg::Node;
using osg::Group;
using osg::Geode;


Box::Box(Real width, Real height, Real depth)
  : dim(width,height,depth), massPropertiesCached(false)
{}

Box::Box(const Box& b)
  : dim(b.dim), massPropertiesCached(false)
{}

Box::~Box()
{
}


const MassProperties& Box::getMassProperties(ref<const Material> material) const
{
  if (massPropertiesCached && (density == material->density()))
    return massProperties;

  density = material->density();

  Real dx = dim.x;
  Real dy = dim.y;
  Real dz = dim.z;

  Real volume = dx*dy*dz;
  massProperties.mass = volume*density;

  Real k = massProperties.mass/12.0;
  Matrix3 Ibody;
  Ibody.e(1,1) = (dy*dy + dz*dz)*k;
  Ibody.e(2,2) = (dx*dx + dz*dz)*k;
  Ibody.e(3,3) = (dx*dx + dy*dy)*k;
  massProperties.setIbody(Ibody);

  massProperties.centerOfMass = Point3(0.0,0.0,0.0);

  massPropertiesCached = true;

  return massProperties;
}



osg::Node* Box::createOSGVisual(Visual::Attributes visualAttributes) const
{
  if (isOSGVisualCached(visualAttributes)) return &(*node);

  shape = new osg::Box(osg::Vec3(0.0f,0.0f,0.0f),dim.x,dim.y,dim.z);

  osg::Geode* geode = new Geode();
  geode->setName("Box");
  //  geode->addDrawable( cube );
  geode->addDrawable(new osg::ShapeDrawable(&(*shape)));

  if (!(visualAttributes & Visual::ShowAxes))
    node = geode;
  else {
    if (dim.x > 0.16)  //!!!
      node=geode; //!!!
    else {//!!!
    // create axes
    Group* group = new Group();
    group->addChild( geode );
    group->addChild( createOSGAxes(dimensions()) );
    node = group;
    }//!!!
  }
  attributes = visualAttributes;
  return &(*node);
}



array<Quad3> Box::asQuads() const
{
  array<Quad3> quads;
  quads.push_back(Quad3(Point3(-1, 1,-1),
                        Point3( 1, 1,-1),
                        Point3( 1,-1,-1),
                        Point3(-1,-1,-1)));

  quads.push_back(Quad3(Point3( 1, 1,-1),
                        Point3( 1, 1, 1),
                        Point3( 1,-1, 1),
                        Point3( 1,-1,-1)));

  quads.push_back(Quad3(Point3( 1, 1, 1),
                        Point3(-1, 1, 1),
                        Point3(-1,-1, 1),
                        Point3( 1,-1, 1)));

  quads.push_back(Quad3(Point3(-1, 1, 1),
                        Point3(-1, 1,-1),
                        Point3(-1,-1,-1),
                        Point3(-1,-1, 1)));

  quads.push_back(Quad3(Point3(-1, 1, 1),
                        Point3( 1, 1, 1),
                        Point3( 1, 1,-1),
                        Point3(-1, 1,-1)));

  quads.push_back(Quad3(Point3(-1,-1, 1),
                        Point3(-1,-1,-1),
                        Point3( 1,-1,-1),
                        Point3( 1,-1, 1)));

  // scale to dimensions
  for(Int i=0; i<6; i++) {
    Quad3& q(quads[i]);
    for(Int c=0; c<4; c++) {
      Point3& cc(q[c]);
      cc.x *= dim.x/2.0;
      cc.y *= dim.y/2.0;
      cc.z *= dim.z/2.0;
    }
  }

  return quads;
}


Segment3 Box::shortestSegmentBetween(const Transform& t, const Point3& p) const
{
  array<Quad3> quads(asQuads());
  Real dist2 = consts::maxReal;
  Point3 closest;
  for(Int i=0; i<quads.size(); i++) {
    quads[i].transform(t);
    Point3 cp(quads[i].pointClosestTo(p));
    Real d2 = (p-cp).norm();
    if (d2 < dist2) {
      dist2 = d2;
      closest = cp;
    }
  }
  return Segment3(closest,p);
}


Segment3 Box::shortestSegmentBetween(const Transform& t, const Segment3& s) const
{
  array<Quad3> quads(asQuads());
  Real dist2 = consts::maxReal;
  Segment3 shortest;
  for(Int i=0; i<quads.size(); i++) {
    quads[i].transform(t);
    Segment3 ss(quads[i].shortestSegmentBetween(s));
    Real d2 = ss.norm();
    if (d2 < dist2) {
      dist2 = d2;
      shortest = ss;
    }
  }
  return shortest;
}


Segment3 Box::shortestSegmentBetween(const Transform& t, const Triangle3& tri) const
{
  array<Quad3> quads(asQuads());
  Real dist2 = consts::maxReal;
  Segment3 shortest;
  for(Int i=0; i<quads.size(); i++) {
    quads[i].transform(t);
    Segment3 ss(quads[i].shortestSegmentBetween(tri));
    Real d2 = ss.norm();
    if (d2 < dist2) {
      dist2 = d2;
      shortest = ss;
    }
  }
  return shortest;
}


Segment3 Box::shortestSegmentBetween(const Transform& t, const Quad3& q) const
{
  array<Quad3> quads(asQuads());
  Real dist2 = consts::maxReal;
  Segment3 shortest;
  for(Int i=0; i<quads.size(); i++) {
    quads[i].transform(t);
    Segment3 ss(quads[i].shortestSegmentBetween(q));
    Real d2 = ss.norm();
    if (d2 < dist2) {
      dist2 = d2;
      shortest = ss;
    }
  }
  return shortest;
}


Segment3 Box::shortestSegmentBetween(const Transform& t1, ref<const Shape> s, const Transform& t2) const
{
  Segment3 shortest;

  if (instanceof(*s, const Box)) {
    ref<const Box> b(narrow_ref<const Box>(s));

    array<Quad3> quads1(asQuads());
    array<Quad3> quads2(b->asQuads());
    for(Int i=0; i<quads1.size(); i++) quads1[i].transform(t1);
    for(Int i=0; i<quads2.size(); i++) quads2[i].transform(t2);

    Real dist2 = consts::maxReal;
    for(Int i1=0; i1<quads1.size(); i1++) {
      for(Int i2=0; i2<quads2.size(); i2++) {
        Segment3 ss(quads1[i1].shortestSegmentBetween(quads2[i2]));
        Real d2 = ss.norm();
        if (d2 < dist2) {
          dist2 = d2;
          shortest = ss;
        }
      }
    }

  }
  else if (instanceof(*s, const Sphere)) {

    ref<const Sphere> sphere(narrow_ref<const Sphere>(s));
    // just find the shortest segment between the box and the center, then
    //  shorten it to the sphere surface
    // (doesn't handle case where they intersect properly)
    Point3 sphereCenter( t2.getTranslation() );
    shortest = Segment3( shortestSegmentBetween( t1, sphereCenter ) );

    Real r(sphere->radius());
    // shorten it by r on the e end
    Vector3 dir(shortest.e-shortest.s);
    shortest.e = shortest.s + dir*(1-(r/dir.length()));

  }
  else { // other shapes not yet implemented;
    Unimplemented;
  }

  return shortest;
}



gfx::Point3 Box::support(const gfx::Vector3& v) const
{
  return gfx::Point3( (v.x<0)?-(dim.x/2.0):(dim.x/2.0),
                      (v.y<0)?-(dim.y/2.0):(dim.y/2.0),
                      (v.z<0)?-(dim.z/2.0):(dim.z/2.0) );
}


base::ref<CollisionModel> Box::getCollisionModel(CollisionModel::CollisionModelType modelType) const
{
  if ((collisionModel!=0) &&
      ((this->modelType==modelType) || (modelType==CollisionModel::AnyModel)))
    return collisionModel;

  collisionModel = Shape::getCollisionModel(modelType);
  this->modelType=modelType;

  return collisionModel;
}


void Box::serialize(base::Serializer& s)
{
  s(dim.x,"x")(dim.y,"y")(dim.z,"z");

  if (s.isInput()) {
    massPropertiesCached = false;
    node = 0;
    collisionModel = ref<CollisionModel>(0);
  }
}



bool Box::formatSupported(String format, Real version, ExternalizationType type) const
{
  return ( (format=="xml") && (version==1.0) );
}


void Box::externalize(base::Externalizer& e, String format, Real version)
{
  if (format == "") format = "xml";

  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" v"+base::realToString(version)+" unsupported"));

  if (e.isOutput()) {

    DOMElement*  boxElem = e.createElement("box");
    e.setElementAttribute(boxElem,"width",base::realToString(dim.x));
    e.setElementAttribute(boxElem,"height",base::realToString(dim.y));
    e.setElementAttribute(boxElem,"depth",base::realToString(dim.z));

    e.appendElement(boxElem);
  }
  else {

    massPropertiesCached = false;
    node = 0;
    collisionModel = ref<CollisionModel>(0);

    DOMNode* context = e.context();

    DOMElement* boxElem = e.getFirstElement(context, "box");

    dim.x = e.toReal( e.getDefaultedElementAttribute(boxElem, "width", "1") );
    dim.y = e.toReal( e.getDefaultedElementAttribute(boxElem, "height", "1") );
    dim.z = e.toReal( e.getDefaultedElementAttribute(boxElem, "depth", "1") );

    e.removeElement(boxElem);

  }
}



