/****************************************************************************
  Copyright (C)2004 David Jung <opensim@pobox.com>

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

  $Id: Box.cpp 1031 2004-02-11 20:46:36Z jungd $

****************************************************************************/

#include <physics/CompositeShape>

#include <base/Externalizer>
#include <gfx/VisualTriangles>
#include <physics/Material>
#include <physics/Cylinder>
#include <physics/Cone>

#include <osg/Group>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>


using physics::CompositeShape;

using base::Transform;
using base::Point3;
using base::reflist;
using base::dom::DOMNode;
using base::dom::DOMElement;
using gfx::Color3;
using gfx::VisualTriangles;
using gfx::Segment3;
using gfx::Triangle3;
using gfx::Quad3;
using physics::BoundingBox;
using physics::BoundingSphere;
using physics::MassProperties;
using physics::CollisionModel;

using osg::Node;
using osg::Group;
using osg::Geode;


CompositeShape::CompositeShape(const base::reflist<Shape>& shapes, const std::list<base::Transform>& transforms)
 : massPropertiesCached(false)
{
  Assertm(shapes.size()==transforms.size(), "should be one transform per shape");

  array<ref<Shape> > ashapes(shapes.size());

  reflist<Shape>::const_iterator s = shapes.begin();
  reflist<Shape>::const_iterator send = shapes.end();
  Int i=0;
  while (s != send) {
    if ((*s)->visualTypeSupported(OSGVisual) == false)
      throw new std::invalid_argument(Exception("only OSGVisual shapes can be used to compose an OSGVisual composite shape"));
    ashapes[i++] = *s;
    ++s;
  }

  array<Transform> atransforms(transforms.size());
  std::list<Transform>::const_iterator t = transforms.begin();
  std::list<Transform>::const_iterator tend = transforms.end();
  i=0;
  while (t != tend) {
    atransforms[i++] = *t;
    ++t;
  }

  this->shapes = ashapes;
  this->transforms = atransforms;
}


CompositeShape::CompositeShape(const CompositeShape& c)
  : shapes(c.shapes), transforms(c.transforms), massPropertiesCached(false)
{
}

CompositeShape::~CompositeShape()
{
}


BoundingBox CompositeShape::getBoundingBox() const
{
  Unimplemented;
}


BoundingSphere CompositeShape::getBoundingSphere() const
{
  Unimplemented;
}


const MassProperties& CompositeShape::getMassProperties(ref<const Material> material) const
{
  if (massPropertiesCached && (density == material->density()))
    return massProperties;

  density = material->density();
Unimplemented;
//...
//  massProperties.setIbody(Ibody);

  massProperties.centerOfMass = Point3(0.0,0.0,0.0);

  massPropertiesCached = true;

  return massProperties;
}



osg::Node* CompositeShape::createOSGVisual(Visual::Attributes visualAttributes) const
{
  if (isOSGVisualCached(visualAttributes)) return &(*node);

  osg::Group* group = new osg::Group;
  for(Int i=0; i<shapes.size();i++) {
    osg::MatrixTransform* transform = NewObj osg::MatrixTransform();
    transform->setMatrix( transforms[i].getTransform() );
    group->addChild(transform);
    transform->addChild( shapes[i]->createOSGVisual(visualAttributes) );
  }

  group->setName("CompositeShape");

  node = group;
  attributes = visualAttributes;
  return &(*node);
}





Segment3 CompositeShape::shortestSegmentBetween(const Transform& t, const Point3& p) const
{
  Unimplemented;
}


Segment3 CompositeShape::shortestSegmentBetween(const Transform& t, const Segment3& s) const
{
  Unimplemented;
}


Segment3 CompositeShape::shortestSegmentBetween(const Transform& t, const Triangle3& tri) const
{
  Unimplemented;
}


Segment3 CompositeShape::shortestSegmentBetween(const Transform& t, const Quad3& q) const
{
  Unimplemented;
}


Segment3 CompositeShape::shortestSegmentBetween(const Transform& t1, ref<const Shape> s, const Transform& t2) const
{
  Unimplemented;
}




/// \TODO implement
base::ref<CollisionModel> CompositeShape::getCollisionModel(CollisionModel::CollisionModelType modelType) const
{
  if ((collisionModel!=0) &&
      ((this->modelType==modelType) || (modelType==CollisionModel::AnyModel)))
    return collisionModel;

  collisionModel = Shape::getCollisionModel(modelType); //!!!???
  this->modelType=modelType;

  return collisionModel;
}


/// \TODO implement
void CompositeShape::serialize(base::Serializer& s)
{
  /*
  s(dim.x,"x")(dim.y,"y")(dim.z,"z");

  if (s.isInput()) {
    massPropertiesCached = false;
    node = 0;
    collisionModel = ref<CollisionModel>(0);
  }
  */
}



bool CompositeShape::formatSupported(String format, Real version, ExternalizationType type) const
{
  return ( (format=="xml") && (version==1.0) );
}


/// \TODO implement
void CompositeShape::externalize(base::Externalizer& e, String format, Real version)
{
  if (format == "") format = "xml";

  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" v"+base::realToString(version)+" unsupported"));



  if (e.isOutput()) {
    DOMElement* compositeElem = e.createElement("composite");

    for(Int i=0; i<shapes.size(); i++) {
      //...
Unimplemented;
    }

    e.appendElement(compositeElem);
  }
  else {

    massPropertiesCached = false;
    collisionModel = ref<CollisionModel>(0);
    clearVisualCache();

    DOMNode* context = e.context();

    DOMElement* boxElem = e.getFirstElement(context, "composite");
Unimplemented;
//...

    e.removeElement(boxElem);

  }

/*
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
  */

}



