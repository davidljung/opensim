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

  $Id: VisualDebugUtil.cpp 1138 2004-09-28 20:59:25Z jungd $
  $Revision: 1.2 $
  $Date: 2004-09-28 16:59:25 -0400 (Tue, 28 Sep 2004) $
  $Author: jungd $

****************************************************************************/

#include <physics/VisualDebugUtil>

#include <physics/Material>
#include <physics/Cone>
#include <physics/CompositeShape>

#include <osg/Group>
#include <osg/Geode>
#include <osg/ShapeDrawable>

using physics::VisualDebugUtil;

using base::Transform;
using base::reflist;
using gfx::Color4;
using gfx::Segment3;
using gfx::Visual;
using physics::Box;
using physics::Sphere;
using physics::Cylinder;
using physics::Capsule;
using physics::Cone;
using physics::CompositeShape;
using physics::Material;

using osg::Node;
using osg::Group;
using osg::Geode;
using osg::MatrixTransform;



VisualDebugUtil::ObjectMap VisualDebugUtil::debugObjects;

Visual::Attributes VisualDebugUtil::attributes;
osg::ref_ptr<osg::Group> VisualDebugUtil::node;


VisualDebugUtil::DebugObjectData::DebugObjectData(const String& name, ref<const Shape> shape, const gfx::Color4& color, const base::Transform& configuration)
      : name(name), shape(shape), color(color), configuration(configuration)
{
  transform = new MatrixTransform();
  transform->setMatrix(configuration.getTransform());
}


void VisualDebugUtil::addDebugObject(ref<const Shape> shape, const String& name, Transform worldConfiguration, const gfx::Color4& color)
{
  debugObjects[name] = DebugObjectData(name, shape, color, worldConfiguration);
}


void VisualDebugUtil::addDebugBoxObject(base::Dimension3 dimensions, const String& name, Transform worldConfiguration, const gfx::Color4& color)
{
  ref<const Box> box(NewObj Box(dimensions.x,dimensions.y,dimensions.z));
  addDebugObject(box, name, worldConfiguration, color);
}


void VisualDebugUtil::addDebugSphereObject(Real radius, const String& name, Transform worldConfiguration, const gfx::Color4& color)
{
  ref<const Sphere> sphere(NewObj Sphere(radius));
  addDebugObject(sphere, name, worldConfiguration, color);
}


void VisualDebugUtil::addDebugCylinderObject(Real height, Real radius, const String& name, Transform worldConfiguration, const gfx::Color4& color)
{
  ref<const Cylinder> cylinder(NewObj Cylinder(height, radius));
  addDebugObject(cylinder, name, worldConfiguration, color);
}

void VisualDebugUtil::addDebugCapsuleObject(Real height, Real radius, const String& name, Transform worldConfiguration, const gfx::Color4& color)
{
  ref<const Capsule> capsule(NewObj Capsule(height, radius));
  addDebugObject(capsule, name, worldConfiguration, color);
}


void VisualDebugUtil::addDebugVectorArrow(Vector3 v, const String& name, Transform worldConfiguration, const gfx::Color4& color)
{
  // create a arrow from a cylinder and a cone
  Real radius = v.length()/100.0;
  Real headRadius = radius*2;
  Real headHeight = radius*4;
  ref<Cylinder> body(NewObj Cylinder(v.length() - headHeight, radius));
  ref<Cone> head(NewObj Cone(headHeight, headRadius));

  Transform bodyTransform(Vector3(0,0,-headHeight/2.0));
  Transform headTransform(Vector3(0,0,v.length()/2.0 - 3.0*headHeight/4.0)); // translate head to end of body

  reflist<Shape> shapes;
  shapes.push_back(body);
  shapes.push_back(head);


  // compute a transform, c, that will rotate the z-axis aligned arrow to the direction represented by
  //  the vector v
  Segment3 seg(Point3(0,0,0), v);
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

  std::list<Transform> transforms;
  transforms.push_back(c*bodyTransform);
  transforms.push_back(c*headTransform);

  ref<const CompositeShape> arrow(NewObj CompositeShape(shapes, transforms));
  addDebugObject(arrow, name, worldConfiguration, color);
}




void VisualDebugUtil::setConfiguration(const String& name, const base::Transform& configuration)
{
  ObjectMap::iterator obj = debugObjects.find(name);
  if (obj != debugObjects.end()) {
    DebugObjectData& objdata( (*obj).second );
    objdata.configuration = configuration;
    objdata.transform->setMatrix(configuration.getTransform()); // update Visual
  }
}


void VisualDebugUtil::setColor(const String& name, const gfx::Color4& color)
{
  ObjectMap::iterator obj = debugObjects.find(name);
  if (obj != debugObjects.end()) {
    DebugObjectData& objdata( (*obj).second );
    objdata.color = color;


    // recreate the shape node with new material color
    if (objdata.transform->getNumChildren() > 0)
      objdata.transform->removeChild(objdata.transform->getChild(0));

    ref<Material> material(NewObj Material("plastic", objdata.color));
    osg::Node& shapeNode = *objdata.shape->createOSGVisual(attributes);
    if (!objdata.shape->includesAppearance()) {
      shapeNode.setStateSet( &(*material->createState()) );
    }
    objdata.transform->addChild(&shapeNode); // hang shape on it's transform
  }
}


void VisualDebugUtil::setColorAll(gfx::Color4& color)
{
  ObjectMap::iterator obj = debugObjects.begin();
  ObjectMap::iterator end = debugObjects.end();
  while (obj != end) {
    DebugObjectData& objdata( (*obj).second );
    setColor(objdata.name, color);
    ++obj;
  }
}


osg::Node* VisualDebugUtil::createOSGVisual(Attributes visualAttributes) const
{
  if ((node!=0) && (attributes==visualAttributes))
    return &(*node);

  node = new osg::Group();
  node->setName("VisualDebugUtil");

  attributes = visualAttributes;

  updateVisual();

  return &(*node);
}


void VisualDebugUtil::updateVisual()
{
  if (node==0) return; // no Visual to update

  // remove all Shapes from the top-level group and re-add them (to account for
  //  additions and removals)
  while (node->getNumChildren() > 0)
    node->removeChild(node->getChild(0));

  ObjectMap::iterator obj = debugObjects.begin();
  ObjectMap::iterator end = debugObjects.end();
  while (obj != end) {
    DebugObjectData& objdata( (*obj).second );

    // create Shape Visual if it hasn't already been done
    if ( objdata.transform->getNumChildren() == 0) {
      ref<Material> material(NewObj Material("plastic", objdata.color));
      osg::Node& shapeNode = *objdata.shape->createOSGVisual(attributes);
      if (!objdata.shape->includesAppearance()) {
        shapeNode.setStateSet( &(*material->createState()) );
      }
      objdata.transform->addChild(&shapeNode); // hang shape on it's transform
    }

    objdata.transform->setMatrix(objdata.configuration.getTransform()); // update configuration

    node->addChild(&(*objdata.transform)); // add to top level group

    ++obj;
  }

}
