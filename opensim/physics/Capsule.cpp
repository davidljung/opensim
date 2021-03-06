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

  $Id: Capsule.cpp 1142 2004-09-28 21:03:00Z jungd $

****************************************************************************/

#include <physics/Capsule>

#include <base/Externalizer>
#include <gfx/VisualTriangles>
#include <physics/Material>
#include <physics/OBBCollisionModel>
#include <physics/GJKCollisionModel>

#include <osg/Group>
#include <osg/Geode>
#include <osg/LOD>
#include <osg/ShapeDrawable>

using physics::Capsule;

using base::Transform;
using base::dom::DOMNode;
using base::dom::DOMElement;

using gfx::Segment3;
using gfx::VisualTriangles;

using physics::MassProperties;
using physics::CollisionModel;
using physics::OBBCollisionModel;
using physics::GJKCollisionModel;


using osg::Node;
using osg::Group;
using osg::Geode;
using osg::LOD;
using osg::Vec3;
using osg::Vec2;


Capsule::Capsule()
  : _height(1.0), _radius(1.0), massPropertiesCached(false)
{
}

Capsule::Capsule(Real height, Real radius)
  : _height(height), _radius(radius), massPropertiesCached(false)
{
  if (_radius<0) _radius=consts::epsilon;
}

Capsule::Capsule(const Capsule& c)
  : _height(c._height), _radius(c._radius), massPropertiesCached(false)
{
}

Capsule::~Capsule()
{
}


/// \todo this is calculated for a cylinder, correct it for capsule
const MassProperties& Capsule::getMassProperties(ref<const Material> material) const
{
  if (massPropertiesCached && (density == material->density()))
    return massProperties;

  density = material->density();
  Real volume = consts::Pi*Math::sqr(_radius)*_height;
  massProperties.mass = volume*density;

  const Real m = massProperties.mass;
  Matrix3 Ibody;
  Ibody.e(1,1) = m*Math::sqr(_radius)/4.0 + m*Math::sqr(_height)/12.0;
  Ibody.e(2,2) = Ibody.e(1,1);
  Ibody.e(3,3) = m*Math::sqr(_radius)/2.0;

  massProperties.setIbody(Ibody);
  massProperties.centerOfMass = Point3(0.0,0.0,0.0);

  massPropertiesCached = true;
  return massProperties;
}



Segment3 Capsule::shortestSegmentBetween(const base::Transform& t, const Point3& p) const
{
  Unimplemented;
}


Segment3 Capsule::shortestSegmentBetween(const base::Transform& t, const gfx::Segment3& s) const
{
  // find the point on the axis segment closest to s
  Point3 axisstart(0,0,-height()/2.0); axisstart = t.transform(axisstart);
  Point3 axisend(0,0,height()/2.0); axisend = t.transform(axisend);
  Segment3 axisseg( axisstart, axisend);
  Segment3 toaxis = s.shortestSegmentBetween(axisseg);

  // now, shortest segment is identical to that from a sphere centered at
  //  toaxis.e and s
  ref<Sphere> sphere( NewObj Sphere(radius()) );
  Transform center(toaxis.e);
  return sphere->shortestSegmentBetween(center, s);
}


Segment3 Capsule::shortestSegmentBetween(const base::Transform& t, const gfx::Triangle3& tri) const
{
  Unimplemented;
}


Segment3 Capsule::shortestSegmentBetween(const base::Transform& t, const gfx::Quad3& q) const
{
  Unimplemented;
}


Segment3 Capsule::shortestSegmentBetween(const base::Transform& t1, ref<const Shape> s, const base::Transform& t2) const
{
  Unimplemented;
}



/// \todo use Geometry & add normals
osg::Node* Capsule::createOSGCapsule(Visual::Attributes visualAttributes,
                                     Int slices, Int stacks) const
{

  bool onlyVerts = ((visualAttributes & Visual::VerticesOnly) != 0);

  // oriented along the z-axis
  osg::ref_ptr<osg::ShapeDrawable> shapeDrawable = new osg::ShapeDrawable(&(*shape));
  osg::ref_ptr<osg::TessellationHints> tessHints = new osg::TessellationHints();
  tessHints->setTargetNumFaces(slices*stacks);
  tessHints->setTessellationMode(osg::TessellationHints::USE_TARGET_NUM_FACES);
  tessHints->setCreateNormals(!onlyVerts);
  tessHints->setCreateTextureCoords(!onlyVerts);
  shapeDrawable->setTessellationHints(&(*tessHints));

  osg::Geode* geode = new osg::Geode();
  geode->addDrawable(&(*shapeDrawable));

  return geode;
}



osg::Node* Capsule::createOSGVisual(Visual::Attributes visualAttributes) const
{
  if (isOSGVisualCached(visualAttributes)) return &(*node);

  shape = new osg::Capsule(osg::Vec3(0.0f,0.0f,0.0f),radius(),height());

  Real r = Math::maximum(0.5,radius());
  Real h = Math::maximum(1.0,height());

//  osg::Node* node0 = createOSGCapsule(visualAttributes, Int(52*r), (h<40)?Int(12*h):2000);
//  osg::Node* node1 = createOSGCapsule(visualAttributes, Int(40*r), (h<40)?Int(8*h):1300);
//  osg::Node* node2 = createOSGCapsule(visualAttributes, Int(20*r), (h<40)?Int(4*h):700);
//  osg::Node* node3 = createOSGCapsule(visualAttributes, 8, 1);

//replace above after normals added (i.e. switched from mesh to geometry)
  osg::Node* node0 = createOSGCapsule(visualAttributes, Int(104*r), (h<40)?Int(12*h):2000);
  osg::Node* node1 = createOSGCapsule(visualAttributes, Int(80*r), (h<40)?Int(8*h):1300);
  osg::Node* node2 = createOSGCapsule(visualAttributes, Int(40*r), (h<40)?Int(4*h):700);
  osg::Node* node3 = createOSGCapsule(visualAttributes, 16, 1);


  osg::LOD* lod = new osg::LOD();
  lod->setName("Capsule");
  lod->addChild(node0);
  lod->addChild(node1);
  lod->addChild(node2);
  lod->addChild(node3);

  lod->setRange(0,0,2.0);
  lod->setRange(1,2.0,16.0);
  lod->setRange(2,16.0,120.0*Math::maximum(r,h));
  lod->setRange(3,120.0*Math::maximum(r,h),consts::Infinity);

  if (!(visualAttributes & Visual::ShowAxes))
    node = lod;
  else {
    Group* group = new Group();
    group->addChild( lod );
    group->addChild( createOSGAxes(base::Dimension3(2.0*radius(),2.0*radius(),height())) );
    node = group;
  }

  attributes = visualAttributes;
  return &(*node);
}



base::ref<CollisionModel> Capsule::getCollisionModel(CollisionModel::CollisionModelType modelType) const
{
  if ((collisionModel!=0) &&
      ((this->modelType==modelType) || (modelType==CollisionModel::AnyModel)))
    return collisionModel;

  collisionModel = Shape::getCollisionModel(modelType);
  this->modelType=modelType;

  return collisionModel;
}


void Capsule::serialize(base::Serializer& s)
{
  s(_height)(_radius);

  if (s.isInput()) {
    massPropertiesCached = false;
    collisionModel = ref<CollisionModel>(0);
    clearVisualCache();
  }
}



bool Capsule::formatSupported(String format, Real version, ExternalizationType type) const
{
  return ( (format=="xml") && (version==1.0) );
}


void Capsule::externalize(base::Externalizer& e, String format, Real version)
{
  if (format == "") format = "xml";

  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" v"+base::realToString(version)+" unsupported"));

  if (e.isOutput()) {

    DOMElement*  capsuleElem = e.createElement("capsule");
    e.setElementAttribute(capsuleElem,"height",base::realToString(_height));
    e.setElementAttribute(capsuleElem,"radius",base::realToString(_radius));

    e.appendElement(capsuleElem);
  }
  else { // input

    massPropertiesCached = false;
    node = 0;
    collisionModel = ref<CollisionModel>(0);

    DOMNode* context = e.context();

    DOMElement* capsuleElem = e.getFirstElement(context, "capsule");

    _height = e.toReal( e.getDefaultedElementAttribute(capsuleElem, "height", "1") );
    _radius = e.toReal( e.getDefaultedElementAttribute(capsuleElem, "radius", "1") );

    e.removeElement(capsuleElem);

  }
}


