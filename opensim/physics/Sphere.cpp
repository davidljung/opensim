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

  $Id: Sphere.cpp 1141 2004-09-28 21:02:35Z jungd $

****************************************************************************/

#include <physics/Sphere>

#include <base/Externalizer>
#include <gfx/VisualTriangles>
#include <physics/Material>
#include <physics/OBBCollisionModel>
#include <physics/GJKCollisionModel>

#include <osg/Group>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/LOD>

using base::dom::DOMNode;
using base::dom::DOMElement;
using gfx::Point2;
using gfx::Point3;
using gfx::Vector3;
using gfx::Segment3;
using gfx::VisualTriangles;
using physics::Sphere;
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


Sphere::Sphere()
  : _radius(1.0), massPropertiesCached(false)
{
}

Sphere::Sphere(Real radius)
  : _radius(radius), massPropertiesCached(false)
{
  shape = new osg::Sphere(osg::Vec3(0,0,0),radius);
}

Sphere::Sphere(const Sphere& s)
  : _radius(s._radius), massPropertiesCached(false)
{
  shape = new osg::Sphere(osg::Vec3(0,0,0),radius());
}

Sphere::~Sphere()
{
}


const MassProperties& Sphere::getMassProperties(ref<const Material> material) const
{
  if (massPropertiesCached && (density == material->density()))
    return massProperties;

  density = material->density();
  Real volume = 4.0*(consts::Pi*Math::cube(radius()))/3.0;
  massProperties.mass = volume*density;

  Matrix3 Ibody;
  Ibody(1,1) = Ibody(2,2) = Ibody(3,3) = (2.0*massProperties.mass*Math::sqr(radius()))/5.0;
  massProperties.setIbody(Ibody);

  massProperties.centerOfMass = Point3(0.0,0.0,0.0);

  massPropertiesCached = true;
  return massProperties;
}


Segment3 Sphere::shortestSegmentBetween(const base::Transform& t, const Point3& p) const
{
  Point3 c(t.getTranslation());
  Vector3 diff( p - c );
  Real diffLen = diff.length();
  Segment3 seg;
  seg.e = p;

  if (diffLen > consts::epsilon)
    seg.s = c + radius()*diff.normalize();
  else
    seg.s = c + radius()*Vector3(1,0,0);

  return seg;
}



Segment3 Sphere::shortestSegmentBetween(const base::Transform& t, const gfx::Segment3& s) const
{
  // algorithm from XEngine by Martin Ecker ( http://xengine.sourceforge.net )
  Point3 c(t.getTranslation());
  Vector3 segDir( s.e - s.s );
  Real distance = 0;
  Point3 p1,p2;
  Real DlengthSquared = s.norm();
  Real u = 0;
  if (DlengthSquared >= consts::epsilon)		// avoid division by 0
    u = dot(segDir, c - s.s) / DlengthSquared;

  Vector3 R;
  if (u <= 0)
    R = s.s;
  else if (u >= 1)
    R = s.e;
  else
    R = s.s + u * segDir;

  Vector3 E = R - c;
  Real Elength = E.length();

  if (Elength > radius())		// segment does not intersect the sphere
  {
    p1 = R;
    p2 = c + (radius() / Elength) * E;		// note that E cannot have zero length
    distance = Elength - radius();
  }
  else												// segment intersects the sphere or is completely contained
  {
    Vector3 centerToStart = s.s  - c;
    Vector3 centerToEnd = s.e - c;
    Real centerToStartLength = centerToStart.length();
    Real centerToEndLength = centerToEnd.length();

    if (centerToStartLength < radius() && centerToEndLength < radius()) {
      // the segment is completely contained within the sphere
      if (centerToStartLength < radius()) {		// then the end point must be the closest to the sphere's shell
        p1 = s.e;
        p2 = c + radius() * centerToEnd.normalize();
        distance = radius() - centerToEndLength;
      }
      else {					// the start point is closest to the sphere's shell
        p1 = s.s;
        p2 = c + radius() * centerToStart.normalize();
        distance = radius() - centerToStartLength;
      }
    }
    else {	// the segment intersects the sphere
      Vector3 projCenter = s.s - u * segDir;

      Vector3 intersection = projCenter;
      if (DlengthSquared >= consts::epsilon)		// to avoid division by 0
      {
        // take abs of radius^2 - (projCenter-c).norm() in case it's negative (DJ)
        Real s = Math::sqrt(Math::abs( Math::sqr(radius()) - (projCenter - c).norm() ));
        // if the end point of the segment is inside the sphere, the direction vector points from outside of
        // the sphere to the inside; therefore we have to swap the direction vector which we do by inverting s
        if (centerToEndLength < radius())
          s = -s;
        intersection += (s / Math::sqrt(DlengthSquared)) * segDir;
      }

      p1 = intersection;
      p2 = intersection;
      distance = 0;
    }
  }

  return Segment3(p2,p1);
}


Segment3 Sphere::shortestSegmentBetween(const base::Transform& t, const gfx::Triangle3& tri) const
{
  Unimplemented;
}


Segment3 Sphere::shortestSegmentBetween(const base::Transform& t, const gfx::Quad3& q) const
{
  Unimplemented;
}


Segment3 Sphere::shortestSegmentBetween(const base::Transform& t1, ref<const Shape> s, const base::Transform& t2) const
{
  if (instanceof(*s, const Sphere)) {

    ref<const Sphere> sphere1(this);
    ref<const Sphere> sphere2(narrow_ref<const Sphere>(s));
    Point3 c1(t1.getTranslation());
    Point3 c2(t2.getTranslation());
    Real r1( sphere1->radius() );
    Real r2( sphere2->radius() );

    Point3 p1,p2;
    Vector3 D = c2 - c1;
    Real Dlength = D.length();

    // normalize D
    if (Dlength < consts::epsilon)
      D.setZero();
    else
      D /= Dlength;

    if (Dlength > r1 + r2) {		// spheres do not intersect
      p1 = c1 + r1 * D;
      p2 = c2 - r2 * D;
    }
    else if (Dlength < Math::abs(r1 - r2))		// one sphere is contained in the other
    {
      p1 = c1 + r1 * D;
      p2 = c2 + r2 * D;
    }
    else if (Dlength < consts::epsilon)		// |D| == 0, the spheres are equal
    {
      Vector3 intersection = c1;
      intersection.x += r1;

      p1 = intersection;
      p2 = intersection;
    }
    else {		// the spheres intersect
      Real t = 0.5 * (1 + (Math::sqr(r1) - Math::sqr(r2)) / Math::sqr(Dlength));
      Real r = Math::sqrt(Math::sqr(r1) - Math::sqr(t * Dlength));
      Vector3 C = c1 + t * D;

      Matrix3 basis;
      basis.setOrthonormalBasisOf(D);// get the coordinate system of the circle, D is the plane normal and therefore the z vector of the basis
      Vector3 intersection = C + r * basis.row(1);

      p1 = intersection;
      p2 = intersection;
    }

    return Segment3(p1,p2);

  }
  else if (instanceof(*s, const Box)) {

    return s->shortestSegmentBetween(t2,ref<const Shape>(this), t1).swapEnds();

  }
  else {
    Unimplemented;
  }
}






gfx::Point3 Sphere::support(const base::Vector3& v) const
{
  Real s = v.length();
  if (s > consts::epsilon)
    return v*(_radius/s);
  else
    return base::Point3(0,0,0);
}


osg::Node* Sphere::createOSGSphere(Visual::Attributes visualAttributes,
                                   Int slices) const
{
  osg::Geode* geode = new osg::Geode();
  geode->addDrawable(new osg::ShapeDrawable(&(*shape)));

  return geode;
}


osg::Node* Sphere::createOSGVisual(Visual::Attributes visualAttributes) const
{
  if (isOSGVisualCached(visualAttributes)) return &(*node);

  shape = new osg::Sphere(osg::Vec3(0,0,0),radius());

  Real d = Math::maximum(radius()*2.0,1.0);

  //!!! is this necessary?? - slices isn't used anymore and osg::Sphere *may* be auto LOD anyway (?) !!!
  osg::Node* node0 = createOSGSphere(visualAttributes, Int(22*d));
  osg::Node* node1 = createOSGSphere(visualAttributes, Int(16*d));
  osg::Node* node2 = createOSGSphere(visualAttributes, Int(8*d));
  osg::Node* node3 = createOSGSphere(visualAttributes, 8);

  osg::LOD* lod = new osg::LOD();
  lod->setName("Sphere");
  lod->addChild(node0);
  lod->addChild(node1);
  lod->addChild(node2);
  lod->addChild(node3);

  lod->setRange(0,0,2.0);
  lod->setRange(1,2.0,16.0);
  lod->setRange(2,16.0,120.0*d);
  lod->setRange(3,120.0*d,consts::Infinity);

  if (!(visualAttributes & Visual::ShowAxes))
    node = lod;
  else {
    Group* group = new Group();
    group->addChild( lod );
    group->addChild( createOSGAxes(base::Dimension3(radius(),radius(),radius())*2.0) );
    node = group;
  }

  attributes = visualAttributes;
  return &(*node);
}


base::ref<CollisionModel> Sphere::getCollisionModel(CollisionModel::CollisionModelType modelType) const
{
  if ((collisionModel!=0) &&
      ((this->modelType==modelType) || (modelType==CollisionModel::AnyModel)))
    return collisionModel;

  collisionModel = Shape::getCollisionModel(modelType);
  this->modelType=modelType;

  return collisionModel;
}


void Sphere::serialize(base::Serializer& s)
{
  s(_radius);

  if (s.isInput()) {
    massPropertiesCached = false;
    node = 0;
    collisionModel = ref<CollisionModel>(0);
  }
}



bool Sphere::formatSupported(String format, Real version, ExternalizationType type) const
{
  return ( (format=="xml") && (version==1.0) );
}


void Sphere::externalize(base::Externalizer& e, String format, Real version)
{
  if (format == "") format = "xml";

  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" v"+base::realToString(version)+" unsupported"));

  if (e.isOutput()) {

    DOMElement*  sphereElem = e.createElement("sphere");
    e.setElementAttribute(sphereElem,"radius",base::realToString(_radius));

    e.appendElement(sphereElem);
  }
  else { // input

    massPropertiesCached = false;
    collisionModel = ref<CollisionModel>(0);
    clearVisualCache();

    DOMNode* context = e.context();

    DOMElement* sphereElem = e.getFirstElement(context, "sphere");

    _radius = e.toReal( e.getDefaultedElementAttribute(sphereElem, "radius", "1") );

    e.removeElement(sphereElem);

  }
}


