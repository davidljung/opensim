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

  $Id: Torus.cpp 1144 2004-09-28 21:03:37Z jungd $
  $Revision: 1.8 $
  $Date: 2004-09-28 17:03:37 -0400 (Tue, 28 Sep 2004) $
  $Author: jungd $

****************************************************************************/

#include <physics/Torus>
#include <physics/Material>
#include <gfx/VisualTriangles>

#include <osg/Group>
#include <osg/Geode>
#include <osg/LOD>


using physics::Torus;
using physics::MassProperties;
using physics::CollisionModel;

using gfx::Segment3;
using gfx::VisualTriangles;

using osg::Node;
using osg::Group;
using osg::Geode;
using osg::LOD;
using osg::Vec3;
using osg::Vec2;


Torus::Torus(Real innerRadius, Real outerRadius)
  : _innerRadius(innerRadius), _outerRadius(outerRadius)
{
}

Torus::Torus(const Torus& t)
  : _innerRadius(t._innerRadius), _outerRadius(t._outerRadius)
{
}

Torus::~Torus()
{
}


const MassProperties& Torus::getMassProperties(ref<const Material> material) const
{
  if (massPropertiesCached && (density == material->density()))
    return massProperties;

  density = material->density();
  massProperties = MassProperties(VisualTriangles(*this),material);

  massPropertiesCached = true;
  return massProperties;
}


Segment3 Torus::shortestSegmentBetween(const base::Transform& t, const Point3& p) const
{
  Unimplemented;
}


Segment3 Torus::shortestSegmentBetween(const base::Transform& t, const gfx::Segment3& s) const
{
  Unimplemented;
}


Segment3 Torus::shortestSegmentBetween(const base::Transform& t, const gfx::Triangle3& tri) const
{
  Unimplemented;
}


Segment3 Torus::shortestSegmentBetween(const base::Transform& t, const gfx::Quad3& q) const
{
  Unimplemented;
}


Segment3 Torus::shortestSegmentBetween(const base::Transform& t1, ref<const Shape> s, const base::Transform& t2) const
{
  Unimplemented;
}


/// \TODO this inherits from ShapeOSGVisualData but doesn't use shape member, perhaps reimplement using osg::TriangleMesh ?
osg::Node* Torus::createOSGTorus(Visual::Attributes visualAttributes,
                                 Int sides, Int rings) const
{
  bool onlyVerts = ((visualAttributes & Visual::VerticesOnly) != 0);

  SInt i, j;
  Real theta, phi, theta1;
  Real cosTheta, sinTheta;
  Real cosTheta1, sinTheta1;
  Real ringDelta, sideDelta;

  Real cosPhi, sinPhi, dist;

  sideDelta = 2.0 * consts::Pi / Real(sides);
  ringDelta = 2.0 * consts::Pi / Real(rings);

  theta = 0.0;
  cosTheta = 1.0;
  sinTheta = 0.0;

  array<Vec3>& coords = *new array<Vec3>(rings*(sides*2+2));
  array<Vec3>& normals = *new array<Vec3>(rings*(sides*2+2));
  //array<Vec2>& texCoords = *new array<Vec2>(rings*(sides*2+2)); // !!! implement
  array<int>& lengths = *new array<int>(rings);
  for(Int i=0; i<rings; i++)
    lengths[i] = sides*2+2;

  Int index=0;

  for (i = rings - 1; i>= 0;i--) {
    theta1 = theta + ringDelta;
    cosTheta1 = Math::cos(theta1);
    sinTheta1 = Math::sin(theta1);

    phi = (sides+1)*sideDelta;//0.0;
    for (j = sides; j>=0; j--) {
      phi = phi - sideDelta;
      cosPhi = Math::cos(phi);
      sinPhi = Math::sin(phi);
      dist = outerRadius() + (innerRadius() * cosPhi);

      if (!onlyVerts) {
        normals[index] = Vec3(cosTheta1 * cosPhi, sinPhi, -sinTheta1 * cosPhi);
        normals[index+1] = Vec3(cosTheta * cosPhi, sinPhi, -sinTheta * cosPhi);
      }
      coords[index] = Vec3(cosTheta1 * dist, innerRadius() * sinPhi, -sinTheta1 * dist);
      coords[index+1] = Vec3(cosTheta * dist, innerRadius() * sinPhi, -sinTheta * dist);
      index+=2;

    }
    theta = theta1;
    cosTheta = cosTheta1;
    sinTheta = sinTheta1;
  }


  GeoSet* torus = new GeoSet();
  torus->setPrimType(GeoSet::QUAD_STRIP);
  torus->setNumPrims(rings);
  torus->setCoords(coords.c_array());
  torus->setPrimLengths(lengths.c_array());
  if (!onlyVerts) {
    torus->setNormals(normals.c_array());
    torus->setNormalBinding(osg::GeoSet::BIND_PERVERTEX);
    //torus->setTextureCoords(texCoords.c_array());
    //torus->setTextureBinding(osg::GeoSet::BIND_PERVERTEX);
  }

  osg::Geode* geode = new osg::Geode();
  geode->addDrawable(torus);

  return geode;
}


osg::Node* Torus::createOSGVisual(Visual::Attributes visualAttributes) const
{
  if (isOSGVisualCached(visualAttributes)) return &(*node);

  Real ri = Math::maximum(1.0,innerRadius()*2.0);
  Real ro = Math::maximum(1.0,outerRadius()*2.0);

  osg::Node* node0 = createOSGTorus(visualAttributes, Int(22*ri), Int(22*ro));
  osg::Node* node1 = createOSGTorus(visualAttributes, Int(16*ri), Int(16*ro));
  osg::Node* node2 = createOSGTorus(visualAttributes, Int(8*ri), Int(12*ro));
  osg::Node* node3 = createOSGTorus(visualAttributes, 6, 8);

  osg::LOD* lod = new osg::LOD();
  lod->setName("Torus");
  lod->addChild(node0);
  lod->addChild(node1);
  lod->addChild(node2);
  lod->addChild(node3);

  lod->setRange(0,0,2.0);
  lod->setRange(1,2.0,16.0);
  lod->setRange(2,16.0,120.0*ro);
  lod->setRange(3,120.0*ro,consts::Infinity);

  if (!(visualAttributes & Visual::ShowAxes))
    node = lod;
  else {
    Group* group = new Group();
    group->addChild( lod );
    group->addChild( createOSGAxes(base::Dimension3(outerRadius(),innerRadius(),outerRadius())*2.0) );
    node = group;
  }

  attributes = visualAttributes;
  return &(*node);
}



base::ref<CollisionModel> Torus::getCollisionModel(CollisionModel::CollisionModelType modelType) const
{
  if ((collisionModel!=0) &&
      ((this->modelType==modelType) || (modelType==CollisionModel::AnyModel)))
    return collisionModel;

  collisionModel = Shape::getCollisionModel(modelType);
  this->modelType=modelType;

  return collisionModel;
}

void Torus::serialize(base::Serializer& s)
{
  s(_innerRadius)(_outerRadius);

  if (s.isInput()) {
    massPropertiesCached = false;
    collisionModel = ref<CollisionModel>(0);
    clearVisualCache();
  }
}

