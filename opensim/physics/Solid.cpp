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
  
  $Id: Solid.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.7 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/Solid>
#include <base/ref>

#include <osg/Node>
#include <osg/Transform>
#include <osg/Geode>

using physics::Solid;
using physics::Shape;
using physics::Material;
using physics::CollisionModel;
using physics::MassProperties;

using osg::Node;
using osg::MatrixTransform;
using osg::Geode;


Solid::Solid(ref<const Shape> shape, ref<const Material> material)
  : Body(shape), material(material)
{
}

Solid::Solid(const Solid& s)
  : Body(s), material(s.material)
{
}

Solid::~Solid() 
{
}



MassProperties Solid::massProperties() const
{
  // not cached in Solid, so calculate from shape & material
  //  (concrete derived Solids typically cache mass properties as it is expensive to
  //   compute)
  MassProperties massProps;
  try {

    massProps = getShape()->getMassProperties(material);

  } catch (std::exception& e) {
    Logln("Couldn't calculate Mass properties of shape " << getShape()->className() << ", using default.");

    // Set to the mass properties of the bounding box
    BoundingBox bb(getShape()->getBoundingBox());
    Real dx = bb.getDimension().x;
    Real dy = bb.getDimension().y;
    Real dz = bb.getDimension().z;

    Real volume = dx*dy*dz;
    if (base::equals(volume,0)) {
      Logln("Warning: Bounding box of shape is empty.");
      dx=dy=dz=volume=1.0;
    }

    massProps.mass = volume*material->density();
    Real k = massProps.mass/12.0;
    Matrix3 Ibody;
    Ibody(1,1) = (dy*dy + dz*dz)*k;
    Ibody(2,2) = (dx*dx + dz*dz)*k;
    Ibody(3,3) = (dx*dx + dy*dy)*k;
    massProps.setIbody(Ibody);
  }
  return massProps;
}








void Solid::updateVisual()
{
  if (node != 0) {
    // sync OSGVisual with current state
    Matrix4 m(getOrientation());
    m.setTranslationComponent(getPosition());
    worldTransform->setMatrix(m);
  }
}


osg::Node* Solid::createOSGVisual(Visual::Attributes visualAttributes) const
{
  if ((node!=0) && (attributes==visualAttributes))
    return &(*node);

  if (!getShape()->visualTypeSupported(OSGVisual))
    throw std::runtime_error(Exception("Shape doesn't support OSGVisual"));

  osg::Node& shapeNode = *getShape()->createOSGVisual(visualAttributes);

  // Setup Material
  if (!getShape()->includesAppearance()) {
    shapeNode.setStateSet( &(*material->createState()) );
  }

  //  if (visualAttributes & Visual::ShowEdges)
  
  MatrixTransform* transform = new MatrixTransform();
  transform->setName(className());
  transform->addChild(&shapeNode);
  worldTransform = transform;
  
  Matrix4 m(getOrientation());
  m.setTranslationComponent(getPosition());
  worldTransform->setMatrix(m);

  if (visualAttributes & Visual::ShowCollisionModel) {
    ref<CollisionModel> collisionModel = getShape()->getCollisionModel(CollisionModel::AnyModel);
    if (collisionModel->visualTypeSupported(OSGVisual))
      transform->addChild( collisionModel->createOSGVisual(visualAttributes) );
  }

  node = transform;
  
  return &(*node);
}
