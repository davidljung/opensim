/****************************************************************************
  Copyright (C)2002 David Jung <opensim@pobox.com>

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
  
  $Id: ODECollisionModel.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.4 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/ODECollisionModel>

#include <physics/Shape>
#include <physics/Box>
#include <physics/Sphere>
#include <physics/Cylinder>
#include <physics/Polyhedron>
/*
#include <gfx/Color4>
#include <gfx/Triangle3>
#include <gfx/TriangleContainer>
#include <gfx/TriangleIterator>
#include <gfx/VisualTriangles>
#include <gfx/IndexedPoint3Array>
*/
#include <base/Dimension3>

#include <osg/Node>
#include <osg/Group>
#include <osg/Transform>
#include <osg/StateSet>
#include <osg/Material>
#include <osg/Vec4>
#include <osg/PolygonMode>


using physics::ODECollisionModel;
using physics::Shape;
using physics::Box;
using physics::Sphere;
using physics::Cylinder;

/*
using gfx::Triangle3;
using gfx::Color4;
using gfx::TriangleContainer;
using gfx::TriangleIterator;
using gfx::VisualTriangles;
using gfx::IndexedPoint3Array;
*/
using base::Dimension3;
using base::Matrix3;
using base::transpose;
using base::cross;
using base::reflist;
using base::dynamic_cast_ref;


using osg::Vec4;
using osg::StateSet;


ODECollisionModel::ODECollisionModel(ref<const Shape> shape)
  : shape(shape)
{
  create(shape);
}


ODECollisionModel::ODECollisionModel(const ODECollisionModel& cm)
  : shape(cm.shape)
{
  create(shape);
}


ODECollisionModel::~ODECollisionModel()
{
  dGeomDestroy(geomID);
}


void ODECollisionModel::create(ref<const Shape> shape) 
{
  if (instanceof(*shape, const Box)) {
    ref<const Box> box = dynamic_cast_ref<const Box>(shape);
    Dimension3 dim = box->dimensions();
    geomID = dCreateBox(0, dim.x, dim.y, dim.z);
  }
  else if (instanceof(*shape, const Sphere)) {
    ref<const Sphere> sphere = dynamic_cast_ref<const Sphere>(shape);
    geomID = dCreateSphere(0, sphere->radius());
  }
  else if (instanceof(*shape, const Cylinder)) {
    ref<const Cylinder> cylinder = dynamic_cast_ref<const Cylinder>(shape);
    geomID = dCreateCCylinder(0, cylinder->radius(), cylinder->height());
  }
  else {
    Logln("Unsupported Shape: " << shape->className() << ", using bounding sphere for collision region");
    Real radius = (shape->getBoundingSphere()).radius();
    geomID = dCreateSphere(0, radius);
  }
}



osg::Node* ODECollisionModel::createOSGVisual(Visual::Attributes visualAttributes) const
{
  // !!! nothing to show for the model for now
  return new osg::Node();
}

