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
  
  $Id: SOLIDCollisionModel.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.2 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/SOLIDCollisionModel>

#include <physics/Box>
#include <physics/Shape>
#include <gfx/Color4>
#include <gfx/Triangle3>
#include <gfx/TriangleContainer>
#include <gfx/TriangleIterator>
#include <gfx/VisualTriangles>

#include <osg/Node>
#include <osg/Group>
#include <osg/Transform>
#include <osg/StateSet>
#include <osg/Material>
#include <osg/Vec4>
#include <osg/PolygonMode>


using physics::SOLIDCollisionModel;
using physics::Shape;
using physics::Box;

using gfx::Triangle3;
using gfx::Color4;
using gfx::TriangleContainer;
using gfx::TriangleIterator;
using gfx::VisualTriangles;

using base::Matrix3;
using base::transpose;
using base::cross;
using base::array;

using osg::Vec4;
using osg::StateSet;


SOLIDCollisionModel::SOLIDCollisionModel(const gfx::TriangleContainer& triangles)
  : shapeRefOwner(true)
{
  buildModel(triangles);
}

SOLIDCollisionModel::SOLIDCollisionModel(ref<const Shape> shape)
  : shapeRefOwner(true)
{
  buildModel(shape);
}

SOLIDCollisionModel::SOLIDCollisionModel(const SOLIDCollisionModel& cm)
  : shapeRef(cm.shapeRef), shapeRefOwner(false)
{
}

SOLIDCollisionModel::~SOLIDCollisionModel()
{
  dtDeleteShape(shapeRef);
}


void SOLIDCollisionModel::buildModel(const gfx::TriangleContainer& triangles)
{
  // Load all the triangles into the SOLID shape
  //  NB: This would be more efficient if it indexed the existing vertex data
  //  rather than copying it. !!

  shapeRef = dtNewComplexShape();

  TriangleContainer::const_iterator t = triangles.begin();
  TriangleContainer::const_iterator end = triangles.end();
  while (t != end) {
    const Triangle3& tri(*t);
    dtBegin(DT_SIMPLEX);
    dtVertex(tri[1].x, tri[1].y, tri[1].z);
    dtVertex(tri[2].x, tri[2].y, tri[2].z);
    dtVertex(tri[3].x, tri[3].y, tri[3].z);
    dtEnd();
    ++t;
  }  

  dtEndComplexShape();
}


void SOLIDCollisionModel::buildModel(ref<const Shape> shape)
{
  // Just use the Shape's triangles for now.
  //  A better approach is to create a specific SOLID primitice corresponding
  //  to the specific shape. !!
  ref<const Shape> s(shape);
  buildModel(VisualTriangles(*s));
}




osg::Node* SOLIDCollisionModel::createOSGVisual(Visual::Attributes visualAttributes) const
{
  if (!(visualAttributes & ShowCollisionModel)
      /*|| tris.empty()*/ ) 
    return new osg::Node();
  
  osg::Node* node = new osg::Node();//createOBBVisualRecurse(b[0], 0);
  node->setName("debug");

  // Set state to be transparent, random colour
  StateSet* state = new osg::StateSet();
  osg::Material* mat = new osg::Material();
  Vec4 col( base::random(), base::random(), base::random(), 1.0);
  mat->setEmission( osg::Material::FRONT_AND_BACK, Vec4(0,0,0,0) );
  mat->setAmbient( osg::Material::FRONT_AND_BACK, col );
  mat->setDiffuse( osg::Material::FRONT_AND_BACK, col );
  mat->setSpecular( osg::Material::FRONT_AND_BACK, Vec4(1,1,1,0) );
  mat->setShininess( osg::Material::FRONT_AND_BACK, 0.3);
  state->setAttribute( mat );
  state->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);

  osg::PolygonMode* polyMode = new osg::PolygonMode;
  polyMode->setMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );
  state->setAttributeAndModes(polyMode,osg::StateAttribute::ON);

  node->setStateSet(state);
  
  return node;
}

