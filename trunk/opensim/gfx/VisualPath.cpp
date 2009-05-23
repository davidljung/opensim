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

  $Id: VisualPath.cpp 1136 2004-09-28 20:55:11Z jungd $

****************************************************************************/

#include <gfx/VisualPath>


#include <osg/Node>
#include <osg/Transform>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osg/Drawable>
#include <osg/Geometry>

using gfx::VisualPath;

using base::Orient;

using osg::Node;
using osg::Transform;
using osg::MatrixTransform;
using osg::PositionAttitudeTransform;
using osg::Geometry;
using osg::Vec3;


osg::Node* VisualPath::createOSGVisual(Visual::Attributes visualAttributes) const
{
  if ((node!=0) && (attributes==visualAttributes))
    return &(*node);


  // create a set of line segments between the positions at a sequence of t:[0..1]
  //  also explicitly include all distibguished t values.

  array<Real> ta; // array of t values we'll evaluate the path at

  Real t=0.0;
  Int di=0;                          // distinguished t index
  Real dt = distinguishedValue(di);  // "             t value
  while (t<1.0) {

    if ((t > dt) && (di < numDistinguishedValues())) {
      ta.push_back(dt);
      if (!Math::equals(t,dt))
        ta.push_back(t);

      di++;
      if (di < numDistinguishedValues())
        dt = distinguishedValue(di);
    }
    else
      ta.push_back(t);

    t += 0.01;
    if (t>1.0) t=1.0;
  }
  ta.push_back(1.0);


  // create geometry
  osg::Geode* geode = new osg::Geode();

  osg::Geometry* linesGeom = new osg::Geometry();

  osg::Vec3Array* vertices = new osg::Vec3Array((ta.size()-1)*2);
  for(Int i=0; i<ta.size()-1; i++) {
    Point3 p1(position(ta[i]));
    Point3 p2(position(ta[i+1]));
    (*vertices)[2*i].set(p1.x,p1.y,p1.z);
    (*vertices)[2*i+1].set(p2.x,p2.y,p2.z);
  }

  linesGeom->setVertexArray(vertices);

  // set the colors as before, plus using the aobve
  osg::Vec4Array& colors = *new osg::Vec4Array(1);
  colors[0] = osg::Vec4(1.0f,1.0f,0.0f,1.0f);
  linesGeom->setColorArray(&colors);
  linesGeom->setColorBinding(osg::Geometry::BIND_OVERALL);


  // set the normal in the same way as color.
  osg::Vec3Array& normals = *new osg::Vec3Array(1);
  normals[0] = osg::Vec3(0.0f,-1.0f,0.0f);
  linesGeom->setNormalArray(&normals);
  linesGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);

  linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,(ta.size()-1)*2));

  // add the points geomtry to the geode.
  geode->addDrawable(linesGeom);



  // now, if orientation visualization requested, create line segments for zaxis direction vectors at each vertex
  if (zaxisLength > 0) {
    osg::Geometry* vectorsGeom = new osg::Geometry();
    osg::Vec3Array* vecVerts = new osg::Vec3Array(ta.size()*2);
    Vector3 zaxis(0,0,zaxisLength);
    for(Int i=0; i<ta.size(); i++) {
      Point3 p1(position(ta[i]));
      Orient o(orientation(ta[i]));
      Point3 p2( p1 + o.rotate(zaxis) );
      (*vecVerts)[2*i].set(p1.x,p1.y,p1.z);
      (*vecVerts)[2*i+1].set(p2.x,p2.y,p2.z);
    }

    vectorsGeom->setVertexArray(vecVerts);

    vectorsGeom->setColorArray(&colors);
    vectorsGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
    vectorsGeom->setNormalArray(&normals);
    vectorsGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);

    vectorsGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,ta.size()*2));

    // add these points to geode too
    geode->addDrawable(vectorsGeom);
  }


  attributes = visualAttributes;
  geode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

  node = geode;

  return &(*node);
}

