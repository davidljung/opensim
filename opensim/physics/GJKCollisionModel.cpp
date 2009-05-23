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
  
  $Id: GJKCollisionModel.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.5 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/GJKCollisionModel>

#include <physics/Shape>
#include <physics/Box>
#include <physics/Sphere>
#include <physics/Polyhedron>

#include <gfx/Color4>
#include <gfx/Triangle3>
#include <gfx/TriangleContainer>
#include <gfx/TriangleIterator>
#include <gfx/VisualTriangles>
#include <gfx/IndexedPoint3Array>


using physics::GJKCollisionModel;
using physics::Shape;
using physics::Box;

using gfx::Triangle3;
using gfx::Color4;
using gfx::TriangleContainer;
using gfx::TriangleIterator;
using gfx::VisualTriangles;
using gfx::IndexedPoint3Array;

using base::Matrix3;
using base::transpose;
using base::cross;
using base::reflist;



GJKCollisionModel::GJKCollisionModel(const gfx::TriangleContainer& triangles)
{
  ref<const Polyhedron> poly(NewNamedObj("GJKCollisionModel Polyhedron") Polyhedron(triangles));
  shape = poly;
  supportFunction = NewObj PolyhedronSupport(poly);
}

GJKCollisionModel::GJKCollisionModel(ref<const Shape> shape)
  : shape(shape)
{
  initSupportFunction();
}


GJKCollisionModel::GJKCollisionModel(const GJKCollisionModel& cm)
  : shape(cm.shape)
{
  initSupportFunction();
}

GJKCollisionModel::~GJKCollisionModel()
{
  if (supportFunction) DeleteObj supportFunction;
}


void GJKCollisionModel::initSupportFunction()
{
  if (instanceof(*shape, const Box))
    supportFunction = NewObj BoxSupport(shape);
  else if (instanceof(*shape, const Sphere))
    supportFunction = NewObj SphereSupport(shape);
  else if (instanceof(*shape, const Polyhedron)) 
    supportFunction = NewObj PolyhedronSupport(shape);
  else {
    // if we don't directly support this Shape class, convert it into a Polyhedron via it's Visual
    Logln(shape->className() + " class not directly supported; converting to Polyhedron via Visual.");
    ref<Polyhedron> poly(NewNamedObj("GJKCollisionModel Polyhedron") Polyhedron(VisualTriangles(*shape)));
    shape = poly;
    supportFunction = NewObj PolyhedronSupport(shape);
  }
}



base::Point3 GJKCollisionModel::BoxSupport::operator()(ref<GJKModelState> s, const base::Vector3& v) const
{
  base::Dimension3 ext(box->dimensions()/2.0);
  return Point3(v.x<0? -ext.x : ext.x,
		v.y<0? -ext.y : ext.y,
		v.z<0? -ext.z : ext.z);
}
 

base::Point3 GJKCollisionModel::SphereSupport::operator()(ref<GJKModelState> s, const base::Vector3& v) const
{
  Real l = v.length();

  if (!base::equals(l,0)) {
    Real r = sphere->radius() / l;
    return v*r;
  }
  else 
    return Point3();
}
 

base::Point3 GJKCollisionModel::PolyhedronSupport::operator()(ref<GJKModelState> s, const base::Vector3& v) const
{
  /*

  Real d;
  reflist<Polyhedron::Vertex>::const_iterator vert = poly->vertices_begin();
  reflist<Polyhedron::Vertex>::const_iterator end = poly->vertices_end();
  ref<const Polyhedron::Vertex> c(*vert);
  Real h = dot(c->coordinate(), v);
  while (vert != end) {
    if ((d = dot((*vert)->coordinate(),v)) > h) {
      c = *vert;
      h = d;
    }
    ++vert;
  } 
  return c->coordinate();
  */

  // Start with the last support point (if any) and hill climb until we get to the top
  ref<const Polyhedron::Vertex> c; // current vertex
  if (s->lastSupport != 0)
    c = s->lastSupport;
  else {
    if (poly->vertices_begin() == poly->vertices_end())
      throw std::runtime_error(Exception("can't collide a polyhedron that has no vertices"));
    c = *poly->vertices_begin(); // first vert (any would do)
  }

  Real h = dot(c->coordinate(), v);
  Real d;

  ref<const Polyhedron::Vertex> last_vertex;
  for(;;) {
    // loop through edges of c and check each vert at the other end of the edge (vert)
    Polyhedron::EdgeList::const_iterator_const e = c->edges_begin();
    Polyhedron::EdgeList::const_iterator_const end = c->edges_end();
    
    ref<const Polyhedron::Vertex> vert = (*e)->otherVertex(c);
    d = dot(vert->coordinate(), v);
    while ((e != end) && ((vert == last_vertex)||(d <= h)) ) {
      ++e;
      if (e != end) { //!!! remove??
	vert = (*e)->otherVertex(c);
	d = dot(vert->coordinate(), v);
      }
    }
    // we either found a d > h (that wasn't the last vertex) or exhasted the edges
    if (e == end) { // no more edges; must have the biggest h
      s->lastSupport = c;
      s->lastSupportPoint = c->coordinate();

      return c->coordinate();
    }
    
    last_vertex = c;
    c = vert;
    h = d;
  } // for
  
}





