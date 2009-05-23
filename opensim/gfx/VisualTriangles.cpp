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
  
  $Id: VisualTriangles.cpp 1030 2004-02-11 20:46:17Z jungd $
  $Revision: 1.5 $
  $Date: 2004-02-11 15:46:17 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <gfx/VisualTriangles>

#include <gfx/TriangleIterator>

#include <osg/Node>
#include <osg/Transform>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Group>
#include <osg/Switch>
#include <osg/LOD>
#include <osg/Geode>
#include <osg/Drawable>
#include <osg/Geometry>
#include <osg/PrimitiveSet>


using gfx::VisualTriangles;
using gfx::TriangleContainer;
using gfx::TriangleIterator;

using osg::Node;
using osg::Group;
using osg::Switch;
using osg::Transform;
using osg::MatrixTransform;
using osg::PositionAttitudeTransform;
using osg::Geometry;
using osg::PrimitiveSet;
using osg::DrawArrays;
using osg::DrawElementsUShort;
using osg::Vec3;
using osg::Array;
using osg::Vec3Array;
using osg::NodeVisitor;



VisualTriangles::VisualTriangles(const Visual& visual)
  : LODChild(0)
{
  if (visual.visualTypeSupported(Visual::OSGVisual)) {
    extractTriangles(visual.createOSGVisual(Visual::VerticesOnly));
  }
  else
    throw std::runtime_error(Exception("Visual type not supported")); 
}


VisualTriangles::VisualTriangles(const Visual& visual, Int LODChild)
  : LODChild(LODChild)
{
  if (visual.visualTypeSupported(Visual::OSGVisual)) {
    extractTriangles(visual.createOSGVisual(Visual::VerticesOnly));
  }
  else
    throw std::runtime_error(Exception("Visual type not supported")); 
}


void VisualTriangles::extractTriangles(osg::Node* node)
{
  TriangleExtractor extractor(triangles,LODChild);
  extractor.setTraversalMode(NodeVisitor::TRAVERSE_ACTIVE_CHILDREN);
  node->accept(extractor);
}



VisualTriangles::const_iterator TriangleContainer::begin() const
{
  return TriangleIterator(*this, true); 
}

VisualTriangles::const_iterator TriangleContainer::end() const 
{
  return TriangleIterator(*this, false); 
}


void VisualTriangles::TriangleExtractor::apply(osg::Transform& node)
{
  if (node.getName() != "debug") {

    if (instanceof(node, osg::MatrixTransform)) {

      osg::MatrixTransform& mt(narrow_cast<MatrixTransform>(node));
      

      Matrix4 t(mt.getMatrix());
      transform.push( transform.top()*t ); // !!! check mul order
      
      node.traverse(*this);
      
      transform.pop();
    }
    else if (instanceof(node, osg::PositionAttitudeTransform)) {
      //!!!
      throw std::runtime_error(Exception("PositionAttitudeTransform not implemented"));
    }
    else {
      Logln("unknown transform type, not applied (treated as identity)");
      node.traverse(*this);
    }

  }
}


void VisualTriangles::TriangleExtractor::apply(osg::Switch& node)
{
  if (node.getName() != "debug") 
    node.traverse(*this);
}

void VisualTriangles::TriangleExtractor::apply(osg::Group& node)
{
  if (node.getName() != "debug") // don't traverse debugging graphics (axes etc.)
    node.traverse(*this);
}

void VisualTriangles::TriangleExtractor::apply(osg::LOD& node)
{
  if (node.getName() != "debug") {
    int n = node.getNumChildren();
    int child = (int(LODChild)<n)?LODChild:(n-1);
    node.getChild(child)->accept(*this);
  }
}

void VisualTriangles::TriangleExtractor::apply(osg::Geode &node)
{
  // Extract triangles from any GeoSets in the Geode
  int numDrawables = node.getNumDrawables();
  for(Int d=0; d<Int(numDrawables); d++) {
    osg::Drawable* drawable = node.getDrawable(d);
/*
    if (instanceof(*drawable,osg::GeoSet)) {
      GeoSet* geoSet = narrow_cast<GeoSet>((osg::Drawable*)drawable); // !!! check this
      
      Int numPrims = geoSet->getNumPrims();
      const Vec3* coords = geoSet->getCoords();
      const int* lens = geoSet->getPrimLengths();
      const Vec3* c=coords;

      Assertm(lens,"lens!=0"); //!!! why would it be 0? (it sometimes is - OSG bug?)

      switch (geoSet->getPrimType()) {
      case GeoSet::NO_TYPE: // no triangles
      case GeoSet::POINTS:
      case GeoSet::LINES:
      case GeoSet::LINE_STRIP:
      case GeoSet::FLAT_LINE_STRIP:
      case GeoSet::LINE_LOOP:
      break;
	
      case GeoSet::TRIANGLES: {
        for(Int p=0; p < numPrims; p++) {
          Int n = lens[p];
          for(Int v=0; v < n; v+=3) 
            triangles.push_back(Triangle3(c[v],c[v+1],c[v+2]));
          c +=n;
        }
      }
      break;

      case GeoSet::FLAT_TRIANGLE_STRIP:
      case GeoSet::TRIANGLE_STRIP: {
        for(Int p=0; p < numPrims; p++) {
          Int n = lens[p];
          for(Int v=2; v < n; v++) {
            if ((v%2)==0)
              triangles.push_back(Triangle3(c[v-2],c[v-1],c[v]));
            else
              triangles.push_back(Triangle3(c[v-1],c[v-2],c[v]));
          }
          c +=n;
        }
      }
      break;
	
      case GeoSet::TRIANGLE_FAN:
      case GeoSet::FLAT_TRIANGLE_FAN: {
        for(Int p=0; p < numPrims; p++) {
          Int n = lens[p];
          Vec3 f(c[0]);
          for(Int v=2; v < n; v++) 
            triangles.push_back(Triangle3(f,c[v-1],c[v]));
          c +=n;
        }
      }
      break;
	
      case GeoSet::QUADS: {
        for(Int p=0; p < numPrims; p++) {
          Int n = lens[p];
          for(Int v=0; v < n; v+=4) {
            triangles.push_back(Triangle3(c[v],c[v+1],c[v+2]));
            triangles.push_back(Triangle3(c[v+2],c[v+3],c[v]));
          }
          c +=n;
        }
      }
      break;

      case GeoSet::QUAD_STRIP: {
        for(Int p=0; p < numPrims; p++) {
          Int n = lens[p];
          for(Int v=2; v < n; v+=2) {
            triangles.push_back(Triangle3(c[v-2],c[v-1],c[v]));
            triangles.push_back(Triangle3(c[v],c[v-1],c[v+1]));
          }
          c +=n;
        }
      }
      break;
	
      case GeoSet::POLYGON: {
        Logln("Tesselation of polygons into triangles is not yet supported:");
        Logln(" will return convex version of polygon.");
        for(Int p=0; p < numPrims; p++) {
          Int n = lens[p];
          Vec3 f(c[0]);
          for(Int v=2; v < n; v++) 
            triangles.push_back(Triangle3(f,c[v-1],c[v]));
          c +=n;
        }
      }
      break;

      default: ;
      Logln("Warning: Unknown osg::GetSet primitive type.");
      }

   
    }
    else*/ if (instanceof(*drawable,osg::Geometry)) {
      Geometry* geom = narrow_cast<Geometry>((osg::Drawable*)drawable); 

      const Array& va(*geom->getVertexArray());
      Assert(va.getDataType() == Array::Vec3ArrayType);
      const Vec3Array& v(dynamic_cast<const Vec3Array&>(va));

      for(Int p=0; p<geom->getNumPrimitiveSets(); p++) {
	
        const PrimitiveSet* pset(geom->getPrimitiveSet(p));
	
        Int n = pset->getNumIndices();

        switch (pset->getMode()) {
          case PrimitiveSet::POINTS: // no triangles
          case PrimitiveSet::LINES:
          case PrimitiveSet::LINE_STRIP:
          case PrimitiveSet::LINE_LOOP:
          break; 
          
          case PrimitiveSet::TRIANGLES: {
            for(Int i=0; i<n; i+=3) {
              Int v0(pset->index(i));
              Int v1(pset->index(i+1));
              Int v2(pset->index(i+2));
              triangles.push_back(Triangle3(v[v0],v[v1],v[v2]));
            }
          } break;
          
          case PrimitiveSet::TRIANGLE_STRIP: {
            for(Int i=2; i < n; i++) {
              Int vi(pset->index(i));
              Int vim1(pset->index(i-1));
              Int vim2(pset->index(i-2));
              if ((i%2)==0)
                triangles.push_back(Triangle3(v[vim2],v[vim1],v[vi]));
              else
                triangles.push_back(Triangle3(v[vim1],v[vim2],v[vi]));
            }
            
          } break;
          
          case PrimitiveSet::TRIANGLE_FAN: {
            Point3 f(v[pset->index(0)]);
            for(Int i=2; i < n; i++) {
              Int vi(pset->index(i));
              Int vim1(pset->index(i-1));
              triangles.push_back(Triangle3(f,v[vim1],v[vi]));
            }
          } break;
          
          case PrimitiveSet::QUADS: {
            for(Int i=0; i < n; i+=4) {
              Int vi(pset->index(i));
              Int vip1(pset->index(i+1));
              Int vip2(pset->index(i+2));
              Int vip3(pset->index(i+3));
              triangles.push_back(Triangle3(v[vi],v[vip1],v[vip2]));
              triangles.push_back(Triangle3(v[vip2],v[vip3],v[vi]));
            }
          } break;
          
          case PrimitiveSet::QUAD_STRIP: {
            for(Int i=2; i < n; i+=2) {
              Int vi(pset->index(i));
              Int vim1(pset->index(i-1));
              Int vim2(pset->index(i-2));
              Int vip1(pset->index(i+1));
              triangles.push_back(Triangle3(v[vim2],v[vim1],v[vi]));
              triangles.push_back(Triangle3(v[vi],v[vim1],v[vip1]));
            }
          } break;
          
          case PrimitiveSet::POLYGON: {
            Logln("Tesselation of polygons into triangles is not yet supported.");
            Unimplemented;
          } break;
          
          default:
          Logln("Warning: Unsupported PrimitiveSet::Mode");
          
        } // switch

      }
    }
    else
      Logln("Warning: Unsupported osg::Drawable subclass " << drawable->className());

  }

}
