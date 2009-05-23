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
  
  $Id: SimpleTerrain.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.2 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/SimpleTerrain>
#include <physics/Material>
#include <physics/OBBCollisionModel>

#include <gfx/Tesselation>
#include <gfx/Color3>
#include <gfx/Triangle3>
#include <base/array>

using base::array;

using gfx::Triangle3;
using gfx::Color3;
using gfx::Tesselation;

using physics::SimpleTerrain;
using physics::MassProperties;
using physics::BoundingBox;
using physics::BoundingSphere;
using physics::CollisionModel;
using physics::OBBCollisionModel;


SimpleTerrain::SimpleTerrain(HeightField& heightfield) 
	: Terrain(heightfield), massPropertiesCached(false), tesselated(false),
		collisionModelCached(false)
{
}

SimpleTerrain::SimpleTerrain(const SimpleTerrain& t)
	: Terrain(t), massPropertiesCached(false), tesselated(false),
		collisionModelCached(false)
{
}

SimpleTerrain::~SimpleTerrain() 
{
	if (tesselated) delete tesselation; 
	if (collisionModelCached) delete collisionModel;
}


Real& SimpleTerrain::height(Real x, Real y) throw(std::out_of_range)
{
	return heightfield.height(x,y);
}

const Real& SimpleTerrain::height(Real x, Real y) const throw(std::out_of_range)
{
	return heightfield.height(x,y);
}



BoundingBox SimpleTerrain::getBoundingBox() const
{
	return BoundingBox();//!!!
}

BoundingSphere SimpleTerrain::getBoundingSphere() const
{
	return BoundingSphere(); //!!!
}


const MassProperties& SimpleTerrain::getMassProperties(const Material& material) const
{
  if (massPropertiesCached && (massProperties.density == material.density()))
    return massProperties;

  //!!! should we just make a mass properties that is infinite mass?
  // the c.o.m. is not at the origin anyway
  // (of cource, c.o.m. at origin and valid mass properties would be best)
  
  // ANS: No.  Don't use the terrain as a dynamic object in physics or
  //  it may cause instability.  Specifically make it a fixed object

  // construct mass property from Tesselatable 
  massProperties = MassProperties(*this, material);
  
  massPropertiesCached = true;
  return massProperties;
}


const Tesselation& SimpleTerrain::getTesselation(Int properties) const
{
  if (tesselated) {
    if (properties == this->properties)  // same properties, used cached Tesselation
      return *tesselation; 
    else
      delete tesselation;  // different properties, re-tesselate
  }
  
  tesselation = new Tesselation();
  this->properties = properties;
  
  HeightField& hf(heightfield);
  
  //!!! texture coords. not yet implemented
  //!!! sides and bottom not yet implemented
  
  // First, build an array of vertices (one per height field sample)
  array<Point3> vertices(hf.nx()*hf.ny());
  SInt vi=0;
  for(Int yi=0; yi<hf.ny(); yi++)
    for(Int xi=0; xi<hf.nx(); xi++) {
      vertices[vi] = Point3(xi*hf.dx(), hf.height(xi,yi),hf.ysize()-yi*hf.dy());
      vi++;
    }
  
  // Now calulcate normals
  //  (inefficient! - compute the 8 triangles that share each vertex, and average
  //   their normals)
  array<Vector3> normals(hf.nx()*hf.ny());
  array<Triangle3> tri(8);
#define vAt(xxi,yyi) ( (((xxi) < 0) || ((xxi) >= (SInt)hf.nx()) || ((yyi) < 0) || ((yyi) >= (SInt)hf.ny()))?\
  Point3((xxi)*hf.dx(),0.0,hf.ysize()-(yyi)*hf.dy()):(vertices[(yyi)*hf.nx()+(xxi)]) )
  
  if (properties & VertexNormals) {
    
    for(SInt yi=0; yi<(SInt)hf.ny(); yi++)
      for(SInt xi=0; xi<(SInt)hf.nx(); xi++) {
	tri[0].p1 = vAt(xi,yi); tri[0].p2 = vAt(xi+1,yi);   tri[0].p3 = vAt(xi+1,yi+1);
	tri[1].p1 = vAt(xi,yi); tri[1].p2 = vAt(xi+1,yi+1); tri[1].p3 = vAt(xi, yi+1);
	tri[2].p1 = vAt(xi,yi); tri[2].p2 = vAt(xi, yi+1);  tri[2].p3 = vAt(xi-1, yi+1);
	tri[3].p1 = vAt(xi,yi); tri[3].p2 = vAt(xi-1,yi+1); tri[3].p3 = vAt(xi-1,yi);
	tri[4].p1 = vAt(xi,yi); tri[4].p2 = vAt(xi-1,yi);   tri[4].p3 = vAt(xi-1,yi-1);
	tri[5].p1 = vAt(xi,yi); tri[5].p2 = vAt(xi-1,yi-1); tri[5].p3 = vAt(xi,yi-1);
	tri[6].p1 = vAt(xi,yi); tri[6].p2 = vAt(xi,yi-1);   tri[6].p3 = vAt(xi+1,yi-1);
	tri[7].p1 = vAt(xi,yi); tri[7].p2 = vAt(xi+1,yi-1); tri[7].p3 = vAt(xi+1,yi);
	
	// average normals
	Vector3 norm;
	for(Int t=0; t<8; t++) 
	  norm += tri[t].normal();
	norm /= 8.0;
	
	normals[yi*hf.nx() + xi] = norm;
      }
  }
  
  array<Point3>  vertexArray( (hf.nx()-1)*(hf.ny()-1)*2*3 );
  array<Vector3> normalArray( (hf.nx()-1)*(hf.ny()-1)*2*3 );
  
  // Build vertex arrays
#define nAt(xi,yi) normals[(yi)*hf.nx()+(xi)]
  SInt index=0;
  for(SInt xi=1; xi<(SInt)hf.nx(); xi++)
    for(SInt yi=1; yi<(SInt)hf.ny(); yi++) {
      vertexArray[index+2]   = vAt(xi, yi);
      vertexArray[index+1] = vAt(xi-1,yi-1);
      vertexArray[index] = vAt(xi-1, yi);
      if (properties & VertexNormals) {
	normalArray[index+2]   = nAt(xi,yi);
	normalArray[index+1] = nAt(xi-1,yi-1);
	normalArray[index] = nAt(xi-1,yi);
      }
      index+=3;
      
      vertexArray[index+2]   = vAt(xi-1, yi-1);
      vertexArray[index+1] = vAt(xi,yi);
      vertexArray[index] = vAt(xi, yi-1);
      if (properties & VertexNormals) {
	normalArray[index+2]   = nAt(xi-1,yi-1);
	normalArray[index+1] = nAt(xi,yi);
	normalArray[index] = nAt(xi,yi-1);
      }
      index+=3;
    }
  
  TriangleArray& terrainTris = *new TriangleArray(&vertexArray,0,
						  (properties & VertexNormals)?&normalArray:0,
						  0,0,false);
  
  tesselation->insert(terrainTris);
  tesselated=true;

  return *tesselation;
}


osg::GeoSet& SimpleTerrain::getGeoSet(Int properties=VertexNormals) const
{
  return *new osg::GeoSet(); // implement!!!
}


CollisionModel& SimpleTerrain::getCollisionModel(CollisionModel::CollisionModelType modelType) const
{
	if (collisionModelCached && 
			((this->modelType==modelType) || (modelType==CollisionModel::AnyModel)))
		return *collisionModel;
	
	if (   (modelType == CollisionModel::OBBModel) 
			|| (modelType == CollisionModel::AnyModel)) {
		if (collisionModelCached) delete collisionModel;
		collisionModel = new OBBCollisionModel(*this);
		this->modelType=modelType;
		collisionModelCached=true;
		return *collisionModel;
	}
	else
		throw std::invalid_argument(Exception("CollisionModelType not supported"));
}

 
