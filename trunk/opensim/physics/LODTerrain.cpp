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
  
  $Id: LODTerrain.cpp 1139 2004-09-28 21:01:10Z jungd $
 
****************************************************************************/

#include <physics/LODTerrain>

#include <base/Externalizer>
#include <base/array>
#include <base/File>

#include <gfx/VisualTriangles>

#include <physics/Material>
#include <physics/OBBCollisionModel>
#include <physics/GJKCollisionModel>

#include <osg/Node>
#include <osg/StateSet>
#include <osg/Fog>


using base::array;

using gfx::Segment3;
using gfx::CLODTerrainRenderer;
using gfx::CLODTerrainDrawable;

using physics::LODTerrain;
using physics::MassProperties;
using physics::BoundingBox;
using physics::BoundingSphere;
using physics::CollisionModel;
using physics::OBBCollisionModel;
using physics::GJKCollisionModel;


LODTerrain::LODTerrain() 
  : massPropertiesCached(false), boundsCached(false)
{
  shapeHasAppearance=true;
}

LODTerrain::LODTerrain(const LODTerrain& t)
  : Terrain(t), renderer(t.renderer), drawable(t.drawable),
    massPropertiesCached(false), boundsCached(false)
  
{
  shapeHasAppearance=true;
}

LODTerrain::~LODTerrain() 
{
}


void LODTerrain::loadMap(ref<base::VFile> mapfile) throw(std::invalid_argument, base::io_error)
{
  if (instanceof(*mapfile,base::File)) {
    renderer = new CLODTerrainRenderer( const_cast<char*>(mapfile->pathName().str().c_str()), 40000); // fix 40000 !!!!
    renderer->SetDetailThreshold(25.0f);
    drawable = new CLODTerrainDrawable();
    drawable->SetTerrain(&(*renderer));
    boundsCached=massPropertiesCached=false;
  }
  else
    throw std::invalid_argument(Exception("Can only load from standard Unix files.  Sorry"));
  
}

void LODTerrain::loadHeightField(ref<HeightField> heightfield) throw(std::invalid_argument, base::io_error)
{
  boundsCached=massPropertiesCached=false;
  throw new std::invalid_argument(Exception("not implemented"));
}


void LODTerrain::setHeight(Real x, Real y, Real h) throw(std::out_of_range)
{
  if (renderer==0) throw new std::out_of_range(Exception("No terrain data loaded"));

  if ((x < 0) || (x > renderer->GetWidth()))
    throw new std::out_of_range(Exception("x out of range"));

  if ((y < 0) || (y > renderer->GetHeight()))
    throw new std::out_of_range(Exception("y out of range"));

  // implement !!!! need to add a set method to CLODTerrainRenderer (or a get index from x,y)
}

Real LODTerrain::getHeight(Real x, Real y) const throw(std::out_of_range)
{
  if (renderer==0) throw new std::out_of_range(Exception("No terrain data loaded"));

  if ((x < 0) || (x > renderer->GetWidth()))
    throw new std::out_of_range(Exception("x out of range"));

  if ((y < 0) || (y > renderer->GetHeight()))
    throw new std::out_of_range(Exception("y out of range"));

  gfx::CLODTerrainRenderer* r = const_cast<gfx::CLODTerrainRenderer*>(&(*renderer));  // yuk!!!
  return r->GetElevation(x,y);
}

base::Dimension3 LODTerrain::getDimension() const
{
  return boundingBox.getDimension();
}

BoundingBox LODTerrain::getBoundingBox() const
{
  if (!boundsCached)
    computeBounds();
  return boundingBox;
}

BoundingSphere LODTerrain::getBoundingSphere() const
{
  if (!boundsCached)
    computeBounds();
  return boundingSphere;
}


void LODTerrain::computeBounds() const
{
  if (renderer!=0) {
    base::Dimension3 dim(base::Dimension3(renderer->GetWidth(),
					  renderer->GetHeight(),renderer->GetMaxElevation()));
    boundingBox.setCenter(base::Point3());
    boundingBox.setDimension(dim);
    
    Real radius = Math::maximum(dim.x,dim.y,dim.z)/2.0;
    boundingSphere = BoundingSphere(base::Point3(), radius);
    
  }
  else {
    boundingBox = BoundingBox();
    boundingSphere = BoundingSphere();
  }

}


const MassProperties& LODTerrain::getMassProperties(ref<const Material> material) const
{
  if (massPropertiesCached && (density == material->density()))
    return massProperties;

  //!!! should we just make a mass properties that is infinite mass?
  // the c.o.m. is not at the origin anyway
  // (of cource, c.o.m. at origin and valid mass properties would be best)
  
  // ANS: No.  Don't use the terrain as a dynamic object in physics or
  //  it may cause instability.  Specifically make it a fixed object

  density = material->density();

  // construct mass property from Tesselatable 
  massProperties = MassProperties(gfx::VisualTriangles(*this), material);
  
  massPropertiesCached = true;
  return massProperties;
}


Segment3 LODTerrain::shortestSegmentBetween(const base::Transform& t, const Point3& p) const
{
  Unimplemented;
}


Segment3 LODTerrain::shortestSegmentBetween(const base::Transform& t, const gfx::Segment3& s) const
{
  Unimplemented;
}


Segment3 LODTerrain::shortestSegmentBetween(const base::Transform& t, const gfx::Triangle3& tri) const
{
  Unimplemented;
}


Segment3 LODTerrain::shortestSegmentBetween(const base::Transform& t, const gfx::Quad3& q) const
{
  Unimplemented;
}


Segment3 LODTerrain::shortestSegmentBetween(const base::Transform& t1, ref<const Shape> s, const base::Transform& t2) const
{
  Unimplemented;
}




osg::Node* LODTerrain::createOSGVisual(Attributes visualAttributes) const
{
  if ((node!=0) && (visualAttributes==attributes))
    return &(*node);

  if (renderer==0) return new osg::Node; // no terrain loaded

  // Create a osg::Geode and add the drawable to it
  osg::Geode* geode = new osg::Geode();
  geode->setName("LODTerrain");
  geode->addDrawable( const_cast<gfx::CLODTerrainDrawable*>(&(*drawable)) ); // !!!

  node = geode;

  return &(*node);
}


// Collidable
base::ref<CollisionModel> LODTerrain::getCollisionModel(CollisionModel::CollisionModelType modelType) const
{
  //!!!  throw std::runtime_error(Exception("not implemented")); 
  if ((collisionModel!=0) && 
      ((this->modelType==modelType) || (modelType==CollisionModel::AnyModel)))
    return collisionModel;

  collisionModel = Shape::getCollisionModel(modelType);
  this->modelType=modelType;

  return collisionModel;
}


 
void LODTerrain::serialize(base::Serializer& s)
{
  throw std::runtime_error(Exception("unimplemented"));
}



bool LODTerrain::formatSupported(String format, Real version, ExternalizationType type) const
{ 
  return ( (format=="xml") && (version==1.0) ); 
}


void LODTerrain::externalize(base::Externalizer& e, String format, Real version)
{
  if (format == "") format = "xml";
                                                                                                                                                                                                    
  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" v"+base::realToString(version)+" unsupported"));
                                                                                                                                                                                                    
  if (e.isOutput()) {
    Unimplemented;                                                                                                                                                                                                 
  }
  else {
    Unimplemented;                                                                                                                                                                                                 
  }
}


