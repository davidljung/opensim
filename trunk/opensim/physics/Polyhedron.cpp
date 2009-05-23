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
  
  $Id: Polyhedron.cpp 1151 2004-09-28 21:12:49Z jungd $
 
****************************************************************************/

#include <physics/Polyhedron>

#include <base/Application>
#include <base/File>
#include <base/VDirectory>
#include <base/BinarySerializer>
#include <base/Cache>
#include <base/Dimension3>
#include <base/io_error>
#include <base/Externalizer>
#include <gfx/VisualTriangles>
#include <gfx/TriangleIterator>
#include <gfx/Triangle3>
#include <physics/Material>
//#include <physics/OBBCollisionModel>
#include <physics/GJKCollisionModel>


#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/Group>
#include <osg/Geode>
#include <osg/ShapeDrawable>

using physics::Polyhedron;

using base::Application;
using base::File;
using base::VFile;
using base::VDirectory;
using base::Serializer;
using base::BinarySerializer;
using base::Cache;
using base::reflist;
using gfx::VisualTriangles;
using gfx::Segment3;
using physics::BoundingBox;
using physics::BoundingSphere;
using physics::MassProperties;
using physics::Material;
using physics::CollisionModel;

using osg::Group;
using osg::Vec3;



const Real coordEps = 1e-10;


/// \todo consider building the geometry on demand, rather than
/// always building and copying it around. Perhaps seperate it out
/// into a different class for representation.  So we can just construct
/// winged-edge if needed (e.g. after convex hull conversion of splitting
/// into convex parts, for example)
Polyhedron::Polyhedron() 
  : boundsCached(false), massPropertiesCached(false),
    model(0), node(0), collisionModel(0) 
{
}

Polyhedron::Polyhedron(ref<base::VFile> file) throw(std::invalid_argument, base::io_error)
  : boundsCached(false), massPropertiesCached(false)
{
  // First see if the osg model & polyhedron geometry data has
  //  been cached for this file
  ref<Cache> cache = Application::getInstance()->universe()->cache();
  ref<VDirectory> d = cache->getCache(file->pathName());
  if (d->contains(file->name())) { // cached

    // Deserialize from cache file
    BinarySerializer in(Serializer::Input, d->file(file->name()));
    in(*this);

  }
  else { // not cached, build geometry & cache it

    // Load OSG model
    //  OSG doesn't support reading from a stream, only a real file
    if (instanceof(*file,File)) {
      try {
	model = osgDB::readNodeFile( file->pathName().str().c_str() );
      } catch (std::exception&) {
	model = 0;
      }
      if (model==0)
	throw std::runtime_error(Exception("Error loading model file "+file->pathName().str()));

      shapeHasAppearance = true;
      
      // build geometry from OSG model's Visual
      Logln("Building Polyhedron data-structure from " << file->name().str());
      buildGeometry(VisualTriangles(*this));
    }
    else
      throw std::invalid_argument(Exception("Can only load from standard Unix files.  Sorry"));

    // Serialize to cache file
    ref<VFile> cacheFile = d->createFile(file->name());
    BinarySerializer out(Serializer::Output, cacheFile);
    out(*this);
    out.flush();
    cacheFile->close();
  }

}

/// \todo test (normals?, mesh->setIncides() - yes indices needed, implement)
Polyhedron::Polyhedron(const gfx::TriangleContainer& tris)
  : boundsCached(false), massPropertiesCached(false), node(0)
{
  // Put triangles into a model (OSGVisual only for now)
   osg::TriangleMesh* mesh = new osg::TriangleMesh();

   array<Vec3>& coords = *new array<Vec3>(0,128);
   array<Vec3>& normals = *new array<Vec3>(0,128);
   gfx::TriangleContainer::const_iterator i = tris.begin();
   gfx::TriangleContainer::const_iterator end = tris.end();
   while (i != end) {
     const gfx::Triangle3& t(*i);
     coords.push_back(t(1).toVec3());
     coords.push_back(t(2).toVec3());
     coords.push_back(t(3).toVec3());
     base::Vector3 n(t.normal());
     n.normalize();
     normals.push_back(n.toVec3());
     ++i;
   }
   coords.trim();
   normals.trim();

   
   osg::Vec3Array* v = new osg::Vec3Array(coords.size());
   for(Int i=0; i<coords.size(); i++)
     (*v)[i] = coords[i];
   mesh->setVertices(v);
   
   osg::Geode* geode = new osg::Geode();
   geode->setName("Polyhedron(TriangleContainer)");
   geode->addDrawable(new osg::ShapeDrawable(mesh));
   model = geode;

   buildGeometry(tris);
}

Polyhedron::Polyhedron(osg::Node& n)
  : boundsCached(false), massPropertiesCached(false)
{
  model = &n;
  buildGeometry(VisualTriangles(*this));
}


Polyhedron::Polyhedron(const Polyhedron& p)
  : ComplexShape(p), 
    boundsCached(false), massPropertiesCached(false), 
    verts(p.verts), edges(p.edges), polys(p.polys),
    model(model)
{
}


Polyhedron::Polyhedron(const Shape& s)
  : Shape(s), 
    boundsCached(true), 
    boundingBox(s.getBoundingBox()), boundingSphere(s.getBoundingSphere()),
    massPropertiesCached(false)
{
  if (s.visualTypeSupported(OSGVisual)) {
    model = s.createOSGVisual();
    buildGeometry(VisualTriangles(*this));
  }
  else
    throw std::invalid_argument(Exception("Shape must support OSGVisual"));
}


Polyhedron::~Polyhedron()
{
  // The geometry structures contain circular references, so we must
  //  break the links so that they will be reclaimed properly
  VertexList::iterator v = verts.begin();
  VertexList::iterator vend = verts.end();
  while (v != vend) {
    (*v)->edges.clear();
    ++v;
  }  

  EdgeList::iterator ei = edges.begin();
  EdgeList::iterator eend = edges.end();
  while (ei != eend) {
    (*ei)->v1 = ref<Vertex>(0);
    (*ei)->v2 = ref<Vertex>(0);
    (*ei)->poly1 = ref<Polygon>(0);
    (*ei)->poly2 = ref<Polygon>(0);
    ++ei;
  }

  PolygonList::iterator p = polys.begin();
  PolygonList::iterator pend = polys.begin();
  while (p != pend) {
    (*p)->edges.clear();
    ++p;
  }
}




const MassProperties& Polyhedron::getMassProperties(ref<const Material> material) const
{ 
  if (massPropertiesCached && (density == material->density()))
    return massProperties;

  density = material->density();
  massProperties = MassProperties(VisualTriangles(*this),material);

  massPropertiesCached = true;
  return massProperties;
}



Segment3 Polyhedron::shortestSegmentBetween(const base::Transform& t, const Point3& p) const
{
  Unimplemented;
}


Segment3 Polyhedron::shortestSegmentBetween(const base::Transform& t, const gfx::Segment3& s) const
{
  Unimplemented;
}


Segment3 Polyhedron::shortestSegmentBetween(const base::Transform& t, const gfx::Triangle3& tri) const
{
  Unimplemented;
}


Segment3 Polyhedron::shortestSegmentBetween(const base::Transform& t, const gfx::Quad3& q) const
{
  Unimplemented;
}


Segment3 Polyhedron::shortestSegmentBetween(const base::Transform& t1, ref<const Shape> s, const base::Transform& t2) const
{
  Unimplemented;
}




// CollisionModelProvider
base::ref<CollisionModel> Polyhedron::getCollisionModel(CollisionModel::CollisionModelType modelType) const
{
  if ((collisionModel!=0) && 
      ((this->modelType==modelType) || (modelType==CollisionModel::AnyModel)))
    return collisionModel;
  
  collisionModel = Shape::getCollisionModel(modelType);
  this->modelType=modelType;

  return collisionModel;
}









osg::Node* Polyhedron::createOSGVisual(Attributes visualAttributes) const
{
  if ((node!=0) && (visualAttributes==attributes))
    return &(*node);

  if (model==0) return new osg::Group(); // empty

  if (!(visualAttributes & Visual::ShowAxes)) 
    node = model;
  else {
    // create axes 
    Group* group = new Group();
    group->addChild( &(*model) );
    group->addChild( createOSGAxes(getBoundingBox().getDimension()/2.0) );
    node = group;
  }

  attributes = visualAttributes;
  return &(*node);
}


BoundingBox Polyhedron::getBoundingBox() const
{
  if (!boundsCached)
    computeBounds();
  return boundingBox;
}

BoundingSphere Polyhedron::getBoundingSphere() const
{
  if (!boundsCached)
    computeBounds();
  return boundingSphere;
}


void Polyhedron::computeBounds() const
{
  // Let OSG calculate the BoundingSphere of the visual
  const osg::BoundingSphere& bs = createOSGVisual(gfx::Visual::VerticesOnly)->getBound();
  Real radius = bs.radius();
  boundingSphere = BoundingSphere(bs.center(), radius);
  // Now calculate bounding box to enclose the sphere
  boundingBox.setCenter(bs.center());
  boundingBox.setDimension(base::Dimension3(radius,radius,radius)*2.0);

  boundsCached=true;
}



// methods for building the geometric model

ref<Polyhedron::Vertex> Polyhedron::vertex(const Point3& p)
{
  // search backward through existing vertices
  // (for most models existing vertices
  //  that are referenced tend to be ones that have been
  //  encountered reciently)
  VertexList::reverse_iterator v = verts.rbegin();
  VertexList::reverse_iterator end = verts.rend();
  while (v != end) {
    if ((*v)->coord.equals(p, coordEps))
      return (*v);
    ++v;
  }

  // not found, so create a new vertex
  ref<Vertex> vert(NewObj Vertex(p));
  verts.push_back(vert);
  return vert;
}


ref<Polyhedron::Edge> Polyhedron::edge(ref<Vertex> v1, ref<Vertex> v2)
{
  // if edge v1-v2 is preexisting, then it should be in v1's list of
  //  edges (or v2)
  EdgeList::const_iterator ei = v1->edges_begin();
  EdgeList::const_iterator end = v1->edges_end();
  while (ei != end) {
    ref<Edge> e(*ei); 
    if (   ((e->v1 == v1) && (e->v2 == v2))
	|| ((e->v1 == v2) && (e->v2 == v1)) )
      return e;
    ++ei;
  }

  // wasn't there, so create a new edge
  ref<Edge> edge(NewObj Edge(v1,v2));
  v1->edges.push_back(edge);
  v2->edges.push_back(edge);
  edges.push_back(edge);
  return edge;
}



void Polyhedron::buildGeometry(const gfx::TriangleContainer& tris)
{
  gfx::TriangleContainer::const_iterator i = tris.begin();
  gfx::TriangleContainer::const_iterator end = tris.end();

  while (i != end) {
    const gfx::Triangle3& t(*i);

    ref<Vertex> v1 = vertex(t[0]);
    ref<Vertex> v2 = vertex(t[1]);
    ref<Vertex> v3 = vertex(t[2]);
    
    ref<Edge> e1 = edge(v1, v2);
    ref<Edge> e2 = edge(v2, v3);
    ref<Edge> e3 = edge(v3, v1);
    
    ref<Polygon> poly(NewObj Polygon());
    poly->addEdge(e1);
    poly->addEdge(e2);
    poly->addEdge(e3);

    e1->addPoly(poly);
    e2->addPoly(poly);
    e3->addPoly(poly);
    
    polys.push_back(poly);
    
    ++i;
  }

}


void Polyhedron::serialize(base::Serializer& s)
{
  // first serialize the OSG model
  //  (currently the only way to do that with OSG is to write it to
  //    a file, then copy the file to the serialization stream.
  //    The reverse for input)

  bool emptyModel = (model == 0);
  s(emptyModel);

  if (!emptyModel) {

    ref<VDirectory> tempDir = Application::getInstance()->filesystem()->temp();
    // create a file "polyhedronN.osg" where N is the first Int such that the
    //  name is available
    Int pn = 0;
    ref<VFile> tempFile;
    String name;
    do {
      name = String("polyhedron")+base::intToString(pn)+".osg";
      try {
	tempFile = tempDir->createFile( name );
      } catch (base::io_error&) {
	tempFile = ref<VFile>(0);
	pn++;
      }
    } while (tempFile == 0);

    
    if (s.isOutput()) { // Output
      // write model to tmpFile
      tempFile->close();
      osgDB::writeNodeFile( *model, tempFile->pathName().str().c_str() );

      // copy all the bytes from tmpFile to a memory buffer (we need the size in advance)
      array<char> mbuf(0,32768);
      std::istream& in = tempFile->istream();
      while (in.good()) {
	mbuf.at(mbuf.size()) = in.get();
      }
      tempFile->close();
      tempDir->deleteFile( name );

      // output size to s
      Int osgModelSize = mbuf.size()-1; // size (bytes)
      s(osgModelSize);
      // an the buffer
      for(Int i=0; i<osgModelSize; i++)
	s(mbuf[i]);
    }
    else { // Input
      // read osg data from s and write it to tmpFile
      Int osgModelSize;
      s(osgModelSize); // read size (bytes)

      std::ostream& out = tempFile->ostream();
      char c;
      for(Int i=0; i<osgModelSize; i++) {
	s(c);
	out << c;
      } 
      tempFile->close();
      
      // now ask OSG to read tmpFile into a model
      model = osgDB::readNodeFile( tempFile->pathName().str().c_str() );

      tempDir->deleteFile( name );
    }

  }
  else { // emptyModel
    if (s.isInput()) model = 0;
  }

  // finally, now serialize the polyhedron geometry data
  bool follow = s.followReferences(false);

  s(verts)(edges)(polys);
  s.followReferences(follow);
  

  // zero out cached data
  if (s.isInput()) {
    boundsCached = massPropertiesCached = 0;
    node = 0;
    collisionModel = ref<CollisionModel>(0);
  }

}




void Polyhedron::getAdjacentVertices(const Point3& v, array<Point3>& adjacent) const
{
  /*
  adjacent.clear();
  Int vi;
  for(vi=0; vi<verts.size(); vi++) {
    if (verts[vi].coord.equals(v,coordEps)) 
      break;
  }
  if (vi==verts.size()) return;

  // for each edge
  array<Int>::const_iterator ei = verts[vi].edges.begin();
  array<Int>::const_iterator end = verts[vi].edges.end();
  while (ei != end) {
    const Edge& e(edges[*ei]);
    if (e.vi1 != vi)
      adjacent.push_back(verts[e.vi1].coord);
    else
      adjacent.push_back(verts[e.vi2].coord);
    ++ei;
  }
  */
  Unimplemented;
}




bool Polyhedron::formatSupported(String format, Real version, ExternalizationType type) const
{ 
  return ( (format=="xml") && (version==1.0) ); 
}


void Polyhedron::externalize(base::Externalizer& e, String format, Real version)
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


