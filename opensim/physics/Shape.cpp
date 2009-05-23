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
  
  $Id: Shape.cpp 1145 2004-09-28 21:04:39Z jungd $
  
****************************************************************************/

#include <physics/Shape>

#include <base/Externalizer>
#include <base/externalization_error>
#include <gfx/VisualTriangles>
#include <physics/Box>
#include <physics/Sphere>
#include <physics/Cylinder>
#include <physics/Capsule>
#include <physics/Cone>
#include <physics/Polyhedron>
#include <physics/GJKCollisionModel>
#include <physics/OBBCollisionModel>
#include <physics/ODECollisionModel>


#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/StateSet>
#include <osg/Material>


using physics::Shape;

using base::Externalizer;
using base::dom::DOMElement;
using base::dom::DOMNode;
using gfx::Visual;
using gfx::VisualTriangles;
using physics::Cylinder;
using physics::CollisionModel;
using physics::GJKCollisionModel;
using physics::OBBCollisionModel;
using physics::ODECollisionModel;

using osg::Group;
using osg::MatrixTransform;
using osg::Vec3;
using osg::Vec4;
using osg::StateSet;


base::ref<CollisionModel> Shape::getCollisionModel(CollisionModel::CollisionModelType modelType) const
{
  if (modelType == CollisionModel::AnyModel)
    return ref<CollisionModel>(NewObj GJKCollisionModel(ref<const Shape>(this)));

  // Construct known CollisionModels from Shape or Visual
  //  (Shape preferred if possible)

  switch (modelType) {
  case CollisionModel::OBBModel: return getCollisionModelFromVisual(modelType); break;
  case CollisionModel::GJKModel: return ref<CollisionModel>(NewObj GJKCollisionModel(ref<const Shape>(this))); break;
  case CollisionModel::ODEModel: return ref<CollisionModel>(NewObj ODECollisionModel(ref<const Shape>(this))); break;
  default:
    throw std::invalid_argument(Exception("Collision modelType not supported"));
  }

}


base::ref<CollisionModel> Shape::getCollisionModelFromVisual(CollisionModel::CollisionModelType modelType) const
{  
  if ((modelType == CollisionModel::GJKModel) 
      || (modelType == CollisionModel::AnyModel)) {

    Int LODChild = 0;
    switch (CollisionModel::getCollisionModelFidelity()) {
    case CollisionModel::Normal:   LODChild = 1; break;
    case CollisionModel::Fast:     LODChild = 3; break;
    case CollisionModel::Fastest:  LODChild = consts::maxInt; break;
    case CollisionModel::Accurate: LODChild = 0; break;
    default: ;
    }

    return ref<CollisionModel>(NewNamedObj("GJKCollisionModel") GJKCollisionModel(VisualTriangles(*this,LODChild)));
  }
  else
    throw std::invalid_argument(Exception("CollisionModelType not supported"));
}




osg::Node* Shape::createOSGAxes(const base::Dimension3& dim)
{
  const Real s = 1.5;
  Real d = Math::minimum(0.06,Math::minimum(dim.x,dim.y,dim.z)/16.0);

  Group* g = NewObj Group;
  g->setName("debug");

  // color the axes X:red, Y:green and Z:blue, with white end cones
  StateSet* red = NewObj StateSet();
  osg::Material* rmat = NewObj osg::Material();
  Vec4 cred(1,0,0,1);
  //  mat->setEmission( osg::Material::FRONT_AND_BACK, Vec4(0,0,0,0) );
  //mat->setAmbient( osg::Material::FRONT_AND_BACK, col );
  rmat->setDiffuse( osg::Material::FRONT_AND_BACK, cred );
  rmat->setSpecular( osg::Material::FRONT_AND_BACK, Vec4(1,1,1,0) );
  rmat->setShininess( osg::Material::FRONT_AND_BACK, 1.0);
  red->setAttribute( rmat );

  StateSet* green = NewObj StateSet();
  osg::Material* gmat = NewObj osg::Material();
  Vec4 cgreen(0,1,0,1);
  //  mat->setEmission( osg::Material::FRONT_AND_BACK, Vec4(0,0,0,0) );
  //mat->setAmbient( osg::Material::FRONT_AND_BACK, col );
  gmat->setDiffuse( osg::Material::FRONT_AND_BACK, cgreen );
  gmat->setSpecular( osg::Material::FRONT_AND_BACK, Vec4(1,1,1,0) );
  gmat->setShininess( osg::Material::FRONT_AND_BACK, 1.0);
  green->setAttribute( gmat );
  
  StateSet* blue = NewObj StateSet();
  osg::Material* bmat = NewObj osg::Material();
  Vec4 cblue(0,0,1,1);
  //  mat->setEmission( osg::Material::FRONT_AND_BACK, Vec4(0,0,0,0) );
  //mat->setAmbient( osg::Material::FRONT_AND_BACK, col );
  bmat->setDiffuse( osg::Material::FRONT_AND_BACK, cblue );
  bmat->setSpecular( osg::Material::FRONT_AND_BACK, Vec4(1,1,1,0) );
  bmat->setShininess( osg::Material::FRONT_AND_BACK, 1.0);
  blue->setAttribute( bmat );

  StateSet* white = NewObj StateSet();
  osg::Material* wmat = NewObj osg::Material();
  Vec4 cwhite(1,1,1,1);
  //  mat->setEmission( osg::Material::FRONT_AND_BACK, Vec4(0,0,0,0) );
  //mat->setAmbient( osg::Material::FRONT_AND_BACK, col );
  wmat->setDiffuse( osg::Material::FRONT_AND_BACK, cwhite );
  wmat->setSpecular( osg::Material::FRONT_AND_BACK, Vec4(1,1,1,0) );
  wmat->setShininess( osg::Material::FRONT_AND_BACK, 1.0);
  white->setAttribute( wmat );


  // a long Clyinder for the axis and a cone-like cylinder 
  //  for the arrow head of each X,Y and Z.

  MatrixTransform* xrot = NewObj MatrixTransform();
  xrot->setMatrix(osg::Matrix::rotate(consts::Pi/2.0,Vec3(0,1,0)));
  xrot->postMult(osg::Matrix::translate(s*dim.x/4.0,0,0));
  g->addChild(xrot);
		
  MatrixTransform* yrot = NewObj MatrixTransform();
  yrot->setMatrix(osg::Matrix::rotate(consts::Pi/2.0,Vec3(-1,0,0)));
  yrot->postMult(osg::Matrix::translate(0,s*dim.y/4.0,0));
  g->addChild(yrot);
		
  MatrixTransform* zrot = NewObj MatrixTransform();
  zrot->setMatrix(osg::Matrix::translate(0,0,s*dim.z/4.0));
  g->addChild(zrot);
		
  // the cylinder axes
  ref<Cylinder> xc(NewObj Cylinder(s*dim.x/2.0,d));
  xrot->addChild(xc->createOSGVisual());
  xrot->setStateSet(red);

  ref<Cylinder> yc(NewObj Cylinder(s*dim.y/2.0,d));
  yrot->addChild(yc->createOSGVisual());
  yrot->setStateSet(green);

  ref<Cylinder> zc(NewObj Cylinder(s*dim.z/2.0,d));
  zrot->addChild(zc->createOSGVisual());
  zrot->setStateSet(blue);

  // Translate each axis cone to the end
  MatrixTransform* xtrans = NewObj MatrixTransform();
  xtrans->setMatrix(osg::Matrix::translate(0,0,s*dim.x/4.0+d));
  xrot->addChild(xtrans);
  
  MatrixTransform* ytrans = NewObj MatrixTransform();
  ytrans->setMatrix(osg::Matrix::translate(0,0,s*dim.y/4.0+d));
  yrot->addChild(ytrans);
  
  MatrixTransform* ztrans = NewObj MatrixTransform();
  ztrans->setMatrix(osg::Matrix::translate(0,0,s*dim.z/4.0+d));
  zrot->addChild(ztrans);
  
  // the end cones
  ref<Cone> cone(NewObj Cone(4*d,2*d));
  osg::Node* coneNode = cone->createOSGVisual();
  coneNode->setStateSet(white);
  
  xtrans->addChild(coneNode);
  ytrans->addChild(coneNode);
  ztrans->addChild(coneNode);

  return g;
}


ref<Shape> Shape::createShapeFromInput(Externalizer& e, String format, Real version)
{
  if (format=="") format="xml";

  if ((format != "xml") || (version!=1.0))  
    throw std::invalid_argument(Exception(String("format ")+format+" v"+base::realToString(version)+" unsupported"));

  DOMNode* context = e.context();

  DOMElement* shapeElem = e.getFirstChildElement(context, "box", false);
  if (shapeElem) { // its a Box
    ref<Box> box(NewObj Box());
    box->externalize(e, format, version);
    e.removeElement(shapeElem);
    return box;
  }

  shapeElem = e.getFirstChildElement(context, "sphere", false);
  if (shapeElem) { // its a Sphere
    ref<Sphere> sphere(NewObj Sphere());
    sphere->externalize(e, format, version);
    e.removeElement(shapeElem);
    return sphere;
  }
  
  shapeElem = e.getFirstChildElement(context, "capsule", false);
  if (shapeElem) { // its a Capsule
    ref<Capsule> capsule(NewObj Capsule());
    capsule->externalize(e, format, version);
    e.removeElement(shapeElem);
    return capsule;
  }
  
  shapeElem = e.getFirstChildElement(context, "cylinder", false);
  if (shapeElem) { // its a Cylinder
    ref<Cylinder> cylinder(NewObj Cylinder());
    cylinder->externalize(e, format, version);
    e.removeElement(shapeElem);
    return cylinder;
  }
  
  shapeElem = e.getFirstChildElement(context, "cone", false);
  if (shapeElem) { // its a Cone
    ref<Cone> cone(NewObj Cone());
    cone->externalize(e, format, version);
    e.removeElement(shapeElem);
    return cone;
  }
  
  shapeElem = e.getFirstChildElement(context, "polyhedron", false);
  if (shapeElem) { // its a Polyhedron
    ref<Polyhedron> polyhedron(NewObj Polyhedron());
    polyhedron->externalize(e, format, version);
    e.removeElement(shapeElem);
    return polyhedron;
  }
  
  throw base::externalization_error(Exception(String("unknown shape Element - must be one of Box, Sphere, Cylinder, Capsule, Cone or Polyhedron")));
}

