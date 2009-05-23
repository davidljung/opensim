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

  $Id: Material.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.6 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
  
****************************************************************************/

#include <physics/Material>

#include <base/Application>
#include <base/VFile>
#include <base/Externalizer>
#include <base/externalization_error>
#include <base/Vector>
#include <gfx/Color3>
#include <gfx/Color4>

#include <osg/Material>
#include <osg/Texture2D>
#include <osg/Vec4>
#include <osgDB/ReadFile>


using base::Application;
using base::VFile;
using base::Vector;
using base::externalization_error;

using base::dom::DOMNode;
using base::dom::DOMElement;

using physics::Material;
using gfx::Color3;
using gfx::Color4;

using osg::Vec4;
using osg::StateSet;


Material::Material(const String& label, const gfx::Color4& color)
	: label(label), mDensity(1.0), 
	  surfaceAppearanceType(BaseColor),
	  mBaseColor(color),
	  stateSetCached(false)
{
}


Material::Material(const Material& m)
  : label(m.label), mDensity(m.mDensity), 
    surfaceAppearanceType(m.surfaceAppearanceType),
    mBaseColor(m.mBaseColor),
    surfaceTextureImage(m.surfaceTextureImage),
    stateSetCached(m.stateSetCached),
    stateSet(m.stateSet)
{
}


Material::~Material()
{
}


void Material::setSurfaceAppearance(const base::PathName& image)
{
  surfaceAppearanceType = TextureImage;
  surfaceTextureImage = image;
  stateSetCached = false;
}


osg::ref_ptr<osg::StateSet> Material::createState() const
{
  if (stateSetCached) return stateSet;

  osg::ref_ptr<StateSet> state = new osg::StateSet();
  osg::ref_ptr<osg::Material> mat = new osg::Material();
  Vec4 col( (float)baseColor().r, 
	    (float)baseColor().g, 
	    (float)baseColor().b, 
            (float)baseColor().a );
  mat->setEmission( osg::Material::FRONT_AND_BACK, Vec4(0,0,0,0) );
  mat->setAmbient( osg::Material::FRONT_AND_BACK, col );
  mat->setDiffuse( osg::Material::FRONT_AND_BACK, col );
  mat->setSpecular( osg::Material::FRONT_AND_BACK, Vec4(1,1,1,0) );
  mat->setShininess( osg::Material::FRONT_AND_BACK, 128);
  mat->setAlpha( osg::Material::FRONT_AND_BACK, (float)baseColor().a );
  state->setAttribute( &(*mat) );
  if (!Math::equals(baseColor().a, 1.0)) {
    state->setMode(GL_BLEND, osg::StateAttribute::ON);
    state->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
  }

  if (surfaceAppearanceType == TextureImage) {

    ref<VFile> imageFile(Application::getInstance()->universe()->cache()->findFile(surfaceTextureImage));

    // load image
    ///!!! unfortunately, the osgDB only reads from filesystem files, not
    ///    streams, so we only support files here for now
    String filename( imageFile->pathName().str() );

    osg::ref_ptr<osg::Image> image = osgDB::readImageFile(filename);
    if (image == (osg::Image*)0)
      throw std::runtime_error(Exception(String("unable to read texture image file:")+filename));

    osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D();
    texture->setImage(&(*image));
    texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_NEAREST);
    texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
    texture->setInternalFormatMode(osg::Texture2D::USE_ARB_COMPRESSION);
    texture->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::REPEAT);
    texture->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::REPEAT);

    state->setTextureAttributeAndModes(0,&(*texture),osg::StateAttribute::ON);
  }

  stateSetCached = true;
  stateSet = state;

  return stateSet;
}


void Material::externalize(base::Externalizer& e, String format, Real version)
{
  if (format=="") format="xml";

  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" v"+base::realToString(version)+" unsupported"));
  
  if (format == "xml") {

    if (e.isOutput()) {

      DOMElement* matElem = e.createElement("material");
      e.setElementAttribute(matElem,"name",label);
      e.setElementAttribute(matElem,"type","simple");
      
      DOMElement* dElem = e.createElement("density",false);
      e.appendText(dElem, base::realToString(mDensity));
      e.appendNode(matElem,dElem);
      e.appendBreak(matElem);

      DOMElement* cElem = e.createElement("basecolor",false);
      Vector c(3);
      c[0]=mBaseColor.r; c[1]=mBaseColor.g; c[2]=mBaseColor.b;
      e.appendText(cElem, e.toString(c,true));
      e.appendNode(matElem,cElem);
      
      if (surfaceAppearanceType == TextureImage) {
	DOMElement* saElem = e.createElement("surfaceappearance",false);
	e.setElementAttribute(saElem,"type","image");
	e.appendText(saElem, surfaceTextureImage.str() );
	e.appendNode(matElem,saElem);
      }

      e.appendElement(matElem);

    }
    else { // input

      DOMNode* context = e.context();
    
      DOMElement* matElem = e.getFirstChildElement(context, "material");

      if ( e.getElementAttribute(matElem, "type") != "simple" )
	throw externalization_error(Exception("unsupported material type"));

      label = e.getDefaultedElementAttribute(matElem, "name", "untitled");

      DOMElement* dElem = e.getFirstChildElement(matElem, "density", false);
      if (dElem) {
	String densityText = e.getContainedText(dElem);
	mDensity = base::stringToReal(densityText);
      }
      else
	mDensity=1.0;

      surfaceAppearanceType = BaseColor;
      DOMElement* cElem = e.getFirstChildElement(matElem, "basecolor",false);
      if (cElem) {
	String colorText = e.getContainedText(cElem);
	Vector3 cv( e.toVector3(colorText) );
	mBaseColor = Color3(cv.x,cv.y,cv.z);
      }
      else
	mBaseColor = Color3(0,1,0);


      DOMElement* saElem = e.getFirstChildElement(matElem, "surfaceappearance",false);
      if (saElem) {
	String type = e.getDefaultedElementAttribute(saElem,"type","image");
	if (type == "image") {
	  surfaceAppearanceType = TextureImage;
	  surfaceTextureImage = e.getContainedText(saElem);
	}
	else
	  throw externalization_error(Exception("unsupported value of 'type' attribute of element 'surfaceappearance'"));
      }



      e.removeElement(matElem);

    }

  }

}

