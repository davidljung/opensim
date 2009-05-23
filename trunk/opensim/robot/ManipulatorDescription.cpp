/****************************************************************************
  Copyright (C)2003 David Jung <opensim@pobox.com>

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
  
  $Id: ManipulatorDescription.cpp 1038 2004-02-11 20:50:44Z jungd $
  $Revision: 1.5 $
  $Date: 2004-02-11 15:50:44 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <robot/ManipulatorDescription>

#include <base/Externalizer>
#include <base/externalization_error>
#include <base/Application>
#include <base/VFile>

using robot::ManipulatorDescription;

using base::Externalizer;
using base::externalization_error;
using base::Application;
using base::VFile;
using base::PathName;
using base::dom::DOMNode;
using base::dom::DOMElement;



void ManipulatorDescription::outputElementXML(base::Externalizer& e, DOMElement* manipElem, String format, Real version) const
{
  e.setElementAttribute(manipElem,"name",getName());
  e.setElementAttribute(manipElem,"type", ((mtype==Serial)?"serial":"parallel") );

  if (mtype == Serial) {

    e.appendComment(manipElem,"matrix transform from mount point of manipulator to origin of link 0");
    
    DOMElement*  btElem = e.createElement("basetransform");
    manipElem->appendChild(btElem);
    
    e.appendText(btElem, e.toString(baseTransform) );

    e.pushContext(manipElem);
    kinematicChain.externalize(e, format, version);
    e.popContext();
    
  }
  else {
    Unimplemented;
  }

  e.appendElement(manipElem);
}




/// \todo move the baseTransform input into a helper function so that is can be called by the handle link code to override
///       (unless we remove the base completely!)
void ManipulatorDescription::inputElementXML(base::Externalizer& e, DOMElement* manipElem, String format, Real version)
{
  setName( e.getDefaultedElementAttribute(manipElem, "name", "manipulator") );
  if ( e.getDefaultedElementAttribute(manipElem, "type", "serial") == "serial" )
    mtype = Serial;
  else
    mtype = Parallel;
  
  if (mtype == Serial) {
    DOMElement* btElem = e.getFirstChildElement(manipElem, "basetransform",false);

    if (btElem) {
    
      // !!! A base transform as part of the manipulator description has been
      //     depricated.  It will be assumed to be the identity
      // !!!
      Logln("Warning: a 'basetransform' specification as part of the manipulator description has been depricated.  Use 'mounttransform' instead.");
     
     
      // if basetransform element contains a translation and/or rotation element
      //  construct transform from that, otherwise assume it contains the text
      //  for a 4x4 transformation matrix
      bool isMatrix = true;
      Vector trans(zeroVector(3));
      Orient rot;
      DOMElement* btTransElem = e.getFirstChildElement(btElem, "translation",false);
      if (btTransElem != 0) {
        isMatrix = false;
        String transText = e.getContainedText(btTransElem);
        trans = e.toVector(transText,true);
        if (trans.size() != 3)
          throw new externalization_error(Exception("basetransform translation vector must have 3 elements"));
      }
      
      DOMElement* btRotElem = e.getFirstChildElement(btElem, "rotation",false);
      if (btRotElem != 0) {
        isMatrix = false;
        String rotText = e.getContainedText(btRotElem);
        Vector ov = e.toVector(rotText,true);
        if (ov.size() == 3) // assume RPY
          rot = Orient(ov[0],ov[1],ov[2]);
        else if (ov.size() == 4) // assume quat
          rot = Orient(Quat4(ov[0],ov[1],ov[2],ov[3]));
        else
          throw new externalization_error(Exception("basetransform rotation must have 3 (for EulerRPY) or 4 (for Quat) elements"));
      }
  
      if (!isMatrix)
        baseTransform = Transform(toVector3(trans),rot).getTransform();
      else {
        String btText = e.getContainedText(btElem,true);
        baseTransform = e.toMatrix4(btText);
      }
    }
    else
      baseTransform.setIdentity(); // assume identity if omitted
    
    e.pushContext(manipElem);
    kinematicChain.externalize(e, format, version);
    e.popContext();
    
  }
}






void ManipulatorDescription::externalize(base::Externalizer& e, String format, Real version)
{
  if (format == "") format = "xml";

  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" v"+base::realToString(version)+" unsupported"));

  if (e.isOutput()) {

    DOMElement*  manipElem = e.createElement("manipulator");
    outputElementXML(e, manipElem, format, version);
    e.appendElement(manipElem);

  } else { // input
      
    DOMNode* context = e.context();
    DOMElement* manipElem = e.getFirstElement(context, "manipulator");

    // handle link
    String link = e.getElementAttribute(manipElem,"link",false);
    if (link != "") {
  
      ref<VFile> linkFile = Application::getInstance()->universe()->cache()->findFile(link,e.getArchivePath());
      load(linkFile,format,version);
    }
    else {
      inputElementXML(e,manipElem, format, version);
    }
    
    e.removeElement(manipElem);

  }
}
