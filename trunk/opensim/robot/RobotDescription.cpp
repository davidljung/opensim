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
  
  $Id: RobotDescription.cpp 1097 2004-09-27 21:49:38Z jungd $
  $Revision: 1.4 $
  $Date: 2004-09-27 17:49:38 -0400 (Mon, 27 Sep 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <robot/RobotDescription>

#include <base/Externalizer>
#include <base/Application>
#include <base/VFile>

using robot::RobotDescription;

using base::Externalizer;
using base::Application;
using base::VFile;
using base::PathName;
using base::dom::DOMNode;
using base::dom::DOMElement;
using robot::KinematicChain;


KinematicChain RobotDescription::getKinematicChain(Int platformDOF, const base::Matrix4& platformTransform,
                                                   Int manipIndex, Int manipChainIndex) const
{
  // construct a chain from platform, manipulator offset, manipulator base transform and manipulator chain
  KinematicChain chain( platform()->getKinematicChain(platformDOF, platformTransform) );

  if (manipulators().size() > 0) {
    if (manipIndex < manipulators().size()) {
      Matrix4 offsetTranslation; offsetTranslation.setToTranslation( manipulatorOffsets()[manipIndex] );
      chain += KinematicChain::Link(offsetTranslation); // manip mount offset from platform (fixed - 0-dof)
      chain += KinematicChain::Link( manipulators()[manipIndex]->getBaseTransform() ); // mount to base frame (fixed - 0-dof)
      chain += manipulators()[manipIndex]->getKinematicChain(manipChainIndex);
    }
    else
      throw std::out_of_range(Exception("manipIndex is out of range"));
  }
  return chain;
}
 
 


void RobotDescription::externalize(base::Externalizer& e, String format, Real version)
{
  if (format == "") format = "xml";

  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" v"+base::realToString(version)+" unsupported"));

  if (e.isOutput()) {

    DOMElement* robotElem = e.createElement("robot");
    e.setElementAttribute(robotElem,"name",getName());

    e.appendComment(robotElem,"description of a robot (with a [mobile] platform and one or more manipulators");
    
    e.pushContext(robotElem);
    platformDescr->externalize(e, format, version);

    for(Int i=0; i<manipulatorDescrs.size(); i++) {
      manipulatorDescrs[i]->externalize(e, format, version);

      DOMElement* manipElem = e.lastAppendedElement();
      // manipulator offset
      DOMElement* offsetElem = e.createElement("offset",false);
      e.appendText(offsetElem, e.toString(manipOffsets[i]));
      e.appendComment(manipElem,"offset relative to platform");
      e.appendNode(manipElem, offsetElem);
      e.appendBreak(manipElem);
    }

    e.popContext();

    e.appendElement(robotElem);

  } else { // input

    DOMNode* context = e.context();
    
    DOMElement* robotElem = e.getFirstElement(context, "robot");

    // handle link
    String link = e.getElementAttribute(robotElem,"link",false);
    if (link != "") {

      ref<VFile> linkFile = Application::getInstance()->universe()->cache()->findFile(link,e.getArchivePath());
      load(linkFile,format,version);
    }
    else {
//!!! does this allow for a link'd robot to have it's name overridden by the linked from element?
      setName( e.getDefaultedElementAttribute(robotElem, "name", "robot") );
      
      e.pushContext(robotElem);
      ref<PlatformDescription> pd( newPlatformDescription() );
      pd->externalize(e, format, version);
      platformDescr = pd;
      
      // get manipulators
      
      DOMElement* manipElem = e.getFirstChildElement(robotElem, "manipulator",false);
      while (manipElem) {
        // get offset
        DOMElement* offsetElem = e.getFirstChildElement(manipElem,"offset");
        String offsetText = e.getContainedText(offsetElem);
        manipOffsets.push_back( Vector3( e.toVector3(offsetText) ) );
        
        // externalize manipulator
        ref<ManipulatorDescription> manipulatorDescr( newManipulatorDescription() );
        manipulatorDescr->externalize(e, format, version);
        manipulatorDescrs.push_back( manipulatorDescr );
        
        manipElem = e.getFirstChildElement(robotElem, "manipulator", false);
      }
      
      e.popContext();
    }

    e.removeElement(robotElem);

  }
}
