/****************************************************************************
  Copyright (C)2004 David Jung <opensim@pobox.com>

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
  
  $Id: SimulatedManipulatorDescription.cpp 1033 2004-02-11 20:47:52Z jungd $
  $Revision: 1.1 $
  $Date: 2004-02-11 15:47:52 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <robot/sim/SimulatedManipulatorDescription>

#include <base/Application>
#include <base/Externalizer>
#include <base/externalization_error>

using robot::sim::SimulatedManipulatorDescription;

using base::Externalizer;
using base::externalization_error;
using base::Application;
using base::PathName;
using base::VFile;
using base::dom::DOMNode;
using base::dom::DOMElement;


/// \todo output of XML init config, geom & prox info
void SimulatedManipulatorDescription::externalize(base::Externalizer& e, String format, Real version)
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
    
    // handle any simulation specific stuff present
    
    // read in initial configuration
    DOMElement* configElem = e.getFirstChildElement(manipElem, "initialconfig",false);
    bool initialConfigSpecified = (configElem != 0);
    if (initialConfigSpecified) {
      String configText = e.getContainedText(configElem);
      q = e.toVector(configText);
      for(Int i=0; i<q.size(); i++) // convert from Deg to Rad
        q[i] = Math::degToRad(q[i]);
    }
    
    if (!initialConfigSpecified) {
      q = zeroVector(kinematicChain.dof());
    }
    else
      if (q.size() != kinematicChain.dof())
        throw new externalization_error(Exception("initialconfig element has vector of wrong dimension ("+base::intToString(q.size())
                                           +"); must be equal to manipulator dof ("+base::intToString(kinematicChain.dof())+")"));
    
    // read in link geometry (if any)
    DOMElement* geomElem = e.getFirstChildElement(manipElem, "geometry",false);
    linkGeometrySpecified = (geomElem != 0);
    if (linkGeometrySpecified) {
      // !!! for now just assume a vector of link radii
      String geomText = e.getContainedText(geomElem);
      Vector r( e.toVector(geomText) );
      linkRadii.resize(r.size());
      for(Int i=0; i<r.size(); i++) linkRadii[i] = r[i];
    }
    
    
    if (linkGeometrySpecified)
      if (linkRadii.size() != kinematicChain.size()+1)
        throw new externalization_error(Exception("geometry element has wrong number of link radii ("+base::intToString(linkRadii.size())
                                           +"); must be equal to number of links + the base ("+base::intToString(kinematicChain.size()+1)+")"));
    

    // read in proximity sensor flag
    DOMElement* proxElem = e.getFirstChildElement(manipElem, "proximitysensors",false);
    hasProxSensors = (proxElem != 0);
    if (hasProxSensors) {
      String rangeText = e.getElementAttribute(proxElem, "range", true);
      proxSensorRange = base::stringToReal(rangeText);
    }

    
    e.removeElement(manipElem);

  }
}

