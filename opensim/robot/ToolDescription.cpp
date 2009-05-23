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

  $Id: ToolDescription.cpp 1099 2004-09-27 21:51:37Z jungd $
  $Revision: 1.2 $
  $Date: 2004-09-27 17:51:37 -0400 (Mon, 27 Sep 2004) $
  $Author: jungd $

****************************************************************************/

#include <robot/ToolDescription>

#include <base/Externalizer>
#include <base/externalization_error>
#include <base/Application>
#include <base/VFile>

using robot::ToolDescription;

using base::Externalizer;
using base::externalization_error;
using base::Application;
using base::VFile;
using base::PathName;
using base::dom::DOMNode;
using base::dom::DOMElement;



void ToolDescription::outputElementXML(base::Externalizer& e, DOMElement* toolElem, String format, Real version) const
{
  e.setElementAttribute(toolElem,"name", getName());

  e.pushContext(toolElem);
  kinematicChain.externalize(e, format, version);
  e.popContext();
}


void ToolDescription::inputElementXML(base::Externalizer& e, DOMElement* toolElem, String format, Real version)
{
  setName( e.getDefaultedElementAttribute(toolElem, "name", "tool") );

  e.pushContext(toolElem);
  kinematicChain.externalize(e, format, version);
  e.popContext();

  // the first link of a tool is always considered inactive/static (i.e. it's mount point
  //  on an existing manipulator is not a joint)
  kinematicChain.activateLink(0,false);
}


void ToolDescription::externalize(base::Externalizer& e, String format, Real version)
{
  if (format == "") format = "xml";

  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" v"+base::realToString(version)+" unsupported"));

  if (e.isOutput()) {

    DOMElement*  toolElem = e.createElement("tool");
    outputElementXML(e, toolElem, format, version);
    e.appendElement(toolElem);

  } else { // input

    DOMNode* context = e.context();

    DOMElement* toolElem = e.getFirstElement(context, "tool");

    // handle link
    String link = e.getElementAttribute(toolElem,"link",false);
    if (link != "") {

      ref<VFile> linkFile = Application::getInstance()->universe()->cache()->findFile(link,e.getArchivePath());
      load(linkFile,format,version);
    }
    else {
      inputElementXML(e, toolElem, format, version);
    }

    e.removeElement(toolElem);

  }
}
