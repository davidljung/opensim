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
  
  $Id: LinkProximitySensorsWidget.cpp 1059 2004-02-27 19:28:44Z jungd $
  $Revision: 1.1 $
  $Date: 2004-02-27 14:28:44 -0500 (Fri, 27 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <apps/LinkProximitySensorsWidget>

#include <base/Math>
#include <base/Time>

#include <robot/Robot>
#include <robot/RobotDescription>


using apps::LinkProximitySensorsWidget;

using base::Math;
using base::Time;
using base::Orient;


using robot::Robot;
using robot::RobotDescription;
using robot::PlatformDescription;
using robot::ManipulatorDescription;
using robot::ControlInterface;
using robot::sim::BasicEnvironment;




// GTKmm
#include <gtkmm.h>

using Gtk::manage;


LinkProximitySensorsWidget::LinkProximitySensorsWidget(GUIControlWindow& parentWindow) 
  : ControlInterfaceWidget(parentWindow), proxInterface(0)
{
}



void LinkProximitySensorsWidget::init(ref<BasicEnvironment> env, ref<Robot> robot, Int index, ref<ControlInterface> interface)
{
  Assert(interface->getType() == "LinkProximitySensors");
  
  proxInterface = interface; 

  // box for sensor info for each link
  Gtk::VBox* proxBox = new Gtk::VBox(false,5);

  proxBox->pack_start(*manage(new Gtk::Label("Proximity sensor range")));
  
  Int numSensors =  proxInterface->inputSize() / 5;
  for(Int s=0; s<numSensors; s++) {
    Gtk::HBox* hbox = new Gtk::HBox();
    Gtk::Label* label = new Gtk::Label(base::intToString(s)+":");
    hbox->pack_start(*manage(label), Gtk::PACK_SHRINK,5);
    hbox->pack_start(*manage(new Gtk::Label("Range:")), Gtk::PACK_SHRINK,5);
    Gtk::Label* rangeLabel = new Gtk::Label("");
    proxLabel.push_back(rangeLabel);
    hbox->pack_start(*manage(rangeLabel), Gtk::PACK_SHRINK,5);
    proxBox->pack_start(*manage(hbox), Gtk::PACK_SHRINK,2);
  }
  
  pack_start(*manage(proxBox), Gtk::PACK_SHRINK,5);
}




void LinkProximitySensorsWidget::iterate(const base::Time& simTime)
{
  if (!(lastSimTime < simTime)) return;
  
  for(Int s=0; s<proxInterface->inputSize()/5; s++) {
    Real range = proxInterface->getInput( 5*s + 0 );
    if (range < consts::maxInt)
      range = Real(Int(range*1000.0))/1000.0;
    String rangeString = (range<consts::maxInt)?(base::realToString(range) + "m"):("");
    proxLabel[s]->set_text(rangeString);
  }
  
  lastSimTime = simTime;
}

