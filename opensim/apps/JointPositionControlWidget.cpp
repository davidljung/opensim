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
  
  $Id: JointPositionControlWidget.cpp 1059 2004-02-27 19:28:44Z jungd $
  $Revision: 1.1 $
  $Date: 2004-02-27 14:28:44 -0500 (Fri, 27 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <apps/JointPositionControlWidget>

#include <base/Math>
#include <base/Time>

#include <robot/Robot>
#include <robot/RobotDescription>


using apps::JointPositionControlWidget;

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


JointPositionControlWidget::JointPositionControlWidget(GUIControlWindow& parentWindow) 
  : ControlInterfaceWidget(parentWindow), posInterface(0)
{
}

Gtk::HScale* gscale;


void JointPositionControlWidget::init(ref<BasicEnvironment> env, ref<Robot> robot, Int index, ref<ControlInterface> interface)
{
  Assert(interface->getType() == "JointPositionControl");
  
  posInterface = interface; 

  // Joint position controls
  Gtk::VBox* jointControlBox = new Gtk::VBox(false,5);

  Int dof =  posInterface->inputSize();
  posAdjustments.clear();
  for(Int j=0; j<dof; j++) {
    Gtk::HBox* hbox = new Gtk::HBox();
    Gtk::Label* label = new Gtk::Label(base::intToString(j)+":");
    hbox->pack_start(*manage(label), Gtk::PACK_SHRINK,5);
    Gtk::Adjustment* adj = new Gtk::Adjustment(0,-159,159,0.5,5);
    posAdjustments.push_back(adj);
    Gtk::HScale* scale = new Gtk::HScale(*manage(adj));
    scale->set_draw_value(true);
    scale->set_digits(1);
    scale->set_value_pos(Gtk::POS_LEFT);
    scale->set_update_policy(Gtk::UPDATE_CONTINUOUS);
    scale->set_size_request(100,-1);
    posScales.push_back(scale);
    hbox->pack_end(*manage(scale));
    jointControlBox->pack_start(*manage(hbox),false,false);
    adj->signal_value_changed().connect( SigC::bind<Int>( SigC::slot(*this, &JointPositionControlWidget::jointPositionScaleChanged ), j) );
  }
  
  pack_start(*manage(jointControlBox));
}


void JointPositionControlWidget::jointPositionScaleChanged(Int j)
{
  if (posInterface == 0) return;
  posInterface->setOutput(j, Math::degToRad( posAdjustments[j]->get_value() ) ); // assumes revolute!!! (i.e. angle)
}


void JointPositionControlWidget::iterate(const base::Time& simTime)
{
  if (!(lastSimTime < simTime)) return;
  
  for(Int j=0; j<posInterface->inputSize(); j++)
    if (!posScales[j]->has_grab()) // only update scale if it's not currently being dragged
      posAdjustments[j]->set_value( Math::radToDeg(posInterface->getInput(j)) ); // assumes revolute!!! (i.e. angle)
  
  lastSimTime = simTime;
}

