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
  
  $Id: GenericControlInterfaceWidget.cpp 1059 2004-02-27 19:28:44Z jungd $
  $Revision: 1.1 $
  $Date: 2004-02-27 14:28:44 -0500 (Fri, 27 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <apps/GenericControlInterfaceWidget>

#include <base/Math>
#include <base/Time>

#include <robot/Robot>
#include <robot/RobotDescription>


using apps::GenericControlInterfaceWidget;

using base::Math;
using base::Time;
using base::Orient;


using robot::Robot;
using robot::ControlInterface;
using robot::sim::BasicEnvironment;




// GTKmm
#include <gtkmm.h>

using Gtk::manage;


GenericControlInterfaceWidget::GenericControlInterfaceWidget(GUIControlWindow& parentWindow) 
  : ControlInterfaceWidget(parentWindow), interface(0)
{
}



void GenericControlInterfaceWidget::init(ref<BasicEnvironment> env, ref<Robot> robot, Int index, ref<ControlInterface> interface)
{
  this->interface = interface; 

  // Outputs
  Gtk::Frame* outputFrame = new Gtk::Frame("Outputs");
  Gtk::VBox* outputBox = new Gtk::VBox(false,5);
  outputFrame->add(*manage(outputBox));

  Int outputSize = interface->outputSize();
  adjustments.clear();
  for(Int i=0; i<outputSize; i++) {
    Gtk::HBox* hbox = new Gtk::HBox();
    Gtk::Label* label = new Gtk::Label(interface->outputName(i)+":");
    hbox->pack_start(*manage(label), Gtk::PACK_SHRINK,5);
    Gtk::Adjustment* adj = new Gtk::Adjustment(0,-50,50,0.1,2);
    adjustments.push_back(adj);
    Gtk::HScale* scale = new Gtk::HScale(*manage(adj));
    scale->set_draw_value(true);
    scale->set_digits(1);
    scale->set_value_pos(Gtk::POS_LEFT);
    scale->set_update_policy(Gtk::UPDATE_CONTINUOUS);
    scale->set_size_request(120,-1);
    hbox->pack_end(*manage(scale));
    outputBox->pack_start(*manage(hbox),false,false);
    adj->signal_value_changed().connect( SigC::bind<Int>( SigC::slot(*this, &GenericControlInterfaceWidget::outputChanged ), i) );
  }
    
  pack_start(*manage(outputFrame));

  
  // Inputs
  Gtk::Frame* inputFrame = new Gtk::Frame("Inputs");
  Gtk::VBox* inputBox = new Gtk::VBox(false,5);
  inputFrame->add(*manage(inputBox));

  Int inputSize = interface->inputSize();
  for(Int i=0; i<inputSize; i++) {
    Gtk::HBox* hbox = new Gtk::HBox();
    Gtk::Label* label = new Gtk::Label(interface->inputName(i)+":");
    hbox->pack_start(*manage(label), Gtk::PACK_SHRINK,5);
    Gtk::Label* inputLabel = new Gtk::Label();
    inputLabels.push_back(inputLabel);
    hbox->pack_end(*manage(inputLabel));
    inputBox->pack_start(*manage(hbox),false,false);
  }
  
  pack_start(*manage(inputFrame));
  
}


void GenericControlInterfaceWidget::outputChanged(Int i)
{
  if (interface != 0)
    interface->setOutput(i, adjustments[i]->get_value() );
}


void GenericControlInterfaceWidget::iterate(const base::Time& simTime)
{
  if (!(lastSimTime < simTime)) return;
  
  for(Int i=0; i<interface->inputSize(); i++) {
    Real value = interface->getInput(i);
    value = Real(Int(value*1000.0))/1000.0;
    inputLabels[i]->set_text( base::realToString(value) );
  }
  
  lastSimTime = simTime;
}

