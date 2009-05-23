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
  
  $Id: JointPosVelControllerWidget.cpp 1059 2004-02-27 19:28:44Z jungd $
  $Revision: 1.1 $
  $Date: 2004-02-27 14:28:44 -0500 (Fri, 27 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <apps/JointPosVelControllerWidget>

#include <base/Math>
#include <base/Time>

#include <robot/Robot>


using apps::JointPosVelControllerWidget;

using base::Math;
using base::Time;
using base::Orient;


using robot::Robot;
using robot::ControlInterface;
using robot::RobotDescription;
using robot::ManipulatorDescription;
using robot::control::ManipulatorPIDPositionController;
using robot::sim::BasicEnvironment;

using apps::JointPositionControlWidget;



// GTKmm
#include <gtkmm.h>

using Gtk::manage;


JointPosVelControllerWidget::JointPosVelControllerWidget(GUIControlWindow& parentWindow)
  :  ControlInterfaceWidget(parentWindow), jointPositionControlWidget(parentWindow)
{
}


void JointPosVelControllerWidget::init(ref<BasicEnvironment> env, ref<Robot> robot, Int index, ref<ControlInterface> interface)
{
  velInterface = interface; 

  // end-effector position controls
  Gtk::VBox* controlBox = new Gtk::VBox(false,5);

  // adapt JointVelocityControl to JointPositionControl via controller
  if (!robot->isDescriptionProvided()) {
    controlBox->pack_start(*manage(new Gtk::Label("No robot description\navailable - can't control")));
    pack_start(*manage(controlBox));
    return;
  }
  ref<const RobotDescription> rd( robot->getRobotDescription() );
  ref<const ManipulatorDescription> md( rd->manipulators()[index] );
  controller = ref<ManipulatorPIDPositionController>(NewObj ManipulatorPIDPositionController(md->getKinematicChain()));
  
  Kp = new Gtk::Adjustment(12,0,100,0.1,1);
  Kp->signal_value_changed().connect( SigC::slot(*this, &JointPosVelControllerWidget::pidParamChanged ) );
  Ki = new Gtk::Adjustment(0,0,10,0.01,0.1);
  Ki->signal_value_changed().connect( SigC::slot(*this, &JointPosVelControllerWidget::pidParamChanged ) );
  Kd = new Gtk::Adjustment(2,0,20,0.01,0.1);
  Kd->signal_value_changed().connect( SigC::slot(*this, &JointPosVelControllerWidget::pidParamChanged ) );
  
  controller->setCoeffs( Kp->get_value() , Ki->get_value() , Kd->get_value()); // PID Kp, Ki, Kd
  controller->setControlInterface(velInterface); // connect posController to manipulator interface
  posInterface = controller->getControlInterface();

  // setup position control widget
  controlBox->pack_start(*manage(new Gtk::Label("Joint Positions:")), Gtk::PACK_SHRINK,5);
  jointPositionControlWidget.init(env, robot, index, posInterface);
  controlBox->pack_start(jointPositionControlWidget, Gtk::PACK_SHRINK,5);

  pack_start(*manage(controlBox), Gtk::PACK_SHRINK,5);
  
  
  // controller parameters
  Gtk::Frame* paramFrame = new Gtk::Frame("Controller Parameters");
  Gtk::VBox* paramBox = new Gtk::VBox(false,5);
  paramFrame->add(*manage(paramBox));
  
  Gtk::HBox* hbox = new Gtk::HBox();
  hbox->pack_start(*manage(new Gtk::Label("P")), Gtk::PACK_SHRINK,5);
  Gtk::SpinButton* spin = new Gtk::SpinButton(*manage(Kp));
  spin->set_digits(3);
  hbox->pack_start(*manage(spin));
  paramBox->pack_start(*manage(hbox), Gtk::PACK_SHRINK,5);
  
  hbox = new Gtk::HBox();
  hbox->pack_start(*manage(new Gtk::Label("I")), Gtk::PACK_SHRINK,5);
  spin = new Gtk::SpinButton(*manage(Ki));
  spin->set_digits(3);
  hbox->pack_start(*manage(spin));
  paramBox->pack_start(*manage(hbox), Gtk::PACK_SHRINK,5);
  
  hbox = new Gtk::HBox();
  hbox->pack_start(*manage(new Gtk::Label("D")), Gtk::PACK_SHRINK,5);
  spin = new Gtk::SpinButton(*manage(Kd));
  spin->set_digits(3);
  hbox->pack_start(*manage(spin));
  paramBox->pack_start(*manage(hbox), Gtk::PACK_SHRINK,5);
  
  pack_start(*manage(paramFrame));
}


void JointPosVelControllerWidget::finalize()
{
  posInterface = ref<ControlInterface>(0);
  controller = ref<ManipulatorPIDPositionController>(0);
}


void JointPosVelControllerWidget::pidParamChanged()
{
  if (controller != 0)
    controller->setCoeffs( Kp->get_value() , Ki->get_value() , Kd->get_value()); // PID Kp, Ki, Kd
}


void JointPosVelControllerWidget::iterate(const base::Time& simTime)
{
  if (!(lastSimTime < simTime)) return;
  
  controller->iterate(simTime);
  jointPositionControlWidget.iterate(simTime);
  
  lastSimTime = simTime;
}

