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
  
  $Id: JointPosAndIKControlWidget.cpp 1059 2004-02-27 19:28:44Z jungd $
  $Revision: 1.1 $
  $Date: 2004-02-27 14:28:44 -0500 (Fri, 27 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <apps/JointPosAndIKControlWidget>

#include <base/Math>
#include <base/Time>

#include <robot/Robot>
#include <robot/RobotDescription>


using apps::JointPosAndIKControlWidget;

using base::Math;
using base::Time;
using base::Orient;


using robot::Robot;
using robot::RobotDescription;
using robot::PlatformDescription;
using robot::ManipulatorDescription;
using robot::ControlInterface;
using robot::sim::BasicEnvironment;
using apps::JointPositionControlWidget;
using apps::InverseKinematicsControlWidget;




// GTKmm
#include <gtkmm.h>

using Gtk::manage;


JointPosAndIKControlWidget::JointPosAndIKControlWidget(GUIControlWindow& parentWindow) 
  : ControlInterfaceWidget(parentWindow), 
    eeControl(false),
    jointPositionControlWidget(parentWindow), jointPosVelControllerWidget(parentWindow),
    inverseKinematicsControlWidget(parentWindow)
{
}


void JointPosAndIKControlWidget::init(ref<BasicEnvironment> env, ref<Robot> robot, Int index, ref<ControlInterface> interface)
{

  notebook.set_border_width(5);

  posInterfaceSupplied = (interface->getType() == "JointPositionControl");
  if (!posInterfaceSupplied)
    Assert(interface->getType() == "JointVelocityControl");
  
  // Joint position controls
  if (posInterfaceSupplied) {
    jointPositionControlWidget.init(env, robot, index, interface);
    notebook.append_page(jointPositionControlWidget, "Joints");
    posInterface = interface;
  }
  else {
    jointPosVelControllerWidget.init(env, robot, index, interface);
    notebook.append_page(jointPosVelControllerWidget, "Joints");
    posInterface = jointPosVelControllerWidget.getPositionControlInterface();
  }
  
  // end-effector position controls
  inverseKinematicsControlWidget.init(env, robot, index, posInterface);
  
  notebook.append_page(inverseKinematicsControlWidget,"End-Effector");
  notebook.signal_switch_page().connect( SigC::slot(*this, &JointPosAndIKControlWidget::tabChanged) );
  
  pack_start(notebook);
}


void JointPosAndIKControlWidget::finalize()
{
  inverseKinematicsControlWidget.finalize();
  if (posInterfaceSupplied)
    jointPositionControlWidget.finalize();
  else
    jointPosVelControllerWidget.finalize();
}  


void JointPosAndIKControlWidget::tabChanged(GtkNotebookPage* page, guint page_num)
{
  eeControl = (page_num==1);
}


void JointPosAndIKControlWidget::iterate(const base::Time& simTime)
{
  if (!(lastSimTime < simTime)) return;

  if (eeControl) {
    inverseKinematicsControlWidget.iterate(simTime);
    if (!posInterfaceSupplied)
      jointPosVelControllerWidget.getController()->iterate(simTime);
  }
  else {
    if (posInterfaceSupplied)
      jointPositionControlWidget.iterate(simTime);
    else
      jointPosVelControllerWidget.iterate(simTime);
  }
  
  lastSimTime = simTime;
}

