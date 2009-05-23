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
  
  $Id: InverseKinematicsControlWidget.cpp 1058 2004-02-27 19:28:15Z jungd $
 
****************************************************************************/

#include <apps/InverseKinematicsControlWidget>

#include <base/Math>
#include <base/Time>
#include <base/VFile>
#include <base/Application>
#include <base/Point3>

#include <robot/Robot>
#include <robot/RobotDescription>


using apps::InverseKinematicsControlWidget;

using base::Math;
using base::Time;
using base::Point3;
using base::Vector3;
using base::Orient;
using base::Trajectory;
using base::VFile;
using base::Application;


using robot::Robot;
using robot::RobotDescription;
using robot::PlatformDescription;
using robot::ManipulatorDescription;
using robot::ControlInterface;
using robot::control::kinematics::IKORController;
using robot::sim::BasicEnvironment;




// GTKmm
#include <gtkmm.h>

using Gtk::manage;


InverseKinematicsControlWidget::InverseKinematicsControlWidget(GUIControlWindow& parentWindow) 
  : ControlInterfaceWidget(parentWindow), targetLock(false), orientationControl(false),
    dialog(0), validTrajectoryLoaded(false), playingTraj(false)
{
}


void InverseKinematicsControlWidget::init(ref<BasicEnvironment> env, ref<Robot> robot, Int index, ref<ControlInterface> interface)
{
  posInterface = interface;
  this->robot = robot;
  this->index = index;

  // end-effector position controls
  Gtk::VBox* eeControlBox = new Gtk::VBox(false,5);
  
  Gtk::HBox* hbox = new Gtk::HBox();
  orientControlCheck = new Gtk::CheckButton("Orientation Control");
  orientControlCheck->set_active(false);
  orientControlCheck->signal_clicked().connect( SigC::slot(*this, &InverseKinematicsControlWidget::orientationControlClicked ) );
  hbox->pack_start(*manage(orientControlCheck), Gtk::PACK_SHRINK,5);

  targetLockButton = new Gtk::CheckButton("Lock Target");
  targetLockButton->signal_clicked().connect( SigC::slot(*this, &InverseKinematicsControlWidget::targetLockClicked ) );
  hbox->pack_start(*manage(targetLockButton), Gtk::PACK_SHRINK,5);
  eeControlBox->pack_start(*manage(hbox), Gtk::PACK_SHRINK,5);

  // !!!perhaps a solution method combo here...

  eeControlBox->pack_start(*manage(new Gtk::HSeparator()), Gtk::PACK_SHRINK,5);
  
  eeCoordButtons.resize(6); // x,y,z [,R,P,Y]
  eeadjustments.resize(6);
  skipUpdate.resize(6);
  for(Int i=0; i<6; i++) skipUpdate[i] = 0;

  // X, Y, Z      
  eeadjustments[0] = new Gtk::Adjustment(0, -5, 5, 0.0005, 0.1);
  eeadjustments[0]->signal_value_changed().connect( SigC::bind<Int>( SigC::slot(*this, &InverseKinematicsControlWidget::eeScaleChanged ), 0) );
  eeCoordButtons[0] = new Gtk::SpinButton(*manage(eeadjustments[0]));
  eeCoordButtons[0]->set_digits(5);
  hbox = new Gtk::HBox();
  hbox->pack_start(*manage(new Gtk::Label("X")), Gtk::PACK_SHRINK,5);
  hbox->pack_end(*manage(eeCoordButtons[0]));
  eeControlBox->pack_start(*manage(hbox), Gtk::PACK_SHRINK,5);

  eeadjustments[1] = new Gtk::Adjustment(0, -5, 5, 0.0005, 0.1);
  eeadjustments[1]->signal_value_changed().connect( SigC::bind<Int>( SigC::slot(*this, &InverseKinematicsControlWidget::eeScaleChanged ), 1) );
  eeCoordButtons[1] = new Gtk::SpinButton(*manage(eeadjustments[1]));
  eeCoordButtons[1]->set_digits(5);
  hbox = new Gtk::HBox();
  hbox->pack_start(*manage(new Gtk::Label("Y")), Gtk::PACK_SHRINK,5);
  hbox->pack_end(*manage(eeCoordButtons[1]));
  eeControlBox->pack_start(*manage(hbox), Gtk::PACK_SHRINK,5);

  eeadjustments[2] = new Gtk::Adjustment(0, -5, 5, 0.0005, 0.1);
  eeadjustments[2]->signal_value_changed().connect( SigC::bind<Int>( SigC::slot(*this, &InverseKinematicsControlWidget::eeScaleChanged ), 2) );
  eeCoordButtons[2] = new Gtk::SpinButton(*manage(eeadjustments[2]));
  eeCoordButtons[2]->set_digits(5);
  hbox = new Gtk::HBox();
  hbox->pack_start(*manage(new Gtk::Label("Z")), Gtk::PACK_SHRINK,5);
  hbox->pack_end(*manage(eeCoordButtons[2]));
  eeControlBox->pack_start(*manage(hbox), Gtk::PACK_SHRINK,5);
  
  // R, P, Y
  orientationBox = new Gtk::VBox(false,5);
  
  eeadjustments[3] = new Gtk::Adjustment(0, -180, 179.9, 0.01, 0.1);
  eeadjustments[3]->signal_value_changed().connect( SigC::bind<Int>( SigC::slot(*this, &InverseKinematicsControlWidget::eeScaleChanged ), 3) );
  eeCoordButtons[3] = new Gtk::SpinButton(*manage(eeadjustments[3]));
  eeCoordButtons[3]->set_digits(2);
  hbox = new Gtk::HBox();
  hbox->pack_start(*manage(new Gtk::Label("Roll")), Gtk::PACK_SHRINK,5);
  hbox->pack_end(*manage(eeCoordButtons[3]));
  orientationBox->pack_start(*manage(hbox), Gtk::PACK_SHRINK,5);
  
  eeadjustments[4] = new Gtk::Adjustment(0, -180, 179.9, 0.01, 0.1);
  eeadjustments[4]->signal_value_changed().connect( SigC::bind<Int>( SigC::slot(*this, &InverseKinematicsControlWidget::eeScaleChanged ), 4) );
  eeCoordButtons[4] = new Gtk::SpinButton(*manage(eeadjustments[4]));
  eeCoordButtons[4]->set_digits(2);
  hbox = new Gtk::HBox();
  hbox->pack_start(*manage(new Gtk::Label("Pitch")), Gtk::PACK_SHRINK,5);
  hbox->pack_end(*manage(eeCoordButtons[4]));
  orientationBox->pack_start(*manage(hbox), Gtk::PACK_SHRINK,5);

  eeadjustments[5] = new Gtk::Adjustment(0, -180, 179.9, 0.01, 0.1);
  eeadjustments[5]->signal_value_changed().connect( SigC::bind<Int>( SigC::slot(*this, &InverseKinematicsControlWidget::eeScaleChanged ), 5) );
  eeCoordButtons[5] = new Gtk::SpinButton(*manage(eeadjustments[5]));
  eeCoordButtons[5]->set_digits(2);
  hbox = new Gtk::HBox();
  hbox->pack_start(*manage(new Gtk::Label("Yaw")), Gtk::PACK_SHRINK,5);
  hbox->pack_end(*manage(eeCoordButtons[5]));
  orientationBox->pack_start(*manage(hbox), Gtk::PACK_SHRINK,5);
  
  eeControlBox->pack_start(*manage(orientationBox), Gtk::PACK_SHRINK,5);
  
  
  
  Gtk::Frame* paramFrame = new Gtk::Frame("Controller Parameters");
  paramFrame->set_border_width(5);
  
  eeControlBox->pack_start(*manage(paramFrame));
  Gtk::VBox* paramBox = new Gtk::VBox(false,5);
  paramFrame->add(*manage(paramBox));
  
  hbox = new Gtk::HBox();
  hbox->pack_start(*manage(new Gtk::Label("Obst Danger dist:")), Gtk::PACK_SHRINK,5);
  dangerAdj = new Gtk::Adjustment(0.08, 0.01, 1, 0.00025, 0.02);
  dangerAdj->signal_value_changed().connect( SigC::slot(*this, &InverseKinematicsControlWidget::dangerDistChanged ) );
  Gtk::SpinButton* dangerSpin = new Gtk::SpinButton(*manage(dangerAdj));
  dangerSpin->set_digits(3);
  hbox->pack_start(*manage(dangerSpin));
  
  paramBox->pack_start(*manage(hbox), Gtk::PACK_SHRINK,5);
  
  
  // controls for ee trajectory playback
  Gtk::Frame* trajFrame = new Gtk::Frame("Trajectory following");
  trajFrame->set_border_width(5);
  
  eeControlBox->pack_start(*manage(trajFrame));
  Gtk::VBox* trajBox = new Gtk::VBox(false,5);
  trajFrame->add(*manage(trajBox));
  
  hbox = new Gtk::HBox(false,5);
  hbox->pack_start(*manage(new Gtk::Label("File:")), Gtk::PACK_SHRINK,5);
  trajFile = new Gtk::Entry();
  trajFile->signal_activate().connect( SigC::slot(*this, &InverseKinematicsControlWidget::filenameActivated) );
  hbox->pack_start(*manage(trajFile));
  Gtk::Button* browseButton = new Gtk::Button("Browse...");
  hbox->pack_start(*manage(browseButton), Gtk::PACK_SHRINK,5);
  browseButton->signal_clicked().connect( SigC::slot(*this, &InverseKinematicsControlWidget::browseClicked ) );
  
  trajBox->pack_start(*manage(hbox), Gtk::PACK_SHRINK,5);

  
  Gtk::Button* stopButton = new Gtk::Button();
  ref<VFile> stopImageFile( Application::getInstance()->universe()->cache()->findFile(String("images/stop_button.xpm")) );
  stopButton->add(*manage(new Gtk::Image(stopImageFile->pathName().str())));
  stopButton->signal_clicked().connect( SigC::slot(*this, &InverseKinematicsControlWidget::stopClicked) );
  
  Gtk::Button* playButton = new Gtk::Button();
  ref<VFile> playImageFile( Application::getInstance()->universe()->cache()->findFile(String("images/play_button.xpm")) );
  playButton->add(*manage(new Gtk::Image(playImageFile->pathName().str())));
  playButton->signal_clicked().connect( SigC::slot(*this, &InverseKinematicsControlWidget::playClicked) );
  
  Gtk::HBox* playControlsBox = new Gtk::HBox(false,5);
  playControlsBox->pack_start(*manage(stopButton), Gtk::PACK_SHRINK,5);
  playControlsBox->pack_start(*manage(playButton), Gtk::PACK_SHRINK,5);

  trajBox->pack_start(*manage(playControlsBox), Gtk::PACK_SHRINK,5);
  
  instantiateController();
  
  pack_start(*manage(eeControlBox));
}


void InverseKinematicsControlWidget::instantiateController()
{
  // create an IK controller
  ikorController = ref<IKORController>(NewObj IKORController(IKORController::FSPLagrangian,
                                       robot, index, false, 
                                       orientationControl, Orient::EulerRPY));
  ikorController->setControlInterface(posInterface);
  ikorController->setProximityDangerDistance( dangerAdj->get_value() );
  eePosInterface = ikorController->getControlInterface(); // only one - no need for name
}


void InverseKinematicsControlWidget::eeScaleChanged(Int i)
{
   if (ikorController == 0) return;
   if (!orientationControl && (i>2)) return; // ignore orientation if not controlling it
   eePosInterface->setOutput(i, eeadjustments[i]->get_value() );
   skipUpdate[i] = 5;
}


void InverseKinematicsControlWidget::orientationControlClicked()
{
  orientationControl = orientControlCheck->get_active();
  instantiateController();
}

  
void InverseKinematicsControlWidget::targetLockClicked()
{
  targetLock = targetLockButton->get_active();
}

void InverseKinematicsControlWidget::dangerDistChanged()
{
  if (ikorController == 0) return;
  ikorController->setProximityDangerDistance( dangerAdj->get_value() );
}


void InverseKinematicsControlWidget::browseClicked()
{
  if (dialog != 0) return; // only allow one dialog open at a time
  
  // open a file dialog so the user can select an ee trajectory file
  dialog = new Gtk::FileSelection("Please choose trajectory or path file");
  dialog->signal_response().connect( SigC::slot(*this, &InverseKinematicsControlWidget::fileDialogResponse ) );
  dialog->set_transient_for(parentWindow);
  dialog->show();
}

void InverseKinematicsControlWidget::fileDialogResponse(int response_id)
{
  bool fileSelected = false;
  std::string filename;
  switch (response_id) {
    case Gtk::RESPONSE_OK: {
      fileSelected = true;
      filename = dialog->get_filename();
    } break;
    
    case Gtk::RESPONSE_CANCEL: {
    } break;
    default: ;
  }
  
  dialog->hide();
  delete dialog;
  dialog = 0;
  
  if (fileSelected) {
    trajFile->set_text(filename);
    loadTrajectory(filename);
  }
}


void InverseKinematicsControlWidget::filenameActivated()
{
  String filename = trajFile->get_text();
  loadTrajectory(filename);
}


void InverseKinematicsControlWidget::iterate(const base::Time& simTime)
{
  if (!(lastSimTime < simTime)) return;
  
  if (ikorController != 0) {
    try {
      ikorController->iterate(simTime);
    } catch (std::exception& e) {
      Logln("exception:" << e.what());
    }
    
    
    if (!playingTraj) { // if not in trajectory playback mode
      
      // update adjustments (if not being adjusted or target is locked)
      if (!targetLock) {
        for (Int i=0; i<6; i++)
          if (skipUpdate[i] == 0) eeadjustments[i]->set_value( eePosInterface->getInput(i) );
      }
      
      for(Int i=0; i<6; i++) {
        if (skipUpdate[i] > 0)
          skipUpdate[i]--;
      }
    }
    else { // trajectory playback mode
      
      Point3 trajPos( traj.position(traj_t) ); // current trajectory point
      Orient trajOrient( traj.orientation(traj_t) );
      trajOrient.changeRepresentation(Orient::EulerRPY);
      
      // update ee
      eePosInterface->setOutput(0, trajPos.x);
      eePosInterface->setOutput(1, trajPos.y);
      eePosInterface->setOutput(2, trajPos.z);
      if (orientationControl) {
        eePosInterface->setOutput(0, trajOrient[0]);
        eePosInterface->setOutput(1, trajOrient[1]);
        eePosInterface->setOutput(2, trajOrient[2]);
      }
      
      // update adjustments
      eeadjustments[0]->set_value(trajPos.x);
      eeadjustments[1]->set_value(trajPos.y);
      eeadjustments[2]->set_value(trajPos.z);
      eeadjustments[3]->set_value(trajOrient[0]);
      eeadjustments[4]->set_value(trajOrient[1]);
      eeadjustments[5]->set_value(trajOrient[2]);
      
      
      traj_t += (simTime - lastSimTime); // update trajectory time point for next time

      if (traj_t > traj.time(1))  // end of trajectory, stop playback
        playingTraj = false;
      
    }
      
  }
  
  lastSimTime = simTime;
}


void InverseKinematicsControlWidget::loadTrajectory(const String& filename)
{
  // try to read the file
  try {
    ref<VFile> file( Application::getInstance()->universe()->cache()->findFile(filename) );
    traj.load(file);
    
    String extraStatus;
    
    // check it is in a frame we can handle
    if (traj.getCoordFrame() == "eebase") {
      // translate origin to ee
      Vector3 eePos(eePosInterface->getInput(0), eePosInterface->getInput(1), eePosInterface->getInput(2));
      traj.translate(eePos);
      validTrajectoryLoaded = true;
    }
    else if (traj.getCoordFrame() == "base") {
      // OK, nothing to do
      validTrajectoryLoaded = true;
    }
    else if (traj.getCoordFrame() == "") {
      // assume 'base' - nothing to do
      extraStatus = String(" (assuming 'base' frame)");
      validTrajectoryLoaded = true;
    }
    else { // unknown unsupported frame
      validTrajectoryLoaded = false;
      parentWindow.pushStatusMessage(GUIControlWindow::StatusError, String("Unsupported trajectory coord. frame:")+traj.getCoordFrame());
    }
    
    if (validTrajectoryLoaded)  
      parentWindow.pushStatusMessage(GUIControlWindow::StatusInfo, String("Trajectory ")+file->name().str()+" loaded"+extraStatus);
  } catch (std::exception& e) {
    // set an error status message
    parentWindow.pushStatusMessage(GUIControlWindow::StatusError, String("Trajectory read error:"+filename+"--"+e.what()));
    Logln("Unable to open file:" << filename << " -- " << e.what());
    validTrajectoryLoaded = false;
  }
  
}



void InverseKinematicsControlWidget::stopClicked()
{
  playingTraj = false;
}

void InverseKinematicsControlWidget::playClicked()
{
  if (!validTrajectoryLoaded) return; // nothing to play

  if (!playingTraj) {
    playingTraj = true;
    traj_t = traj.time(0);
  }
}

