/****************************************************************************
  Copyright (C)2002 David Jung <opensim@pobox.com>

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
  
  $Id: poscontrol.cpp 1028 2004-02-11 20:45:27Z jungd $
  $Revision: 1.16 $
  $Date: 2004-02-11 15:45:27 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <iostream>
#include <iomanip>

#include <robot/robot>

#include <base/ref>
#include <base/Application>
#include <base/Universe>
#include <base/Time>
#include <base/VFile>
#include <base/Vector>
#include <base/Matrix>
#include <base/PathName>
#include <base/Trajectory>

#include <gfx/Color4>
#include <gfx/LookAtCameraManipulator>
#include <gfx/VisualPath>

#include <physics/ODESolidSystem>
#include <physics/ODECollisionDetector>
#include <physics/VisualDebugUtil>

#include <robot/Controllable>
#include <robot/Robot>
#include <robot/RobotController>
#include <robot/PlatformDescription>
#include <robot/ControlInterface>
#include <robot/control/ControllableAdaptor>
#include <robot/control/ManipulatorPIDPositionController>
#include <robot/control/kinematics/IKORController>
#include <robot/ManipulatorJointTrajectory>
#include <robot/sim/SimulatedRobot>
#include <robot/sim/SimulatedBasicEnvironment>


// OSG
#include <osg/Vec4>
#include <osg/Group>
#include <osg/Notify>
#include <osg/Camera>
#include <osg/Fog>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/Material>
#include <osgUtil/SceneView>
#include <osgGLOW/Viewer>



using base::Application;
using base::Universe;
using base::Math;
using base::Time;
using base::PathName;
using base::VFile;
using base::Point3;
using base::Quat4;
using base::Vector3;
using base::Matrix4;
using base::Orient;
using base::Vector;
using base::Trajectory;
using base::Path;
using base::zeroVector;
using base::vectorRange;
using base::matrixColumn;
using base::matrixRange;
using base::Range;
using base::ref;
using gfx::LookAtCameraManipulator;
using gfx::VisualPath;
using gfx::Color4;
using physics::Solid;
using physics::SolidSystem;
using physics::ODESolidSystem;
using physics::CollisionDetector;
using physics::ODECollisionDetector;
using physics::FixedConstraint;
using physics::ConstraintGroup;
using physics::VisualDebugUtil;
using robot::Controllable;
using robot::Robot;
using robot::RobotDescription;
using robot::PlatformDescription;
using robot::ManipulatorDescription;
using robot::ToolDescription;
using robot::ManipulatorJointTrajectory;
using robot::KinematicChain;
using robot::ControlInterface;
using robot::control::ControllableAdaptor;
using robot::control::ManipulatorPIDPositionController;
using robot::sim::BasicEnvironment;
using robot::sim::SimulatedBasicEnvironment;
using robot::sim::SimulatedRobot;



//
// temp includes for testing
//
#include <base/Vector>
#include <base/Matrix>
#include <base/SVD>
#include <physics/Box>
#include <robot/ControlInterface>
#include <robot/Robot>

using base::ref;
using base::narrow_cast_ref;
using base::Vector;
using base::Matrix;
using base::ExpressionVector;
using base::ExpressionMatrix;
using physics::Box;


using robot::operator<<;



// GUI classes

#include "glow.h"
#include "glowQuickPalette.h"

#include "glowLabelWidget.h"
#include "glowPushButtonWidget.h"
#include "glowSliderWidget.h"
#include "glowAux.h"

using glow::Glow;
using glow::GlowQuickPalette;
using glow::GlowQuickPaletteWindow;
using glow::GlowQuickPanelWidget;

using glow::GlowWindow;
using glow::GlowFont;
using glow::GlowFixedSizeWindow;
using glow::GlowPushButtonReceiver;
using glow::GlowWidgetSubwindow;
using glow::GlowLabelParams;
using glow::GlowLabelWidget;
using glow::GlowPushButtonWidget;
using glow::GlowCheckBoxWidget;
using glow::GlowCheckBoxReceiver;
using glow::GlowCheckBoxMessage;
using glow::GlowPushButtonParams;
using glow::GlowPushButtonMessage;
using glow::GlowLabeledSliderParams;
using glow::GlowLabeledSliderWidget;
using glow::GlowSliderReceiver;
using glow::GlowSliderWidget;
using glow::GlowSliderMessage;

Int viewerWidth;
GlowFixedSizeWindow* wind;
GlowQuickPaletteWindow* controlWindow;
GlowSliderWidget* jointSlider[64];
GlowSliderWidget* toolJointSlider[64];
GlowLabelWidget*  jointProximityLabel[64];
GlowSliderWidget* driveSlider;
GlowSliderWidget* steeringSlider;
GlowLabelWidget* eePosLabel[2];
GlowCheckBoxWidget* worldCheckbox;
GlowPushButtonWidget* attachButton;
GlowPushButtonWidget* releaseButton;
GlowPushButtonWidget* zeroButton;
GlowSliderWidget* trajControlSlider;
GlowPushButtonWidget* recordButton;
GlowPushButtonWidget* stopButton;
GlowPushButtonWidget* playButton;
GlowPushButtonWidget* saveButton;
GlowPushButtonWidget* randomButton;
GlowPushButtonWidget* quitButton;
bool randomAngles;
bool worldFrame;
ref<SimulatedBasicEnvironment> simEnv;
ref<SimulatedRobot> simRobot;



static void update();
static void updateEEPosition();
static void updateProximitySensors();

static VisualDebugUtil visualDebugUtil;


// helper class: OSG calls reset() before each App traversal of the scene (once before
//  each 'frame')
class Updater : public osg::NodeVisitor 
{
public:
  Updater(base::ref<base::Universe> u, 
	     osg::Camera* camera, 
	     base::ref<robot::ControlInterface> manipInterface,
	     base::ref<robot::ControlInterface> toolInterface,
	     ManipulatorJointTrajectory t) 
    : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN), u(u), camera(camera), 
      manipInterface(manipInterface), toolInterface(toolInterface),
      traj(t), eePosition(zeroVector(6)),
      freeze(false), dynamic(false),count(0), 
      drive(0), steering(0),
      playback(false)
  {
    N = manipInterface->outputSize();
    toolN = toolInterface->outputSize();
    attached=false;
    for(Int i=0; i<64; i++) {
      params[i]=0;
      tparams[i]=0;
    }
  }

  Int N;
  Int toolN;

  virtual void reset()
  {
    const Real frameTime = 1/50.0; //1/30;
//    const Real stepTime = 1/240.0;
    const Real stepTime = 1/100.0;

    if (!freeze) {
      //
      // Controls

      if (randomAngles) {
	if (count%130 == 0) {
	  for(Int j=0; j<N; j++)
	    params[j] = Math::random()*(maxLimit[j]-minLimit[j]) + minLimit[j];
	  if (params[1]<0) params[1]=-params[1];
	  update();
	  updateEEPosition();
          updateProximitySensors();
	}
      }


      if (playback) {
	Real trajTime( Math::minimum(simTimeElapsed+traj.time(0).seconds(),traj.time(1).seconds()) );
	Vector q( traj.q(trajTime) );
	for(Int i=0; i<N; i++) 
	  if (revolute[i]) 
	    params[i] = Math::radToDeg(q[i]);
	  else
	    params[i] = q[i];
	
	if (simTimeElapsed >= traj.time(1).seconds()-traj.time(0).seconds()) {
	  playback = false;
	  simTimeElapsed = traj.time(1).seconds()-traj.time(0).seconds();
	  trajControlSlider->SetValue(traj.time(0).seconds()+simTimeElapsed);
	  update();
	  updateEEPosition();
          updateProximitySensors();
	}
	else
	  if (count%30) {
	    update();
	    updateEEPosition();
	  }
      }

      
      if (count%30) {
        updateProximitySensors();
      }
      

      for(Int j=0; j<N; j++)
        manipInterface->setOutput(j,
		       revolute[j]?Math::degToRad(params[j]):params[j]);

      if (attached)
        for(Int j=0; j<toolN; j++)
          toolInterface->setOutput(j, trevolute[j]?Math::degToRad(tparams[j]):tparams[j]);

      if (platformInterface && !pd->isHolonomic() && pd->isMobile()) {
//        Debugln(Tmp,"setting outputs: " << drive << "," << drive << "," << steering);
        platformInterface->setOutput(0,drive);
        platformInterface->setOutput(1,drive -1); //!!! -1
        platformInterface->setOutput(2,steering);
      }
      
      // update camera target
      Point3 robotPos(simRobot->getPosition());
      if (lookAtCameraManip->trackingEnabled())
        lookAtCameraManip->setTarget(robotPos.x,robotPos.y,robotPos.z);

      
      /*
	if (count%50 == 0) {
	if (count%100 == 0) {
	Debugcln(DJ,"PLUS");
	controllable->getControlInterface()->setOutput(0,Math::degToRad(45));
	//	controllable->getControlInterface()->setOutput(1,-0.005);
	}
	else {
	Debugcln(DJ,"MINUS");
	controllable->getControlInterface()->setOutput(0,Math::degToRad(45));
	//	controllable->getControlInterface()->setOutput(1,0.005);
	}
	}
      */
      
      // iterate controllers 
      base::reflist<robot::Controller>::iterator c(controllers.begin());
      base::reflist<robot::Controller>::iterator end(controllers.end());
      while (c != end) {
	base::ref<robot::Controller> controller(*c);
	controller->iterate(simTime);
	++c;
      }
      
      
      //
      // Simulation 
      
      Color4 col("lime green",0.2);
      VisualDebugUtil::setColorAll(col);

      Int iterations = Int(frameTime / stepTime);
      
      Time startTime(Time::now());
      for(Int s=0; s<iterations; s++)
        u->simulateForSimTime(stepTime); // step the simulation of the universe
      simTime += Time(stepTime*iterations);
      if (playback) simTimeElapsed += stepTime*iterations;
      
      // now wait until frameTime sec has elapsed if necessary
      //  (so the simulation doesn't run faster than real-time)
      Time elapsed(Time::now() - startTime);
      
      if (elapsed.seconds() < frameTime) {
        Real sleepFor = (frameTime - elapsed.seconds())*0.2;
        Time::sleep(sleepFor);
      }

      count++;
      
      VisualDebugUtil::updateVisual();

    }// if(!freeze)
    else
      Time::sleep(frameTime);

  }

  base::Time simTime;
  base::ref<Universe> u;
  osg::Camera* camera;
  gfx::LookAtCameraManipulator* lookAtCameraManip;
  base::ref<robot::ControlInterface> platformInterface;
  base::ref<robot::ControlInterface> manipInterface;
  base::ref<robot::ControlInterface> toolInterface;
  base::ref<robot::ControlInterface> proxInterface;
  robot::KinematicChain chain;
  ManipulatorJointTrajectory traj; // current trajectory
  base::ref<const robot::PlatformDescription> pd;
  Vector eePosition;
  base::reflist<robot::Controller> controllers; // Controllers to iterate()
  bool freeze, dynamic;
  Int count;
  Real drive, steering;
  Real params[128]; // joint parameters
  bool revolute[128]; // is joint revolute?
  Real minLimit[128];
  Real maxLimit[128];
  bool attached; // is tool attached?
  Real tparams[128]; // tool joint parameters
  bool trevolute[128]; // is joint revolute?
  Real tminLimit[128];
  Real tmaxLimit[128];
  bool playback;
  Real simTimeElapsed;
};

Updater* updater;






class EventReceiver : public GlowPushButtonReceiver,
		      public GlowSliderReceiver,
		      public GlowCheckBoxReceiver 
{
public:
  
  virtual void OnMessage(const GlowPushButtonMessage& message) 
  {
    if (message.widget == quitButton)
      Glow::exitMainLoop();
    if (message.widget == randomButton) {
      randomAngles = !randomAngles;
      updater->count=0;
      updater->playback = false;
    }

    if (message.widget == zeroButton) {
      for(Int i=0; i<updater->N; i++) 
	updater->params[i]=0;
      randomAngles = false;
      updater->playback = false;
      update();
      updateEEPosition();
    }

    if (message.widget == playButton) {
      updater->playback = true;
      updater->simTimeElapsed = trajControlSlider->GetValue() - updater->traj.time(0).seconds();

      if (Math::equals(updater->simTimeElapsed, updater->traj.time(1).seconds(),0.0001))
	updater->simTimeElapsed = 0;
      update();
      updateEEPosition();
    }

    if (message.widget == stopButton) {
      updater->playback = false;
      update();
      updateEEPosition();
    }
    
    if (message.widget == saveButton) {
      ref<VFile> trajFile( updater->u->filesystem()->current()->createFile(String("savedjtraj.xml")) );
      updater->traj.save(trajFile,trajFile->extension());
      Consoleln("Saved trajectory in file '" << trajFile->pathName().str() << "'.");
    }

    if (message.widget == attachButton) {
      if (simEnv->numTools() > 0) {
	simEnv->placeToolInProximity(simEnv->getTool(0), simRobot);
	simRobot->graspTool(0);
	updater->attached=true;
      }
    }

    if (message.widget == releaseButton) {
      simRobot->releaseGrasp();
      updater->attached=false;
    }

  }
  
  virtual void OnMessage(const GlowSliderMessage& message) 
  {
    for(Int i=0; i<updater->N; i++)
      if (message.widget == jointSlider[i]) 
	updater->params[i] = message.value;

    if (updater->platformInterface && !updater->pd->isHolonomic()) {
      if (message.widget == driveSlider)
        updater->drive = message.value;
      if (message.widget == steeringSlider)
        updater->steering = message.value;
    }

    if (updater->attached)
      for(Int i=0; i<updater->toolN; i++)
        if (message.widget == toolJointSlider[i]) 
          updater->tparams[i] = message.value;
    
    if (message.widget == trajControlSlider) {
      Vector q( updater->traj.q(Time(message.value)) );
      for(Int i=0; i<updater->N; i++) 
        updater->params[i] = updater->revolute[i]?Math::radToDeg(q[i]):q[i];
      update();
    }
    updateEEPosition();
    updateProximitySensors();

    randomAngles = false;
    updater->playback = false;
  }

  void OnMessage(const GlowCheckBoxMessage& message)
  {
    if (message.widget == worldCheckbox) {
      worldFrame = (message.state == GlowCheckBoxWidget::off);
      updateEEPosition();
    }
  }

};


static void update()
{
  for(Int i=0; i<updater->N; i++) 
    jointSlider[i]->SetValue(updater->params[i]);

  if (updater->attached)
    for(Int i=0; i<updater->toolN; i++) 
      toolJointSlider[i]->SetValue(updater->tparams[i]);

  if (updater->playback) {
    Real trajTime( Math::minimum(updater->simTimeElapsed+updater->traj.time(0).seconds(),
				 updater->traj.time(1).seconds()) );
    trajControlSlider->SetValue( trajTime );
    stopButton->Activate();
    playButton->Deactivate();
    recordButton->Deactivate();
  }
  else {
    stopButton->Deactivate();
    playButton->Activate();
    recordButton->Activate();
  }
}


static void updateProximitySensors()
{
  char buf[64];
  for(Int i=0; i<=updater->N; i++) {
    Real proximityDistance = updater->proxInterface->getInput(i*5);
    if (proximityDistance > consts::maxInt)
      sprintf(buf,"%d: no object",i);
    else
      sprintf(buf,"%d: %5.3f",i,proximityDistance);
    jointProximityLabel[i]->SetText(buf);
  }  
}


static void updateEEPosition()
{
  // calc EE position & orientation  using the current joint parameters and the forward kinematics transform (Tn)
  Vector q(updater->N);
  for(Int i=0; i<updater->N; i++)
    q[i] = updater->revolute[i]?Math::degToRad(updater->params[i]):updater->params[i];
  
  Matrix T(updater->chain.getForwardKinematics(q)); // 4x4 transform to ee

  if (worldFrame) {
    Matrix4 W4( simRobot->coordFrameTransform(Robot::EndEffectorFrame,
					      Robot::WorldFrame, 0, base::toMatrix4(T),
					      simRobot->getPosition(),
					      simRobot->getOrientation()) );
    T = base::fromMatrix4(W4);
  }


  //  the position is the first 3 elements of column 4 of T
  //  the orientation is the 3x3 submatrix of T - converted into a vector
  //  in the representation 'orientRep'.
  Vector pos(3); pos = vectorRange(Vector(matrixColumn(T,3)), Range(0,3));
  Matrix rot(3,3); rot = matrixRange(T, Range(0,3), Range(0,3));
  Vector orient( Orient(rot).getVector(Orient::EulerRPY) );
  Vector x(6);
  vectorRange(x, Range(0,3)) = pos;
  vectorRange(x, Range(3,x.size())) = orient;
  
  char buf[64];
  sprintf(buf,"(x:%3.1f y:%3.1f z:%3.1f)", x[0],x[1],x[2]);
  eePosLabel[0]->SetText(buf);
  sprintf(buf,"(R:%3.1f P:%3.1f Y:%3.1f)", 
	  Math::radToDeg(x[3]), Math::radToDeg(x[4]), Math::radToDeg(x[5]) );
  eePosLabel[1]->SetText(buf);
}



static void createControls()
{

  char title[64];
  strncpy(title,(String("OpenSim ") + Application::getVersion() + " - Controls").c_str(),64);
  controlWindow = NewObj GlowQuickPaletteWindow(title, viewerWidth+10, 10, GlowQuickPalette::vertical,GlowQuickPalette::alignExpand);

  EventReceiver* rec = NewObj EventReceiver();

  GlowQuickPanelWidget* tophpanel = controlWindow->AddArrangingPanel(GlowQuickPalette::horizontal);
  
  GlowQuickPanelWidget* jhPanel = tophpanel->AddArrangingPanel(GlowQuickPalette::vertical);

  GlowQuickPanelWidget* tpanel = jhPanel->AddPanel(GlowQuickPanelWidget::etchedStyle, "Joint parameters");
  GlowQuickPanelWidget* vpanel = tpanel->AddArrangingPanel(GlowQuickPalette::vertical);

  Assert(updater->N < 20);
  for(Int i=0; i<updater->N; i++) {

    char jointIndex[8];
    sprintf(jointIndex,"%d:%c5.2f ",i,'%');
    Real minLimit = updater->minLimit[i];
    Real maxLimit = updater->maxLimit[i];
    if ((minLimit < -180.0) && (maxLimit>180.0)) {
      minLimit = -360.0; maxLimit=360.0;
    }
    jointSlider[i] = vpanel->AddSlider(minLimit, maxLimit, 0,
                                       GlowSliderWidget::defaultOptions, 10, "%5.2f", jointIndex, rec);
                        
  }

  
  // proximity
  GlowQuickPanelWidget* phPanel = tophpanel->AddArrangingPanel(GlowQuickPalette::vertical);

  GlowQuickPanelWidget* ptpanel = phPanel->AddPanel(GlowQuickPanelWidget::etchedStyle, "Proximity sensors");
  GlowQuickPanelWidget* pvpanel = ptpanel->AddArrangingPanel(GlowQuickPalette::vertical);

  char buf[64];  
  for(Int i=0; i<=updater->N; i++) {
    sprintf(buf,"%d: %5.3f",i,0.0);
    jointProximityLabel[i] = pvpanel->AddLabel(buf);
    jointProximityLabel[i]->SetFont(GlowFont::fixed8by13);
  }  
  

  if (updater->toolN > 0) {
    GlowQuickPanelWidget* tlpanel = jhPanel->AddPanel(GlowQuickPanelWidget::etchedStyle, "Tool Joint parameters");
    GlowQuickPanelWidget* tvpanel = tlpanel->AddArrangingPanel(GlowQuickPalette::vertical);
  
    Assert(updater->toolN < 20);
    for(Int i=0; i<updater->toolN; i++) {
  
      char jointIndex[8];
      sprintf(jointIndex,"%d:%c5.2f ",i,'%');
      Real minLimit = updater->tminLimit[i];
      Real maxLimit = updater->tmaxLimit[i];
      if ((minLimit < -180.0) && (maxLimit>180.0)) {
        minLimit = -360.0; maxLimit=360.0;
      }
      toolJointSlider[i] = tvpanel->AddSlider(minLimit, maxLimit, 0,
                          GlowSliderWidget::defaultOptions, 10, "%5.2f", jointIndex, rec);
    }
  }

  if (updater->pd->isMobile() && !updater->pd->isHolonomic()) {
    GlowQuickPanelWidget* plpanel = jhPanel->AddPanel(GlowQuickPanelWidget::etchedStyle, "Platform controls");
    GlowQuickPanelWidget* pvpanel = plpanel->AddArrangingPanel(GlowQuickPalette::vertical);
    driveSlider = pvpanel->AddSlider(-30,30,0,GlowSliderWidget::defaultOptions,
				     10, "%5.2f", "drive", rec);
    steeringSlider = pvpanel->AddSlider(-7,7,0,GlowSliderWidget::defaultOptions,
					10, "%5.2f", "steering", rec);
  }


  GlowQuickPanelWidget* epanel = controlWindow->AddPanel(GlowQuickPanelWidget::etchedStyle, "End-Effector Position");

  eePosLabel[0] = epanel->AddLabel("(x:00.0 y:00.0 z:00.0)  ");
  eePosLabel[0]->SetFont(GlowFont::fixed8by13);
  eePosLabel[1] = epanel->AddLabel("(R:00.0 P:00.0 Y:00.0)  ");
  eePosLabel[1]->SetFont(GlowFont::fixed8by13);

  worldCheckbox = epanel->AddCheckBox("base frame", GlowCheckBoxWidget::on, rec); 
  worldFrame=false;

  GlowQuickPanelWidget* hPanel = controlWindow->AddArrangingPanel(GlowQuickPalette::horizontal);

  zeroButton = hPanel->AddPushButton("Zero joints", rec);
  randomButton = hPanel->AddPushButton("Random (toggle)", rec);

  GlowQuickPanelWidget* toolpanel = controlWindow->AddPanel(GlowQuickPanelWidget::etchedStyle, "Tools");
  GlowQuickPanelWidget* thPanel = toolpanel->AddArrangingPanel(GlowQuickPalette::horizontal);
  attachButton = thPanel->AddPushButton("Attach", rec);
  releaseButton = thPanel->AddPushButton("Release", rec);
  

  GlowQuickPanelWidget* trajpanel = controlWindow->AddPanel(GlowQuickPanelWidget::etchedStyle, "Joint Trajectory Control");
  trajControlSlider = trajpanel->AddSlider(updater->traj.time(0).seconds(),updater->traj.time(1).seconds(),
					   0, GlowSliderWidget::defaultOptions,
					   updater->traj.numDistinguishedValues(),
					   "%5.2f", "time", rec);
  GlowQuickPanelWidget* horizPanel = trajpanel->AddArrangingPanel(GlowQuickPalette::horizontal);
  recordButton = horizPanel->AddPushButton("Record", rec);
  recordButton->Deactivate();
  stopButton = horizPanel->AddPushButton("Stop", rec);
  stopButton->Deactivate();
  playButton = horizPanel->AddPushButton("Play", rec);
  
  saveButton = trajpanel->AddPushButton("Save", rec);

  quitButton = controlWindow->AddPushButton("Quit", rec);

  controlWindow->Pack();

}
















int main(int argc, char *argv[])
{
  setDebugOutputID(Tmp);
  addDebugOutputID(DJ);
  abortOnAssertionFailureEnabled(true);
  
  try {
    char *homeenv = getenv("OPENSIM_HOME");
    Assertm(homeenv!=0, "environment variable OPENSIM_HOME defined");
    String home(homeenv);

    Application app(home+"/resources",home+"/cache");
    ref<Universe> universe = app.universe();

    glutInit( &argc, argv );
    std::cout << std::setiosflags(std::ios_base::fixed) << std::setprecision(2) << std::setw(6);

    // parse command-line args
    String usage("\nUsage: poscontrol [-env <environment_spec_file>] [-robot <robot_spec_file> ] [-jtraj <manipulator_joint_trajectory_file>] [-traj <trajectory_or_path_file>]");
    PathName envSpecFileName("defaultenv.xml");
    bool useenv = true;
    PathName robotSpecFileName("defaultrobot.xml");
    bool userobot = false;
    PathName jtrajSpecFileName("defaultjtraj.xml");
    bool usejtraj=true;
    PathName trajSpecFileName("defaultpath.xml");
    bool usetraj=false;
    bool dynamic=true;
    SInt arg=1;
    while (arg < argc) {

      if (String(argv[arg]) == "-env") {
	useenv = true;
	envSpecFileName = String(argv[++arg]);
	++arg;
      }
      else if (String(argv[arg]) == "-robot") {
	userobot = true;
	robotSpecFileName = String(argv[++arg]);
	++arg;
      }
      else if (String(argv[arg]) == "-jtraj") {
	usejtraj = true;
	jtrajSpecFileName =  String(argv[++arg]);
	++arg;
      }
      else if (String(argv[arg]) == "-traj") {
	usetraj = true;
	trajSpecFileName =  String(argv[++arg]);
	++arg;
      }
      else if (String(argv[arg]) == "-static") {
	dynamic = false;
	++arg;
      }
      else {
	Consoleln(usage);
	return -1;
      }

    }

    // save the user pre-pending filenames with data/ 
    if (envSpecFileName.isRelative()) {
      if (!universe->filesystem()->exists(envSpecFileName))
	envSpecFileName.prepend("data/");
    }
    if (robotSpecFileName.isRelative()) {
      if (!universe->filesystem()->exists(robotSpecFileName))
	robotSpecFileName.prepend("data/");
    }
    if (jtrajSpecFileName.isRelative()) {
      if (!universe->filesystem()->exists(jtrajSpecFileName))
	jtrajSpecFileName.prepend("data/");
    }
    if (trajSpecFileName.isRelative()) {
      if (!universe->filesystem()->exists(trajSpecFileName))
	trajSpecFileName.prepend("data/");
    }

    Consoleln("");
    
    // create a simple simulation environment for the robot
    ref<SimulatedBasicEnvironment> env(NewObj SimulatedBasicEnvironment(app.filesystem(), universe->cache(), "Environment",dynamic));
    simEnv = env;
    if (useenv) {
      ref<VFile> envFile( universe->cache()->findFile(envSpecFileName) );
      env->load(envFile,envFile->extension());
      Consoleln("Loaded environment from file '" << envFile->pathName().str() << "'.");
    }


    if (simEnv->numTools() < 1) {
      Consoleln("\nAt least one tool must be present in the environment.");
      Consoleln(usage);
      return -1;
    }
      

    ref<Robot> robot;
    if (userobot) {
      Point3 initPos(0,0,1.1+consts::epsilon);
      Orient initOrient;// = Quat4(Vector3(1,1,0),consts::Pi/4.0);
      ref<VFile> robotSpecFile( universe->cache()->findFile(robotSpecFileName) );
      ref<RobotDescription> rd( env->newRobotDescription() );
      rd->load(robotSpecFile);
      robot = ref<Robot>( env->addRobot(rd, initPos, initOrient, false) );
    }
    else {
      if (env->numRobots() < 1) {
        Consoleln("\nAt least one robot must be present (either in the environment provided or using the -r option)");
        Consoleln(usage);
        return -1;
      }
      robot = env->getRobot(0);
    }

    if (robot->isDescriptionProvided())
      Consoleln("Controlling robot: " << robot->getRobotDescription()->getName());
    simRobot = narrow_ref<SimulatedRobot>(robot);

    // get manipulator to control (first)
    ref<const ManipulatorDescription> manipulator(robot->getRobotDescription()->manipulators()[0]);
    const KinematicChain& chain(manipulator->getKinematicChain());

    // create a PID controller to control the manipulator joint positions (dynamic)
    ref<ManipulatorPIDPositionController> posController(NewObj ManipulatorPIDPositionController(chain));
    posController->setCoeffs(12,0,2); // PID Kp, Ki, Kd
    posController->setControlInterface(robot->getControlInterface("manipulatorVelocity1")); // connect posController to manipulator interface

    // create a direct position control interface (static)
    ref<robot::ControlInterface> manipPositionControlInterface = robot->getControlInterface("manipulatorPosition1");
    
    // get tool to control (first)
    ref<const ToolDescription> tool(env->getTool(0)->getToolDescription());
    const KinematicChain& tchain(tool->getKinematicChain());
   
    // create a PID controller to control the tool joint positions
    ref<ManipulatorPIDPositionController> tposController(NewObj ManipulatorPIDPositionController(tchain));
    tposController->setCoeffs(12,0,2); // PID Kp, Ki, Kd
    tposController->setControlInterface(robot->getControlInterface("toolVelocity1")); // connect posController to tool interface

    
    // obtain interface for proximity sensors
    ref<ControlInterface> proximitySensors( robot->getControlInterface("manipulatorProximity1") );
    

    // output manipulator parameters
    for(Int i=0; i<robot->getRobotDescription()->manipulators().size(); i++) {
      ref<const ManipulatorDescription> manipulator(robot->getRobotDescription()->manipulators()[i]);
      Debugln(Tmp,"Manipulator:" << manipulator->getName() << "\n" << manipulator->getKinematicChain());
    }



    /*
    // add some obstacles for effect
    const Int numObstacles = 5;
    gfx::Color3 col(0.7,0.4,0.8);
    const Real boxDim = 0.4;
    Real dim = boxDim;
    Real z=0.05+consts::epsilon;
    for(Int o=0; o<numObstacles; o++) {
      ref<BasicEnvironment::Obstacle> obstacle( env->addBoxObstacle(base::Dimension3(dim,dim,dim),
								    Point3(0.5,1,z+dim/2),
								    Orient(Quat4(Vector3(0,0,1),Math::degToRad(10+o*5))),
								    String("obstacle")+base::intToString(o) )
						);
      env->setObstacleColor(obstacle,col);
      z += dim + 0.001;
      dim *= 0.85;
    }

    //test save out environment
    ref<VFile> envFile( universe->filesystem()->current()->createFile("savedenv.xml") );
    env->save(envFile,envFile->extension());
    Consoleln("Saved environment in file '" << envFile->pathName() << "'.");
    env->load(envFile,envFile->extension());
    Consoleln("Loaded environment from file '" << envFile->pathName() << "'.");
    
    */

    
    // load joint trajectory
    ref<VFile> jtrajFile( universe->cache()->findFile(jtrajSpecFileName) );
    ManipulatorJointTrajectory trajectory;
    trajectory.load(jtrajFile,jtrajFile->extension());
    // remove platform dof's if any (we don't use them at present)
    ref<const PlatformDescription> pd( robot->getRobotDescription()->platform() );
    if (pd->isMobile()) {
      Consoleln("Warning: assuming joint trajectory has 3-dof for platform - which are being discarded");
      trajectory.setNumJoints( trajectory.getNumJoints() - 3, true );
    }
    manipulator->getKinematicChain().convertJointTrajectory(trajectory); // convert revolute joint values to radians
    
    // load regular/ee trajectory
    ref<VFile> trajFile( universe->cache()->findFile(trajSpecFileName) );
    Trajectory eetrajectory;
    eetrajectory.load(trajFile, trajFile->extension());
    // convert to meters
    if (eetrajectory.getUnits() != "") {
      
      if (eetrajectory.getUnits() == "inches")
	;//eetrajectory.scalePosition(/*consts::metersPerInch*/1.0);//fix!!!!
      else
	if (eetrajectory.getUnits() != "meters") {
	  Consoleln("Warning: unknown units specified for trajectory: '" << eetrajectory.getUnits() << "' - no conversion performed.");
	}
    }
    
    
    
    // transform it to world frame (using trajectory initial configuration)
    trajectory.setNumJoints(chain.dof());
    Vector q( trajectory.q(0) );
    Matrix T( chain.getForwardKinematics(q) ); // ee -> base transform
    Robot::CoordFrame trajFrame( Robot::coordFrame(eetrajectory.getCoordFrame()) );
    if (trajFrame == Robot::UnknownFrame) trajFrame = Robot::WorldFrame; // default to world coords.
    Matrix4 t( robot->coordFrameTransform(trajFrame,
					  Robot::WorldFrame,
					  0, base::toMatrix4(T), 
					  simRobot->getPosition(), 
					  simRobot->getOrientation()) );
    eetrajectory.transform(t);
					  


    // Setup OSG visual 
    osg::Group* rootnode = NewObj osg::Group;
    rootnode->setName("root");

    rootnode->addChild( env->createOSGVisual() );

    // display ee trajectory
    VisualPath eepath(eetrajectory);
    rootnode->addChild( eepath.createOSGVisual() );
    
    // show debugging graphics 
    rootnode->addChild( visualDebugUtil.createOSGVisual(/*gfx::Visual::ShowAxes*/) );

    // add environment to Universe (it's both a World and a Simulatable)
    universe->addWorld(env);
    universe->addSimulatable(env);
    universe->enableSimulationRendering(false);
    universe->preSimulate();

    // setup OSG viewer
    osgGLOW::Viewer* viewer = NewObj osgGLOW::Viewer();
    viewer->addViewport( rootnode );
    viewer->setWindowTitle("OpenSim - Joint position control");
    
    // register our camera manipulator
    gfx::LookAtCameraManipulator lookAtCameraManip(-250,13.5,8,
						   -0.18,0.1,0.7);
    viewer->registerCameraManipulator(&lookAtCameraManip);
    //lookAtCameraManip.trackingEnable();

    // Open window so camera manipulator's warp pointer request will succeed
    viewer->open();
    viewerWidth = viewer->PositionX()+viewer->Width();

    viewer->selectCameraManipulator(0);

    osgUtil::SceneView* sceneView = viewer->getViewportSceneView(0);
    osg::Camera* camera = sceneView->getCamera();

    //!!! this should be an osg::ref_ptr
    updater = new Updater(universe,camera,
				dynamic?posController->getControlInterface():manipPositionControlInterface, 
				tposController->getControlInterface(), 
				trajectory);
    updater->lookAtCameraManip = &lookAtCameraManip;
    updater->controllers.push_back(posController);
    updater->controllers.push_back(tposController);
    if (dynamic)
      updater->platformInterface = robot->getControlInterface("platformVelocity");
    else
      updater->platformInterface = robot->getControlInterface("platformPosition"); 
    updater->proxInterface = proximitySensors;
    updater->pd = robot->getRobotDescription()->platform();
    updater->chain = chain;
    updater->toolN = tchain.dof();
    // set joint limits
    for(Int v=0; v<chain.dof(); v++) {
      const KinematicChain::Link& link( chain.linkOfVariable(v) );
      updater->revolute[v] = (link.type() == KinematicChain::Link::Revolute);
      updater->minLimit[v] = updater->revolute[v]?(Math::radToDeg(link.minLimit())):link.minLimit();
      updater->maxLimit[v] = updater->revolute[v]?(Math::radToDeg(link.maxLimit())):link.maxLimit();
    }
    for(Int v=0; v<tchain.dof(); v++) {
      const KinematicChain::Link& link( tchain.linkOfVariable(v) );
      updater->trevolute[v] = (link.type() == KinematicChain::Link::Revolute);
      updater->tminLimit[v] = updater->trevolute[v]?(Math::radToDeg(link.minLimit())):link.minLimit();
      updater->tmaxLimit[v] = updater->trevolute[v]?(Math::radToDeg(link.maxLimit())):link.maxLimit();
    }

    viewer->getViewportSceneView(0)->setUpdateVisitor(updater);

    createControls();

    camera->setLookAt(osg::Vec3(0,0,0), osg::Vec3(15,15,15), osg::Vec3(0,1,0));
    camera->setNearFar(0.5,16000.0);

    viewer->run();
    Debugln(Tmp,"Viewer quit.");
    delete controlWindow;
    delete viewer;
    // hack to wait until all windows are properly closed before exiting
    for(Int i=0; i<100; i++)
      Glow::MainLoop();


  } catch (std::exception& e) {
    Consoleln("caught: " << String(e.what()));
    abort();
  }


  base::MemoryTracer::dumpNamed();
  Consoleln("Exiting.");
  
  return 0;
}

