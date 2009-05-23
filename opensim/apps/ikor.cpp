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
  
  $Id: ikor.cpp 1028 2004-02-11 20:45:27Z jungd $
  $Revision: 1.32 $
  $Date: 2004-02-11 15:45:27 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <iostream>
#include <iomanip>

#include <robot/robot>

#include <base/ref>
#include <base/Application>
#include <base/Universe>
#include <base/Math>
#include <base/Time>
#include <base/File>
#include <base/Trajectory>


#include <gfx/LookAtCameraManipulator>
#include <gfx/Color4>

#include <robot/Controllable>
#include <robot/Robot>
#include <robot/RobotController>
#include <robot/control/ControllableAdaptor>
#include <robot/control/ManipulatorPIDPositionController>
#include <robot/control/kinematics/FullSpaceSolver>
#include <robot/control/kinematics/LeastNormIKSolver>
#include <robot/control/kinematics/IKORController>
#include <robot/sim/SimulatedBasicEnvironment>

using base::Application;
using base::Universe;
using base::Math;
using base::Time;
using base::PathName;
using base::VFile;
using base::Point3;
using base::Vector3;
using base::Quat4;
using base::Path;
using base::zeroVector;
using base::ref;
using gfx::LookAtCameraManipulator;
using gfx::Color4;
using robot::ControlInterface;
using robot::Controllable;
using robot::Robot;
using robot::RobotDescription;
using robot::ManipulatorDescription;
using robot::control::ControllableAdaptor;
using robot::control::ManipulatorPIDPositionController;
using robot::control::kinematics::InverseKinematicsSolver;
using robot::control::kinematics::LeastNormIKSolver;
using robot::control::kinematics::FullSpaceSolver;
using robot::control::kinematics::IKORController;
using robot::sim::SimulatedBasicEnvironment;


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





//
// temp includes for testing
//
#include <base/Vector>
#include <base/Matrix>
#include <base/Orient>
#include <base/SVD>
#include <base/BinarySerializer>
#include <base/SimpleXMLSerializer>
#include <base/SinExpression>
#include <physics/Box>
#include <physics/VisualDebugUtil>
#include <robot/ControlInterface>
#include <robot/Robot>
#include <robot/control/kinematics/IKOR>
#include <robot/ManipulatorDescription>

using base::ref;
using base::narrow_cast_ref;
using base::Serializer;
using base::Serializable;
using base::ExpressionNode;
using base::SinExpression;
using base::BinarySerializer;
using base::SimpleXMLSerializer;
using base::Vector;
using base::Orient;
using base::Range;
using base::Matrix;
using base::ExpressionVector;
using base::ExpressionMatrix;
using base::SVD;
using base::operator*;
using physics::Box;
using physics::VisualDebugUtil;


using robot::operator<<;



static VisualDebugUtil visualDebugUtil;





// GUI classes

#include "glow.h"
#include "glowQuickPalette.h"

#include "glowLabelWidget.h"
#include "glowPushButtonWidget.h"
#include "glowSliderWidget.h"
#include "glowAux.h"

using glow::Glow;
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
typedef struct {
  GlowSliderWidget* targetSlider[6];  
  GlowLabelWidget* err;
} RobotControls;

base::array<RobotControls> robotControls;
GlowPushButtonWidget* quitButton;

char posBuf[30];


const bool orientationControl = false;



//inline Vector makeVector(Real x, Real y, Real z, Real rl, Real p, Real yw)
//{ Vector r(6); r[0]=x; r[1]=y; r[2]=z; r[3]=rl; r[4]=p; r[5]=yw; return r; }



// helper class: OSG calls reset() before each App traversal of the scene (once before
//  each 'frame')
class Updater : public osg::NodeVisitor 
{
public:
  Updater(base::ref<base::Universe> u, osg::Camera* camera, Int numManips,
          base::array< base::ref<robot::Controllable> > controllables) 
    : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN), u(u), camera(camera), controllables(controllables),
      dynamic(false), freeze(false), count(0), numManips(numManips)
  {
    target.resize(numManips);
    eeposinterface.resize(numManips);
    N.resize(numManips);
    linkpositionsinterface.resize(numManips);
    
    for(Int r=0; r<numManips;r++) {
      target[r].resize(6);
      eeposinterface[r] = controllables[r]->getControlInterface();
      N[r] = eeposinterface[r]->outputSize();
      linkpositionsinterface[r] = controllables[r]->getControlInterface("manipulatorLinkPositions");
      setTarget( r, eeposinterface[r]->getInputs() );
    }
  }


  virtual ~Updater() {
  }



  array<Int> N;

  void setTarget(Int r, const Vector& t) {
    if (orientationControl)
      target[r] = t;
    else {
      target[r][0] = t[0]; target[r][1]=t[1]; target[r][2]=t[2];
    }

    robotControls[r].targetSlider[0]->SetValue(t[0]);
    robotControls[r].targetSlider[1]->SetValue(t[1]);
    robotControls[r].targetSlider[2]->SetValue(t[2]);
    if (orientationControl) {
      robotControls[r].targetSlider[3]->SetValue(Math::radToDeg(t[3]));
      robotControls[r].targetSlider[4]->SetValue(Math::radToDeg(t[4]));
      robotControls[r].targetSlider[5]->SetValue(Math::radToDeg(t[5]));
    }
  }


  virtual void reset()
  {
    using base::ref;
    using base::Vector;
    using base::Range;

    const Real frameTime = 1/60.0;
    const Real stepTime = 1/60.0;

    Time startTime(Time::now());

    if (!freeze) {
      //
      // Controls

      for(Int r=0; r<numManips; r++) {
      
        Vector ee( eeposinterface[r]->getInputs() );
  
        if (true) {
          // calc delta between current pos and target pos
          Vector dx(ee.size());
  
          dx = target[r] - ee;
          if (!orientationControl) dx = vectorRange(dx,Range(0,3));
  
          Real error = norm_2(dx);
          if (count%30==0) {
            sprintf(posBuf,"(%5.3f)    ",error);
            //robotControls[r].err->SetText(posBuf);
          }
  
  
          if (dynamic) {
            // limit the dx to help instability
            if (Math::abs(dx[0]) > 0.15) dx[0]=Math::sign(dx[0])*0.15;
            if (Math::abs(dx[1]) > 0.15) dx[1]=Math::sign(dx[1])*0.15;
            if (Math::abs(dx[2]) > 0.15) dx[2]=Math::sign(dx[2])*0.15;
            
            if (orientationControl) {
              if (Math::abs(dx[3]) > 3) dx[3]=Math::sign(dx[3])*3;
              if (Math::abs(dx[4]) > 3) dx[4]=Math::sign(dx[4])*3;
              if (Math::abs(dx[5]) > 3) dx[5]=Math::sign(dx[5])*3;
            }
          }
  
          for(Int j=0; j<(orientationControl?6:3); j++)
            eeposinterface[r]->setOutput(j,dx[j]);
          
        }
        else
          freeze=true;
      }
      

      
      // iterate controllers 
      base::reflist<robot::Controller>::iterator c(controllers.begin());
      base::reflist<robot::Controller>::iterator end(controllers.end());
      while (c != end) {
	base::ref<robot::Controller> controller(*c);
        try {	
          controller->iterate(simTime); 
        } catch (std::exception& e) {} // ignore problems
	++c;
      }
      
      
      Color4 col("lime green",0.2);
      VisualDebugUtil::setColorAll(col);

      //
      // Simulation 
      
      Int iterations = Int(frameTime / stepTime);
      
      for(Int s=0; s<iterations; s++)
	u->simulateForSimTime(stepTime); // step the simulation of the universe
      simTime += Time(stepTime*iterations);
      
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
  base::array< base::ref<robot::Controllable> > controllables;
  base::array< base::ref<robot::ControlInterface> > eeposinterface;
  base::array< base::ref<robot::ControlInterface> > linkpositionsinterface;
  base::reflist<robot::Controller> controllers; // Controllers to iterate()
  base::array<base::Vector> target;
  bool dynamic;
  bool freeze;
  Int count;
  base::Vector lastlinks;
  Int numManips;
};

Updater* updater;

base::array<String> manipNames;





class EventReceiver : public GlowPushButtonReceiver,
		      public GlowSliderReceiver
{
public:
  
  virtual void OnMessage(const GlowPushButtonMessage& message) 
  {
    if (message.widget == quitButton)
      Glow::exitMainLoop();
  }
  
  virtual void OnMessage(const GlowSliderMessage& message) 
  {
    for(Int r=0; r<updater->numManips;r++) {
      if (message.widget == robotControls[r].targetSlider[0])
        updater->target[r][0] = message.value;
      else if (message.widget == robotControls[r].targetSlider[1])
        updater->target[r][1] = message.value;
      else if (message.widget == robotControls[r].targetSlider[2])
        updater->target[r][2] = message.value;
  
      if (orientationControl) {
        if (message.widget == robotControls[r].targetSlider[3])
          updater->target[r][3] = Math::degToRad(message.value);
        else if (message.widget == robotControls[r].targetSlider[4])
          updater->target[r][4] = Math::degToRad(message.value);
        else if (message.widget == robotControls[r].targetSlider[5])
          updater->target[r][5] = Math::degToRad(message.value);
      }
    }
  }

};


static void createControls(Int numManips)
{
  robotControls.resize(numManips);
  
  
  char title[64];
  strncpy(title,(String("OpenSim ") + Application::getVersion() + " - Controls").c_str(),64);
  controlWindow = NewObj GlowQuickPaletteWindow(title);

  EventReceiver* rec = NewObj EventReceiver();

  
  for (Int m=0; m<numManips; m++) {
  
    GlowQuickPanelWidget* tpanel = controlWindow->AddPanel(GlowQuickPanelWidget::etchedStyle, (manipNames[m]+" target position").c_str() );

    GlowSliderWidget* slider = tpanel->AddSlider(-5, 5, 0,
  	  	    GlowSliderWidget::defaultOptions, 7, "%5.2f", "x:%5.2f ", rec);
    slider->Reshape(400, slider->Height());
    robotControls[m].targetSlider[0] = slider;

    robotControls[m].targetSlider[1] = tpanel->AddSlider(-5, 5, 0,
		    GlowSliderWidget::defaultOptions, 7, "%5.2f", "y:%5.2f ", rec);
    robotControls[m].targetSlider[2] = tpanel->AddSlider(-3, 3, 0,
		    GlowSliderWidget::defaultOptions, 7, "%5.2f", "z:%5.2f ", rec);

    if (orientationControl) {
      robotControls[m].targetSlider[3] = tpanel->AddSlider(-180, 180, 0,
				  	  GlowSliderWidget::defaultOptions, 5, "%4.0f", "R:%4.0f ", rec);
      robotControls[m].targetSlider[4] = tpanel->AddSlider(-180, 180, 0,
					  GlowSliderWidget::defaultOptions, 5, "%4.0f", "P:%4.0f ", rec);
      robotControls[m].targetSlider[5] = tpanel->AddSlider(-180, 180, 0,
					  GlowSliderWidget::defaultOptions, 5, "%4.0f", "Y:%4.0f ", rec);
    }

    //GlowQuickPanelWidget* epanel = controlWindow->AddPanel(GlowQuickPanelWidget::etchedStyle, "End-Effector Position Error");

    //robotControls[r].err = epanel->AddLabel("(0.000)  ");
    //robotControls[r].err->SetFont(GlowFont::fixed8by13);

  } // for each robot
  

  quitButton = controlWindow->AddPushButton("Quit", rec);

  controlWindow->Pack();

}


static void deleteControls()
{
  robotControls.clear();
  delete controlWindow;
}













int main(int argc, char *argv[])
{
  setDebugOutputID(None); 
  //addDebugOutputID(Tmp);
  //addDebugOutputID(DJ);
  //addDebugOutputID(Ser);
  //addDebugOutputID(IKOR);
  //addDebugOutputID(FSP);
  abortOnAssertionFailureEnabled(true);

  try {
    char *homeenv = getenv("OPENSIM_HOME");
    Assertm(homeenv!=0, "environment variable OPENSIM_HOME defined");
    String home(homeenv);

    Application app(home+"/resources",home+"/cache");
    ref<Universe> universe = app.universe();

    std::cout << std::setiosflags(std::ios_base::fixed) << std::setprecision(2) << std::setw(6);



    // parse command-line args
    String usage("\nUsage: ikor [-static] <env_spec_file>");
    PathName envSpecFileName("defaultenv.xml");
    bool dynamic = true;
    SInt arg=1;
    while (arg < argc) {
      if (String(argv[arg]) == "-static") {
	dynamic = false;
	++arg;
      }
      else { 
        if (arg < argc) {
          envSpecFileName = String(argv[arg]);
          ++arg;
        }
        else {
  	  Consoleln(usage);
	  return -1;
        }
      }
    }
    
    if (envSpecFileName.isRelative()) 
      if (!universe->filesystem()->exists(envSpecFileName))
        envSpecFileName.prepend(String("data/"));

    Consoleln("");

    // create a simple simulation environment for the robots
    ref<SimulatedBasicEnvironment> env(NewObj SimulatedBasicEnvironment(app.filesystem(), universe->cache(), "Environment",dynamic));
    ref<VFile> envFile( universe->cache()->findFile(envSpecFileName) );
    env->load(envFile,envFile->extension());
    Consoleln("Loaded environment from file '" << envFile->pathName().str() << "'.");


    array< ref<ManipulatorPIDPositionController> > posController; // only if dynamic
    array< ref<IKORController> > ikorControllers;    
    array< ref<Controllable> >   controllables;
    
    Int numManips = 0;
    // for each robot in the env
    for(Int r=0; r<env->numRobots(); r++) {
    
      ref<Robot> robot = env->getRobot(r);
      if (robot->isDescriptionProvided())
        Consoleln("Robot: " << robot->getRobotDescription()->getName());
      
      // get manipulators to control
      array<ref<const ManipulatorDescription> > manipDescs(robot->getRobotDescription()->manipulators());
      
      // for each manip of the robot
      for(Int m=0; m<manipDescs.size(); m++) {
        ref<const ManipulatorDescription> manipulator(manipDescs[m]);
        manipNames.at(numManips) = manipulator->getName();
      
        ref<ControlInterface> posControlInterface;

        if (dynamic) {
          // create a PID controller to control the manipulator joint positions
          posController.at(r) = ref<ManipulatorPIDPositionController>(NewObj ManipulatorPIDPositionController(manipulator->getKinematicChain()));
          posController[r]->setCoeffs(12,0,2); // PID Kp, Ki, Kd
          posController[r]->setControlInterface(robot->getControlInterface("manipulatorVelocity1")); // connect posController to manipulator interface
          posControlInterface = posController[r]->getControlInterface();
        }
        else {
          // static simulations can be directly controlled via position
          posControlInterface = robot->getControlInterface("manipulatorPosition1");
        }
    
  
     
        // create IK controller
//        ref<IKORController> ikorController(NewObj IKORController(IKORController::LeastNorm,
        ref<IKORController> ikorController(NewObj IKORController(IKORController::FSPLagrangian,
                                                               robot, 0, false, 
                                                               orientationControl, Orient::EulerRPY));
                                                               
        ikorController->setControlInterface(posControlInterface); // connect to manipulator position control interface
        ikorControllers.push_back(ikorController);
        controllables.push_back(ikorController);

        numManips++;
     } // end for each manip 
      
    } // end for each robot

    
    // OSG visual test
    Glow::Init(argc, argv);

    
    // Setup OSG visual 
    osg::Group* rootnode = NewObj osg::Group;
    rootnode->setName("root");

    rootnode->addChild( env->createOSGVisual() );    
    
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
    

    createControls(numManips);

    updater = NewObj Updater(universe,camera,numManips,controllables);
    updater->dynamic = dynamic;
    for(Int m=0; m<numManips; m++) {
      if (dynamic) 
        updater->controllers.push_back(posController[m]);
      updater->controllers.push_back(ikorControllers[m]);
    }
    //updater->target = target;

    Assert(updater);
    viewer->getViewportSceneView(0)->setUpdateVisitor(updater);


    camera->setLookAt(osg::Vec3(0,0,0), osg::Vec3(15,15,15), osg::Vec3(0,1,0));
    camera->setNearFar(0.5,16000.0);

    viewer->run();
    
    Debugln(Tmp,"Viewer quit.");
    // hack to wait until all windows are properly closed before exiting
    for(Int i=0; i<100; i++)
      Glow::MainLoop();

    deleteControls();
    delete viewer;

  } catch (std::exception& e) {
    Consoleln("caught: " << String(e.what()));
  }


  base::MemoryTracer::dumpNamed();
  Consoleln("Exiting.");
  
  return 0;
}


