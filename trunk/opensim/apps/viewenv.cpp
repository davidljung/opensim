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

  $Id: viewenv.cpp 1131 2004-09-28 20:50:35Z jungd $

****************************************************************************/

#include <gtkmm.h>

#include <base/base>
#include <robot/robot>

#include <base/Math>
#include <base/Time>
#include <base/Orient>
#include <base/Application>
#include <base/Universe>
#include <base/PathName>
#include <base/VFile>

#include <gfx/LookAtCameraManipulator>
#include <gfx/TrackballManipulator>

#include <physics/VisualDebugUtil>

#include <robot/Robot>
#include <robot/RobotDescription>
#include <robot/ControlInterface>
#include <robot/sim/SimulatedBasicEnvironment>

#include <robot/control/kinematics/IKORController>

#include <apps/ViewEnvWindow>

using base::Math;
using base::Time;
using base::Orient;
using base::Application;
using base::Universe;
using base::PathName;
using base::VFile;

using physics::VisualDebugUtil;

using robot::Robot;
using robot::RobotDescription;
using robot::PlatformDescription;
using robot::ManipulatorDescription;
using robot::ControlInterface;
using robot::sim::SimulatedBasicEnvironment;
using robot::sim::BasicEnvironment;
using robot::control::kinematics::IKORController;


// STL
#include <list>




// OSG
#include <osgProducer/Viewer>
#include <osgDB/WriteFile>



static VisualDebugUtil visualDebugUtil;


// CameraPostDrawCallback for snaping a screenshot of the frame
class SnapImageDrawCallback : public Producer::Camera::Callback
{
  public:
    SnapImageDrawCallback()
      : _snapImageOnNextFrame(false)
    {
    }

    void setFilename(const String& filename) { _filename = filename; }

    void setSnapImageOnNextFrame(bool flag) { _snapImageOnNextFrame = flag; }
    bool getSnapImageOnNextFrame() const { return _snapImageOnNextFrame; }

    virtual void operator()( const Producer::Camera & camera)
    {
        if (!_snapImageOnNextFrame) return;

        SInt x,y;
        Int width,height;
        camera.getProjectionRectangle(x,y,width,height);

        osg::ref_ptr<osg::Image> image = new osg::Image;
        image->readPixels(x,y,width,height,
                          GL_RGB,GL_UNSIGNED_BYTE);
        if (osgDB::writeImageFile(*image,_filename))
        {
            Logln("Saved frame image to `"<< _filename <<"`");
        }

        _snapImageOnNextFrame = false;
    }

protected:

    String _filename;
    bool   _snapImageOnNextFrame;
};






using namespace apps;

int main(int argc, char *argv[])
{
  setDebugOutputID(None);

  Gtk::Main kit(argc, argv);
  ViewEnvWindow gui(kit);

  try {
    char *homeenv = getenv("OPENSIM_HOME");
    Assertm(homeenv!=0, "environment variable OPENSIM_HOME defined");
    String home(homeenv);

    Application app(home+"/resources",home+"/cache");
    app.displayHeader("ViewEnv");
    ref<Universe> universe = app.universe();

    std::cout << std::setiosflags(std::ios_base::fixed) << std::setprecision(2) << std::setw(6);

    // parse command-line args
    String usage("\nUsage: viewenv [-env <env_spec_file>] [-static] [-debug | -nodebug] [-debugoutput | -nodebugoutput] [-debuggfx | -nodebuggfx]");
    PathName envSpecFileName("defaultenv.xml");
    bool dynamic=true;
    bool debugoutput=true;
    bool debuggfx=true;
    SInt arg=1;
    while (arg < argc) {

      if (String(argv[arg]) == "-env") {
        envSpecFileName = String(argv[++arg]);
        ++arg;
      }
      else if (String(argv[arg]) == "-static") {
        dynamic = false;
        ++arg;
      }
      else if (String(argv[arg]) == "-debug") {
        debugoutput = debuggfx = true;
        ++arg;
      }
      else if (String(argv[arg]) == "-nodebug") {
        debugoutput = debuggfx = false;
        ++arg;
      }
      else if (String(argv[arg]) == "-debugoutput") {
        debugoutput = true;
        ++arg;
      }
      else if (String(argv[arg]) == "-nodebugoutput") {
        debugoutput = false;
        ++arg;
      }
      else if (String(argv[arg]) == "-debuggfx") {
        debuggfx = true;
        ++arg;
      }
      else if (String(argv[arg]) == "-nodebuggfx") {
        debuggfx = false;
        ++arg;
      }
      else {
        Consoleln(usage);
        return -1;
      }

    }

    if (debugoutput) {
      //setDebugOutputID(JFKE);
      //setDebugOutputID(Tmp);
      //addDebugOutputID(DJ);
      abortOnAssertionFailureEnabled(true);
      //abortOnExceptionConstructionEnabled(true);
    }

    // save the user pre-pending filenames with data/
    if (envSpecFileName.isRelative()) {
      if (!universe->filesystem()->exists(envSpecFileName))
        envSpecFileName.prepend("data/");
    }

    Consoleln("");

    // create a simple simulation environment for the robot
    ref<SimulatedBasicEnvironment> env(NewObj SimulatedBasicEnvironment(app.filesystem(), universe->cache(), "Environment",dynamic));
    ref<VFile> envFile( universe->cache()->findFile(envSpecFileName) );
    env->load(envFile,envFile->extension());
    Consoleln("Loaded environment from file '" << envFile->pathName().str() << "'.");

    // show GUI
    gui.init(env);
    gui.show();

    // Setup OSG visual & Producer viewer

    // construct the viewer.
    Producer::RenderSurface *rs = new Producer::RenderSurface;
    rs->setWindowName( (Application::getLongName()+" "+Application::getVersion()+" - view environment - Simulation").c_str() );
    rs->setWindowRectangle(0,400,800,600);
    Producer::Camera *camera = new Producer::Camera;
    camera->setRenderSurface(rs);
    SnapImageDrawCallback snapImageDrawCallback;
    camera->addPostDrawCallback(&snapImageDrawCallback);
    Producer::InputArea *ia = new Producer::InputArea;
    ia->addRenderSurface(rs);
    Producer::CameraConfig *cfg = new Producer::CameraConfig;
    cfg->addCamera("Camera",camera);
    cfg->setInputArea(ia);
    osgProducer::Viewer viewer(cfg);

    // set up the viewer with sensible default event handlers.
    viewer.setUpViewer(osgProducer::Viewer::STATE_MANIPULATOR |
                       osgProducer::Viewer::HEAD_LIGHT_SOURCE |
                       osgProducer::Viewer::STATS_MANIPULATOR |
                       osgProducer::Viewer::VIEWER_MANIPULATOR |
                       osgProducer::Viewer::ESCAPE_SETS_DONE );


    gfx::TrackballManipulator* cm = new gfx::TrackballManipulator();
    //gfx::LookAtCameraManipulator lookAtCameraManipulator(-250,13.5,8,
    //                                                -0.18,0.1,0.7);
    viewer.selectCameraManipulator( viewer.addCameraManipulator(cm) );

    // generate scene from environment
    osg::Group* rootnode = NewObj osg::Group;
    rootnode->setName("root");

    rootnode->addChild( env->createOSGVisual(/*debuggfx?gfx::Visual::ShowAxes:0*/) );
    // show debugging graphics
    if (debuggfx)
      rootnode->addChild( visualDebugUtil.createOSGVisual(/*gfx::Visual::ShowAxes*/) );

    // add a viewport to the viewer and attach the scene graph.
    viewer.setSceneData(rootnode);

    // initialize simulation
    base::Time simTime;
    universe->addWorld(env);
    universe->addSimulatable(env);
    universe->enableSimulationRendering(false);
    universe->preSimulate();

    // create the windows and run the threads.
    viewer.realize();

    // adjust camera
    cm->setModelScale(2);
    cm->computePosition(osg::Vec3(2,2,2), osg::Vec3(0,0,0), osg::Vec3(0,0,1)); // eye, center, up

    // for saving the frame image to a file
    SInt cameraX, cameraY;
    Int cameraWidth, cameraHeight;
    camera->getProjectionRectangle(cameraX,cameraY,cameraWidth,cameraHeight);
    Int frameNo = 0;

    // Main loop

    const Real frameTime = 1/30.0; // target render freq.
    Real stepTime = gui.getSimStepSize(); // simulation speed (step size in seconds)
    Int frame = 0;
    Int simStepPeriod = 1; // step simulation every n frames (adaptive to maintain framerate)
    Int noAdjustCount = 100; // don't adjust simStepPeroid until 0

    // do an initial frame (so something is displayed during the first simulation step)
    viewer.frame();
    base::Time lastFrameTime = base::Time::now();


    while(!gui.quit && !viewer.done()) { // while neither 3D view or GUI signals quit
      if (kit.events_pending())
        kit.iteration();
      else {

        // handle updates due to GUI events
        gui.update(simTime);

        // wait for all cull and draw threads to complete.
        viewer.sync();

        // step simulation
        stepTime = gui.getSimStepSize();
        if ((frame % simStepPeriod) == 0) {

          if (debuggfx) {
            // for collision debugging.  Set all debug shapes to green,
            //  then interpenetrations are marked in red etc.
            gfx::Color4 col("lime green",0.15);
            VisualDebugUtil::setColorAll(col);
          }

          universe->simulateForSimTime(stepTime);
          simTime += base::Time(stepTime);

          if (debuggfx)
            VisualDebugUtil::updateVisual();

          // trigger save of frame to file (if requested)
          if (gui.saveFrames()) {
            String filename("/home/jungd/local/frames/viewenv_frame0000.rgb");
            String num = base::intToString(frameNo++);
            for(Int i=0; i<num.length(); i++)
              filename[filename.length()-4-num.length()+i] = num[i];
            snapImageDrawCallback.setFilename(filename);
            snapImageDrawCallback.setSnapImageOnNextFrame(true);
          }
        }


        // update the scene by traversing it with the the update visitor which will
        // call all node update callbacks and animations.
        viewer.update();

        // fire off the cull and draw traversals of the scene.
        viewer.frame();
        if (++frame == consts::maxInt) frame = 0; // count frame (& wrap if necessary)
        if (noAdjustCount > 0) --noAdjustCount;

        // maintain reasonable frame-rate (not unnecessarily fast or too slow)
        base::Time elapsed(base::Time::now() - lastFrameTime);
        gui.setFrameRate( 1.0/elapsed.seconds() );
        gui.setSimSpeed( (stepTime/simStepPeriod) / elapsed.seconds() );

        Real frameRateToTargetRatio = gui.getFrameRate() / (1.0/frameTime);

        if (frameRateToTargetRatio > 2) {
          // rendering too fast
          if ( (noAdjustCount==0) && (simStepPeriod > 1)) {
            --simStepPeriod; // can afford to simulate faster
            noAdjustCount = 200;
//Logfln("ratio=" << frameRateToTargetRatio);
//Logfln("speeding sim period to " << simStepPeriod);
          }

          // wait a bit too
          Real sleepFor = (frameTime - elapsed.seconds());
          base::Time::sleep(sleepFor);
        }
        else if (frameRateToTargetRatio < 0.8) {
          // rendering too slow, reduce the simulation pace
          if (noAdjustCount==0) {
            ++simStepPeriod;
            noAdjustCount = 200;
//Logfln("ratio=" << frameRateToTargetRatio);
//Logfln("slowing sim period to " << simStepPeriod);
          }
        }

        lastFrameTime = base::Time::now();

      }
    } // end main loop

    // wait for all cull and draw threads to complete before exit.
    viewer.sync();

  } catch (std::exception& e) {
    Consoleln("caught: " << String(e.what()));
  }


  gui.hide();
  Consoleln("Exiting.");

  return 0;
}



