#include <base/base>
#include <robot/robot>
#include <robot/sim/sim>

#include <base/Application>
#include <base/Universe>
#include <base/Math>
#include <base/Dimension3>
#include <gfx/TrackballManipulator>

using base::Application;
using base::Universe;
using base::Vector3;
using base::Matrix4;
using base::Dimension3;
using base::Point3;
using base::Orient;


#include <robot/RobotDescription>
#include <robot/sim/SimulatedBasicEnvironment>

using robot::RobotDescription;
using robot::PlatformDescription;
using robot::ManipulatorDescription;
using robot::KinematicChain;
using robot::sim::SimulatedBasicEnvironment;


// Producer / OSG for visualization
#include <osgProducer/Viewer>



int main(int argc, char *argv[])
{ 
  try {

    // Assume that the user has define an OPENSIM_HOME environment variable
    // that points to the top of their OpenSim installation
    // We'll assume that the resource and cache directories are relative
    //  to that

    char *homeenv = getenv("OPENSIM_HOME");
    Assertm(homeenv!=0, "environment variable OPENSIM_HOME defined");
    String home(homeenv);

    // create singleton app
    Application app(home+"/resources",home+"/cache");
    app.displayHeader("MySim");
    
    // make a universe in which everything is simulated
    ref<Universe> universe = app.universe();

    // make use of the robot::sim::SimulatedBasicEnvironment to provide a simple environment
    //  (with a ground, gravity etc.) in which to simulate our robots
    ref<SimulatedBasicEnvironment> env(NewObj SimulatedBasicEnvironment(universe->filesystem(),
                                                                        universe->cache()
                                                                       )
                                      );

    ref<base::VFile> envFile( universe->cache()->findFile(String("data/sim_tut2_env.xml")) );
    env->load(envFile, "xml");

    
    // now tell the universe about our environment
    universe->addWorld(env);         // tell universe to visualize it
    universe->addSimulatable(env);   // tell universe to simulate it
    
    
    // 
    // Next, if we want to visualize the simulation in 3D we need to setup
    //  a window to render into using OpenProducer / OSG
    // construct the viewer.
    //  (this is fairly boiler-plate code - refer to the Producer & OSG docs)
    Producer::RenderSurface *rs = new Producer::RenderSurface;
    rs->setWindowName("MySim");
    rs->setWindowRectangle(0,400,800,600); // set window pos/size
    Producer::Camera *camera = new Producer::Camera; // a camera
    camera->setRenderSurface(rs);
    Producer::CameraConfig *cfg = new Producer::CameraConfig;
    cfg->addCamera("Camera",camera);
    
    osgProducer::Viewer viewer(cfg);
                                                                                                                                                                                                                                            
    // set up the viewer with sensible default event handlers.
    viewer.setUpViewer(osgProducer::Viewer::STATE_MANIPULATOR |
                       osgProducer::Viewer::HEAD_LIGHT_SOURCE |
                       osgProducer::Viewer::STATS_MANIPULATOR |
                       osgProducer::Viewer::VIEWER_MANIPULATOR |
                       osgProducer::Viewer::ESCAPE_SETS_DONE );

    // a camera manipulator to allow us to move the view around via the mouse
    gfx::TrackballManipulator* cm = new gfx::TrackballManipulator();
    viewer.selectCameraManipulator( viewer.addCameraManipulator(cm) ); 
    
    
    // Now, ask the environment to create a scene (an OSG Visual)
    osg::Group* sceneRoot = NewObj osg::Group;       // an empty OSG group node
    sceneRoot->addChild( env->createOSGVisual() );   //  now with the while env visualization as a child
    // and tell the viewer of our scene
    viewer.setSceneData(sceneRoot);
    
    // open the window
    viewer.realize();
    
    // move the camera to somewhere sensible so we can see something
    cm->setModelScale(2);
    cm->computePosition(osg::Vec3(9,2,6), osg::Vec3(0,2,0), osg::Vec3(0,0,1)); // eye, center, up
    
    
    
    // Finally, we need a main loop
    //  here we step the simulation and update the view
    
    universe->preSimulate(); // initialization step
    
    // this simple main loop doesn't have any frame-rate control or other waiting,
    //  so it will just render frames as fast as possible (100% CPU utilization!)
    while(!viewer.done()) {

      viewer.sync(); // wait for draw/cull/update traversals to finish before changing the world state
      
      // step the simulation
      universe->simulateForSimTime(1.0/100.0); // for 100th of a sec (simulation time, not real-time)
      
      // update the scene by traversing it 
      viewer.update();
      
      // fire off the traversals (e.g. draw)
      viewer.frame();
      
    } // end main loop (when ESC hit)
    
    
    viewer.sync();

  } catch (std::exception& e) {
    Consoleln("caught: " << String(e.what())); // display error info
  }
  
  Consoleln("Exiting.");
  
  return 0;
}
