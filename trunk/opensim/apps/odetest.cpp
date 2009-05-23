#include <physics/physics>

#include <base/MemoryTracer>
#include <base/Application>
#include <base/VFileSystem>
#include <base/Cache>
#include <base/BinarySerializer>
#include <base/MD5>

#include <base/Point2>
#include <base/Point3>
#include <base/Vector3>

#include <gfx/LookAtCameraManipulator>

#include <osg/Group>
#include <osg/Notify>
#include <osg/Camera>
#include <osg/Fog>
#include <osgUtil/SceneView>
#include <osgGLOW/Viewer>

using osg::Node;
using osg::Vec3;
using osgGLOW::Viewer;


#include <gfx/Triangle3>
#include <gfx/VisualTriangles>
#include <gfx/CLODTerrainDrawable>
#include <gfx/CLODTerrainRenderer>

#include <physics/MassProperties>
#include <physics/HeightField>
#include <physics/Material>
#include <physics/Box>
#include <physics/Torus>
#include <physics/Sphere>
#include <physics/Cylinder>
#include <physics/LODTerrain>
#include <physics/Polyhedron>
#include <physics/Solid>
#include <physics/SolidSystem>
#include <physics/ODESolidSystem>

#include <stdexcept>
#include <stdio.h>

#include <iostream>

using base::ref;
using base::Point2;
using base::Point3;
using base::Vector3;
using base::Quat4;
using base::Matrix4;
using base::Universe;
using base::Application;
using base::PathName;
using base::VFileSystem;
using base::VFile;
using base::VDirectory;
using base::VEntry;
using base::Cache;
using base::Serializer;
using base::BinarySerializer;
using base::narrow_cast_ref;
using base::dynamic_cast_ref;

using gfx::LookAtCameraManipulator;
using gfx::CLODTerrainDrawable;
using gfx::CLODTerrainRenderer;
using gfx::CLODTerrainSettings;


using physics::Box;
using physics::Torus;
using physics::Sphere;
using physics::Cylinder;
using physics::LODTerrain;
using physics::Polyhedron;
using physics::Material;
using physics::HeightField;
using physics::MassProperties;
using physics::Solid;
using physics::SolidSystem;
using physics::ODESolidSystem;
using physics::ConstraintGroup;
using physics::BallJoint;
using physics::HingeJoint;
using physics::DoubleHingeJoint;
using physics::SliderJoint;
using physics::UniversalJoint;
using physics::Motor;


//#include <physics/OBBCollisionDetector>
//#include <physics/OBBCollisionModel>
//#include <physics/GJKCollisionDetector>
//#include <physics/GJKCollisionModel>
#include <physics/ODECollisionDetector>
#include <physics/ODECollisionModel>
using physics::CollisionDetector;
using physics::CollisionModelProvider;
using physics::CollisionState;
//using physics::OBBCollisionDetector;
//using physics::OBBCollisionModel;
//using physics::GJKCollisionDetector;
//using physics::GJKCollisionModel;
using physics::ODECollisionDetector;
using physics::ODECollisionModel;

Real simtimestep = 1.0/20.0; 
ref<Universe> u;
osg::Camera* camera;
Real x,y,z;



void dumpSceneGraph(osg::Node* node, Int level);
/*
    // A forcer to give a body a 'kick'
    class Kicker : public DynamicBodySystemForcer {
    public:
      Kicker(ref<SolidSystem::DynamicBodyList& bodies,
	     DynamicBody& body, Vector3 force, Vector3 torque)
	: DynamicBodySystemForcer(bodies),
	body(body), force(force), torque(torque), 
	  done(false), kickit(false), applyit(false)
      {
      }
      
      void kickIt() { if (!done) kickit=true; }
      void applyIt() { applyit=true; }

      virtual void accumulateDynamicBodyForces() {
	if (kickit || applyit) {
	  kickit = false;
	  body.f += force;
	  body.torque += torque;
	  done = true;
	}
      }
      
      DynamicBody& body;
      Vector3 force, torque;
      bool done, kickit, applyit;
    };

Kicker* gkicker;
Kicker* gkicker2;
*/
class AppVisitor : public osg::NodeVisitor {
public:
  AppVisitor() : 
    osg::NodeVisitor(TRAVERSE_ALL_CHILDREN), count(0), kicked(false)
  {
  }

  virtual void reset()
  {
    //    u->simulateForRealTime(1.0/10.0,simtimestep);
    //    solidBox->setAngVelocity(base::Vector3(0,0.08,-0.15));
    try {
      u->simulateForSimTime(1.0/20.0);
    } catch (std::exception& e) {
      Consoleln("caught: " << String(e.what()));
    }

    if ((count++ > 3) && !kicked) {
      //      gkicker->kickIt();
      //      gkicker2->kickIt();
      //camera->setLookAt(Vec3(x-4.0,y-4.0,z+12.0), Vec3(x,y,z+10.0), Vec3(0,0,1));
      camera->setLookAt(Vec3(0,0,0), Vec3(25,25,25), Vec3(0,0,1));
      if (count > 3)
	kicked = true;
    }

  }

  Int count;
  bool kicked;
};

int main(int argc, char *argv[])
{
  setDebugOutputID(Tmp);
  addDebugOutputID(DJ);
  //addDebugOutputID(IKOR);
  abortOnAssertionFailureEnabled(true);

  try {
    char *homeenv = getenv("OPENSIM_HOME");
    Assertm(homeenv!=0, "environment variable OPENSIM_HOME defined");
    String home(homeenv);

    Application app(home+"/resources",home+"/cache");

    ref<VFileSystem> fs = app.filesystem();
    ref<Universe> u = app.universe();
    ::u = u;
    ref<Cache> cache = u->cache();


    // FileSystem test
    /*Debugln(Tmp,"CacheTest");
    ref<VDirectory> d = cache->getCache("data/mars.thf");
    ref<VFile> file;
    file = d->createFile("cache1.tst");
    */

    // output to it
    /*    const Int i = 65, i2=66;
    Int j = 0, j2=0;
    ref<Box> sbox = ref<Box>(NewObj Box(3,4,5));
    ref<Box> sbox2 = ref<Box>(NewObj Box(6,7,8));
    ref<Box> cboxin(0);
    String s("hello world.");
    String s2;
    const ref<Box>& cbox(sbox);
    BinarySerializer so(Serializer::Output,file);
    so(i)(i2)(sbox2)(sbox)(cbox)(s);
    file->close();
    BinarySerializer si(Serializer::Input,file);
    si(j)(j2)(sbox2)(sbox)(cboxin)(s2);
    file->close();
    Debugln(Tmp,"got j=" << j << " j2=" << j2);
    Debugln(Tmp,"got dims=" << sbox->dimensions());
    Debugln(Tmp,"got dims2=" << sbox2->dimensions());
    Debugln(Tmp,"got dimscin=" << cboxin->dimensions());
    Debugln(Tmp,s2);
    VDirectory::const_iterator f = d->begin();
    VDirectory::const_iterator end = d->end();
    while (f != end) {
      ref<VEntry> ent = *f;
      Debugln(Tmp,"File:" << ent->pathName());
      ++f;
    }
    Debugln(Tmp,"done.");
    */

    glutInit( &argc, argv );

    Viewer viewer;

    bool grav = (argc>2)?true:false;

    ref<physics::Material> mat(new physics::Material());
    ref<physics::Material> pmat(new physics::Material());
    pmat->setBaseColor(gfx::Color3(1,0.4,1));
    ref<physics::Material> ymat(new physics::Material());
    ymat->setBaseColor(gfx::Color3(1,1,0.3));
    ref<physics::Material> tmat(new physics::Material());
    tmat->setBaseColor(gfx::Color3(0.76,0.38,0));
    ref<physics::Material> bmat(new physics::Material());
    bmat->setBaseColor(gfx::Color3(0,0,1));

		
    ref<VFile> hff = cache->findFile(String("data/mars.thf"));
    ref<HeightField> heightfield( NewObj HeightField(hff));
    heightfield->scaleHeights(15.0);
    //    Terrain& terrainShape = f.newTerrain(heightfield);



    String modelfile("models/turtle.osg"); // default
    if (argc>1) modelfile=String("models/")+argv[1];
    ref<VFile> pfile = cache->findFile(modelfile);
    ref<Polyhedron> poly;
    ref<Solid> solidPoly;
    {
      //ref<Box> sbox = ref<Box>(NewObj Box(1,1,1));
      Debugln(Tmp,"loading polyhedron");
      poly = ref<Polyhedron>(new Polyhedron(pfile));
      //poly = ref<Polyhedron>(new Polyhedron(*sbox));

      // Test triangle extraction and re-use
      //      gfx::VisualTriangles ptris(*poly);
      //      ref<Polyhedron> poly2 = new Polyhedron(ptris);
      //      poly=poly2;

      poly->setIncludesAppearance(false);

      ////      solidPoly = system->createSolid(poly,ymat);
      // add to system first solidPoly->setMomentum(base::Vector3(0.5,-0.3,0.7)*0.01);
    }



    //ref<CollisionDetector> collisionDetector(NewNamedObj("GJKCollisionDetector") GJKCollisionDetector());
    ref<CollisionDetector> collisionDetector(NewNamedObj("ODECollisionDetector") ODECollisionDetector());
    //collisionDetector->collisionEnable(false);  // turn off collisions for now

    ref<SolidSystem> system(NewNamedObj("ODESolidSystem") ODESolidSystem());
    system->setCollisionDetector(collisionDetector);
	
    ref<Box> box(new Box(0.577,0.577,0.577));
    //ref<Torus> torus(new Torus(0.2,1.0));
    //ref<Sphere> sphere(new Sphere(0.5));
    ref<Solid> solid1 = system->createSolid(poly,ymat);
    solid1->setName("box");
    system->addSolid(solid1);
//      solid1->setMomentum(base::Vector3(0.01,-0.005,-0.02)*0.005);
//    solid1->setAngVelocity(base::Vector3(0,0.08,-0.15)*0.002);
		
    ref<Sphere> sphere2(new Sphere(0.5));
    ref<Box>    box2(new Box(1,1,0.5));
    ref<Polyhedron> poly2(new Polyhedron(*box2));
    ref<Solid> solid2 = system->createSolid(box2,pmat);
    solid2->setName("box2");
    system->addSolid(solid2);
    //    solid2->setAngVelocity(base::Vector3(0.1,0.01,-0.05)*20.0);

    solid1->setPosition(base::Point3(0,0,0));
    solid2->setPosition(base::Point3(8,0,0));
    solid2->setVelocity(base::Vector3(-0.05,0,0));



    ref<ConstraintGroup> cgroup = system->createConstraintGroup();
    system->addConstraintGroup(cgroup);

    //Debugfln(Tmp,"creating HingeJoint");
    //ref<HingeJoint> hinge = system->createHingeJoint();
    //cgroup->addConstraint(hinge);

    //Debugln(Tmp,"creating SliderJoint");
    //ref<SliderJoint> slider = system->createSliderJoint();
    //    cgroup->addConstraint(slider);


    //Debugln(Tmp,"attaching");
    //hinge->attach(solid1, solid2);
    //hinge->setAnchor(Point3(0.6,0.6,0.6));
    //hinge->setLowStop(-2*base::Pi/3);
    //hinge->setHighStop(2*base::Pi/3);
    //hinge->setStopRestitution(0.99);


    //Debugln(Tmp,"attaching");
    //    slider->attach(solid1, solid2);
    //    slider->setAnchor(Point3(0.5,0.5,0.5));
    //    slider->setAxis(Vector3(0,1,0));
    //    slider->setLowStop(0);
    //    slider->setHighStop(2);
    //    slider->setStopRestitution(0.8);

    //ref<Motor> motor = system->createMotor();
    //hinge->attachMotor(1, motor);
    //motor->setTargetVel(0.5);
    //motor->setMaxForce(1);
    







    CLODTerrainSettings::GetInstance()->SetMediaPath(const_cast<char*>((home+"/resources/textures").c_str()));

    ref<VFile> mapfile = cache->findFile(PathName("data/Llano.map"));
    ref<LODTerrain> terrain(NewNamedObj("LODTerrain(Llano.map)") LODTerrain());
    terrain->loadMap(mapfile);
    ////    ref<Solid> sterrain = system->createSolid(terrain, tmat);

    // Move solids to center of the terrain
    base::Dimension3 tdim = terrain->getDimension();
    x = tdim.x/2.0;
    y = tdim.y/2.0;
    z = terrain->getHeight(x,y);
    //solidBox->setPosition(base::Point3(x+10.0,y,z+8.0));
    //solidBox2->setPosition(base::Point3(x+10.0,y,z+13.0));
    //    solidBox2->state.x = base::Point3(x+12.0,y+3.0,z+10.0);
    //solidBox2->state.x = base::Point3(0,0,0);
    // add to system first solidPoly->setPosition(base::Point3(x+10.0,y+6.0,z+10.0));


    
    if (grav) 
      system->setGravity(base::Vector3(0,0,-9.8));
//    system->setDrag(0.01);
    
    osg::Node* g = system->createOSGVisual
			(
			 gfx::Visual::ShowAxes 
			 | gfx::Visual::ShowCollisionModel
			 | gfx::Visual::ShowCollisionDetection
			 //| gfx::Visual::ShowNormals
			 );

    osg::Group* group = new osg::Group;
    group->setName("root");
    osg::Group* rootnode = group;

    rootnode->addChild(g);

    // Set state of rootnode (fog etc.)
    osg::StateSet* pState = new osg::StateSet;
    //    pState->setMode(GL_LIGHTING,osg::StateAttribute::ON); 

    osg::Vec4 fogColor(0.65f,0.65f,0.65f,1.0f);
    osg::Fog* fog = new osg::Fog;
    fog->setMode(osg::Fog::LINEAR);
    fog->setDensity(0.1f);
    fog->setStart(100.0f);
    fog->setEnd(10000.0f - 100.0f); // (max view dist)
    fog->setColor(fogColor);
    pState->setAttributeAndModes(fog,osg::StateAttribute::ON);
    rootnode->setStateSet(pState);

    u->addSimulatable(system);
    u->enableSimulationRendering(false);
    u->preSimulate();


    viewer.addViewport( rootnode );
    viewer.setWindowTitle("OpenSim - multirobot physics simulator");

    // register our camera manipulator
    viewer.registerCameraManipulator(
	NewObj gfx::LookAtCameraManipulator(0,30,8,0,0,0));

    AppVisitor* appVisitor = new AppVisitor();
    viewer.getViewportSceneView(0)->setUpdateVisitor(appVisitor);

    // Open window so camera manipulator's warp pointer request will succeed
    viewer.open();

    viewer.selectCameraManipulator(0);

    osgUtil::SceneView* sceneView = viewer.getViewportSceneView(0);
    camera = sceneView->getCamera();
    //    camera->setLookAt(Vec3(15,15,15), Vec3(0,0,0), Vec3(0,1,0));
    camera->setLookAt(Vec3(0,0,0), Vec3(15,15,15), Vec3(0,1,0));
    camera->setNearFar(0.5,16000.0);
    //sceneView->setCalcNearFar(false);
    //    sceneView->setBackgroundColor(osg::Vec4(0.2f, 0.2f, 1.0f, 1.0f));

    dumpSceneGraph(rootnode,0);

    viewer.run();


  } catch (std::exception& e) {
    Consoleln("caught: " << String(e.what()));
  }

  Release(::u);

  base::MemoryTracer::dumpNamed();
  Consoleln("Exiting.");

  return 0;
}


void dumpSceneGraph(osg::Node* node, Int level)
{
  std::cout << String(level,' ') << node->className() << "#" << node->getName()  << std::endl;
  
  if (instanceof(*node,osg::Group)) {
    osg::Group* group = dynamic_cast<osg::Group*>(node);
    if (group->getName() != "debug") {
      if (instanceof(*group,osg::LOD))
	dumpSceneGraph(group->getChild(0),level+1);
      else {
	Int nc = group->getNumChildren();
	for (Int c=0; c<nc; c++) {
	  Node* child = group->getChild(c);
	  dumpSceneGraph(child,level+1);
	}
      }
    }
  }

  if (instanceof(*node, osg::Geode)) {
    osg::Geode* geode = dynamic_cast<osg::Geode*>(node);
    Int nd = geode->getNumDrawables();
    for (Int d=0; d<nd; d++) {
      osg::Drawable* drawable = geode->getDrawable(d);
      std::cout << String(level+1,' ') << "Drawable:" << drawable->className() << std::endl;
    }
    
  }

}
