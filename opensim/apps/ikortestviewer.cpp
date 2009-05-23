/****************************************************************************
  Copyright (C)2003 David Jung <opensim@pobox.com>

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
  
  $Id: ikortestviewer.cpp 1028 2004-02-11 20:45:27Z jungd $
  $Revision: 1.4 $
  $Date: 2004-02-11 15:45:27 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/base>
#include <robot/robot>

#include <base/Application>
#include <base/Universe>
#include <base/VFile>
#include <base/Math>
#include <base/Time>
#include <base/SVD>
#include <gfx/LookAtCameraManipulator>
#include <robot/Robot>
#include <robot/sim/TestBasicEnvironment>
#include <robot/sim/VisualIKORTest>

using base::Application;
using base::Universe;
using base::PathName;
using base::VFileSystem;
using base::VFile;
using base::VDirectory;
using base::ref;
using base::Time;
using base::Math;
using base::Matrix;
using base::Vector;
using gfx::LookAtCameraManipulator;
using robot::Robot;
using robot::sim::BasicEnvironment;
using robot::sim::TestBasicEnvironment;
using robot::sim::IKORTest;
using robot::sim::VisualIKORTest;

// Open Scene Graph classes
#include <osg/NodeVisitor>
#include <osgGLOW/Viewer>


// GUI classes
#include "glow.h"
#include "glowQuickPalette.h"
#include "glowLabelWidget.h"
#include "glowPushButtonWidget.h"
#include "glowSliderWidget.h"
#include "glowAux.h"

using namespace glow;


namespace app {
  
  
class GUI : public osg::NodeVisitor, public base::Object
{
public:
  GUI() 
    : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
  {}

  virtual String className() const { return String("GUI"); }
  
  
  void init(base::ref<VisualIKORTest> itest) 
  {
    this->itest = itest;
    testIndex=0;
    
    char title[64];
    strncpy(title,(String("OpenSim ") + Application::getVersion() + " - IKORTest result viewer").c_str(),64);
    controlWindow = NewObj GlowQuickPaletteWindow(title, winx+winw+10, winy,GlowQuickPalette::vertical,GlowQuickPalette::alignExpand);

    EventReceiver* receiver = NewObj EventReceiver(*this);
    
    // add a widget for selecting which test the other control are manipulating (currentTest())
    GlowQuickPanelWidget* stpanel = controlWindow->AddPanel(GlowQuickPanelWidget::etchedStyle, "Subtests");
    GlowQuickPanelWidget* stvpanel = stpanel->AddArrangingPanel(GlowQuickPalette::vertical);
    testSelector = stvpanel->AddRadioGroup(GlowQuickPalette::vertical, 3, receiver);
    for (Int t=0; t<itest->numTests(); t++) {
      IKORTest::Test& test(itest->getTest(t));
      if (!test.displayRangeSpecified) {
        test.displayRangeSpecified = true;
        test.displayStartIndex = 0;
        test.displayEndIndex = test.times.size()-1;
      }
      testRadioButtons.push_back(testSelector->AddRadioButton(test.getName().c_str()));
    }

    
    // start and end sliders for selecting the trajectory range to display (for the currently selected test)
    GlowQuickPanelWidget* tpanel = controlWindow->AddPanel(GlowQuickPanelWidget::etchedStyle,0);
    GlowQuickPanelWidget* tvpanel = tpanel->AddArrangingPanel(GlowQuickPalette::vertical, GlowQuickPalette::alignExpand);    
    String rangeTitle(currentTest().getName()+": Trajectory range (steps)");
    testPanelLabel = tvpanel->AddLabel(rangeTitle.c_str());
    
    
    char timeBuf[16];
    GlowQuickPanelWidget* starthpanel = tvpanel->AddArrangingPanel(GlowQuickPalette::horizontal, GlowQuickPalette::alignExpand);
    GlowQuickPanelWidget* startvpanel = starthpanel->AddArrangingPanel(GlowQuickPalette::vertical, GlowQuickPalette::alignExpand);
    trajStart = startvpanel->AddSlider(0, currentTest().times.size()-1, currentTest().displayStartIndex,
                                       GlowSliderWidget::defaultOptions, 10, "%4.0f", "start", receiver);
    sprintf(timeBuf, "(time: %5.3fs)", currentTest().times[currentTest().displayStartIndex].seconds());
    trajStartTime = starthpanel->AddLabel(timeBuf);
    trajStartTime->SetFont(GlowFont::fixed8by13);
                                   
    GlowQuickPanelWidget* endhpanel = tvpanel->AddArrangingPanel(GlowQuickPalette::horizontal, GlowQuickPalette::alignExpand);
    GlowQuickPanelWidget* endvpanel = endhpanel->AddArrangingPanel(GlowQuickPalette::vertical, GlowQuickPalette::alignExpand);
    trajEnd   = endvpanel->AddSlider(0, currentTest().times.size()-1, currentTest().displayEndIndex,
                                     GlowSliderWidget::defaultOptions, 10, "%4.0f", "end", receiver);
    sprintf(timeBuf, "(time: %5.3fs)", currentTest().times[currentTest().displayEndIndex].seconds());
    trajEndTime = endhpanel->AddLabel(timeBuf);
    trajEndTime->SetFont(GlowFont::fixed8by13);
              
    qVector = tvpanel->AddLabel( (String("q (at current start time):\n")+formatVector(Vector(14))).c_str() );
    jacobMatrix = tvpanel->AddLabel( (String("Jacobian (at current start time):\n")+formatMatrix(Matrix(6,14))).c_str() );
              
    GlowQuickPanelWidget* fpanel = controlWindow->AddArrangingPanel(GlowQuickPalette::horizontal);    
    showObstCheckbox = fpanel->AddCheckBox("Show obstacles", itest->displayObstacles?GlowCheckBoxWidget::on:GlowCheckBoxWidget::off, receiver); 
    showAxesCheckbox = fpanel->AddCheckBox("Show axes", itest->displayAxes?GlowCheckBoxWidget::on:GlowCheckBoxWidget::off, receiver); 
    showPlatformCheckbox = fpanel->AddCheckBox("Show platform", itest->displayPlatform?GlowCheckBoxWidget::on:GlowCheckBoxWidget::off, receiver); 
    showEEPathCheckbox = fpanel->AddCheckBox("Show EE path", itest->displayEEPath?GlowCheckBoxWidget::on:GlowCheckBoxWidget::off, receiver); 

    GlowQuickPanelWidget* spanel = controlWindow->AddArrangingPanel(GlowQuickPalette::horizontal);    
    saveButton = spanel->AddPushButton("Save", receiver);
    outputSVG = spanel->AddPushButton("Save as SVG", receiver);
    outputBitmap = spanel->AddPushButton("Save as bitmap", receiver);
    
    quitButton = controlWindow->AddPushButton("Quit", receiver);
    
    controlWindow->Pack();
    
    itest->updateVisuals();
  }

  
  
  void setWindowDims(SInt x, SInt y, Int w, Int h) { winx=x; winy=y; winw=w; winh=h; }
  SInt winx, winy;
  Int winw, winh;
  
  void setCameraManipulator(gfx::LookAtCameraManipulator& cameraManipulator)
    { this->cameraManipulator = &cameraManipulator; } 
  osg::ref_ptr<gfx::LookAtCameraManipulator> cameraManipulator;

  void setFilesystem(base::ref<VFileSystem> fs) { filesystem = fs; }
  base::ref<VFileSystem> filesystem;
  
  void setITestFile(base::ref<VFile> itestFile) { this->itestFile = itestFile; }
  base::ref<VFile> itestFile;

  base::ref<VisualIKORTest> itest;
  Int testIndex;
  IKORTest::Test& currentTest() const { return itest->getTest(testIndex); }


  // widgets
  GlowQuickPaletteWindow* controlWindow;
  GlowQuickRadioGroupWidget* testSelector;
  array<GlowRadioButtonWidget*> testRadioButtons;
  GlowLabelWidget* testPanelLabel;
  GlowSliderWidget* trajStart; // control the range of trajectory points displayed (steps)
  GlowSliderWidget* trajEnd; 
  GlowLabelWidget* trajStartTime;
  GlowLabelWidget* trajEndTime;
  GlowLabelWidget* qVector;
  GlowLabelWidget* jacobMatrix;
  GlowCheckBoxWidget* showObstCheckbox;
  GlowCheckBoxWidget* showAxesCheckbox;
  GlowCheckBoxWidget* showPlatformCheckbox;
  GlowCheckBoxWidget* showEEPathCheckbox;
  GlowPushButtonWidget* saveButton;
  GlowPushButtonWidget* outputSVG;
  GlowPushButtonWidget* outputBitmap;
  GlowPushButtonWidget* quitButton;
  
  

  static String formatVector(const Vector& v)
  {
    String vector;
    for(Int i=0; i<v.size(); i++) {
      char buf[16];
      if (v[i]>=0)
        sprintf(buf, " %8.4f ",v[i]);
      else
        sprintf(buf, "%8.4f ",v[i]);
      vector += buf;
    }
    return vector;  
  }  
  
  
  static String formatMatrix(const Matrix& m)
  {
    String matrix;
    for(Int r=0; r<m.size1(); r++) {
      String row;
      for(Int c=0; c<m.size2(); c++) {
        char buf[16];
        if (m(r,c)>=0)
          sprintf(buf, " %8.4f ",m(r,c));
        else
          sprintf(buf, "%8.4f ",m(r,c));
        row += buf;
      }
      if (r<m.size1()-1) row+="\n";
      matrix += row;
    }
    return matrix;
  }

  
  // update widgets to reflect current state
  void update() 
  {
    // update test panel title
    String rangeTitle(currentTest().getName()+": Trajectory range (steps)");
    testPanelLabel->SetText(rangeTitle.c_str());
    
    // update trajectory sliders
    trajStart->SetMaximum(currentTest().times.size()-1);
    trajStart->SetValue(currentTest().displayStartIndex);
    trajEnd->SetMaximum(currentTest().times.size()-1);
    trajEnd->SetValue(currentTest().displayEndIndex);
    char timeBuf[16];
    sprintf(timeBuf, "(time: %5.3fs)", currentTest().times[currentTest().displayStartIndex].seconds());
    trajStartTime->SetText( timeBuf );
    sprintf(timeBuf, "(time: %5.3fs)", currentTest().times[currentTest().displayEndIndex].seconds());
    trajEndTime->SetText( timeBuf );
    
    Matrix Jq( currentTest().Js[currentTest().displayStartIndex] );
    jacobMatrix->SetText( (String("Jacobian (at current start time):\n")+formatMatrix(Jq)).c_str() );
    
    Vector q( currentTest().qs[currentTest().displayStartIndex] );
    qVector->SetText( (String("q (at current start time):\n")+formatVector(q)).c_str() );
  }
  
  
  // Event receiver for widgets
  class EventReceiver : public GlowPushButtonReceiver,
                        public GlowSliderReceiver,
                        public GlowCheckBoxReceiver,
                        public GlowRadioButtonReceiver
  {
  public:
    EventReceiver(GUI& gui) : gui(gui) {}
  
    virtual void OnMessage(const GlowPushButtonMessage& message) 
    { 
      if (message.widget == gui.quitButton)
        Glow::exitMainLoop();
      else if (message.widget == gui.saveButton) {
        // output camera parameters in case the user wants to known them (e.g. to copy them into the original test spec file)
        Consoleln("Camera parameters - target:" << gui.cameraManipulator->getTarget()
                    << " alpha:" << gui.cameraManipulator->getAlpha()
                    << " theta:" << gui.cameraManipulator->getTheta()
                    << " d:" << gui.cameraManipulator->getD() );

        Consoleln("Saving IKORTest file " << gui.itestFile->pathName().str());
        gui.itest->saveResults(false,gui.itestFile->pathName());
      }
      else if (message.widget == gui.outputSVG) {
        PathName svgFileName(gui.itest->getName()+".svg");
        base::ref<VDirectory> outDir( gui.filesystem->getDirectory(svgFileName.path()) );
        Consoleln("Saving SVG output file: " << (outDir->pathName()+svgFileName.name()).str());
        base::ref<VFile> outputFile( outDir->createFile(svgFileName.name()) );
        gui.itest->save(outputFile, "svg");
        outputFile->close();
      }
      else if (message.widget == gui.outputBitmap) {
      }
    }
    
    
    virtual void OnMessage(const GlowSliderMessage& message) 
    {
      IKORTest::Test& test( gui.currentTest() );
      
      if (message.widget == gui.trajStart) {
        test.displayStartIndex = Int(message.value);
        if (test.displayStartIndex > test.displayEndIndex) {
          test.displayEndIndex = test.displayStartIndex;
          gui.trajEnd->SetValue( test.displayEndIndex );
        }
      }
      else if (message.widget == gui.trajEnd) {
        test.displayEndIndex = Int(message.value);
        if (test.displayEndIndex < test.displayStartIndex) {
          test.displayStartIndex = test.displayEndIndex;
          gui.trajStart->SetValue(test.displayStartIndex);
        }
      }
      
      
      char timeBuf[16];
      sprintf(timeBuf, "(time: %5.3fs)", test.times[test.displayStartIndex].seconds());
      gui.trajStartTime->SetText( timeBuf );
      sprintf(timeBuf, "(time: %5.3fs)", test.times[test.displayEndIndex].seconds());
      gui.trajEndTime->SetText( timeBuf );
      
      Matrix Jq( test.Js[test.displayStartIndex] );
      gui.jacobMatrix->SetText( (String("Jacobian (at current start time):\n")+formatMatrix(Jq)).c_str() );

      Vector q( test.qs[test.displayStartIndex] );
      gui.qVector->SetText( (String("q (at current start time):\n")+formatVector(q)).c_str() );
      
      
      gui.itest->updateVisuals();
    }

    
    virtual void OnMessage(const GlowCheckBoxMessage& message)
    {
      if (message.widget == gui.showObstCheckbox)
        gui.itest->displayObstacles = (gui.showObstCheckbox->GetState() == GlowCheckBoxWidget::on);
      else if (message.widget == gui.showAxesCheckbox) 
        gui.itest->displayAxes = (gui.showAxesCheckbox->GetState() == GlowCheckBoxWidget::on);
      else if (message.widget == gui.showPlatformCheckbox) 
        gui.itest->displayPlatform = (gui.showPlatformCheckbox->GetState() == GlowCheckBoxWidget::on);
      else if (message.widget == gui.showEEPathCheckbox) 
        gui.itest->displayEEPath = (gui.showEEPathCheckbox->GetState() == GlowCheckBoxWidget::on);

      gui.itest->updateVisuals();
    }   
    
    
    virtual void OnMessage(const GlowRadioButtonMessage& message)
    {
      for(Int t=0; t<gui.testRadioButtons.size(); t++)
        if (message.buttonWidget == gui.testRadioButtons[t]) {
          gui.testIndex = t;
          gui.update();
          break;
        }
    }
    
    
    GUI& gui;
  };  
  
  
  
  /// this is called on every frame
  virtual void reset() {
    
    const Real frameTime = 1/80.0;

    // update camera state recorded in IKORTest
    itest->lookAtTarget = cameraManipulator->getTarget();
    itest->alpha = cameraManipulator->getAlpha();
    itest->theta = cameraManipulator->getTheta();
    itest->d = cameraManipulator->getD();


    // maintain reasonable frame-rate (not unnecessarily fast)
    Time elapsed(Time::now() - lastFrameTime);
    
    if (elapsed.seconds() < frameTime) {
      Real sleepFor = (frameTime - elapsed.seconds());
      Time::sleep(sleepFor);
    }
    
    lastFrameTime = Time::now();
    
  }
  
  
  base::Time lastFrameTime;
  
  
};


} // namespace app




using namespace app;


int main(int argc, char *argv[])
{
  //setDebugOutputID(JFKE);
  //setDebugOutputID(Tmp);
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
    String usage("\nUsage: ikortestviewer <test_spec_file>");
    PathName testSpecFileName;
    if (argc != 2) {
      Consoleln(usage);
      return -1;
    }
    else
      testSpecFileName = String(argv[1]);

    if (testSpecFileName.isRelative()) testSpecFileName.prepend(String("data/test/"));

    Consoleln("");


    ref<VFile> testSpecFile( universe->cache()->findFile(testSpecFileName) );
    Consoleln("Loading test specification from file '" << testSpecFile->pathName().str() << "'.");
    ref<VisualIKORTest> itest(NewObj VisualIKORTest(testSpecFile, app.filesystem(), universe->cache()));
    
    
    // Setup OSG visual 
    osg::Group* rootnode = NewObj osg::Group;
    rootnode->setName("root");

    rootnode->addChild( itest->createOSGVisual() );    

    universe->enableSimulationRendering(false);
    universe->preSimulate();

    // setup OSG viewer
    osgGLOW::Viewer* viewer = NewObj osgGLOW::Viewer();
    viewer->addViewport( rootnode );
    viewer->setWindowTitle("OpenSim - IKORTest result viewer");
    
    // register our camera manipulator
    base::Point3 target(itest->lookAtTarget);
    osg::ref_ptr<gfx::LookAtCameraManipulator> lookAtCameraManip = new gfx::LookAtCameraManipulator(
                                                                     itest->alpha,itest->theta,itest->d,
                                                                     target.x,target.y,target.z);

    viewer->registerCameraManipulator(&(*lookAtCameraManip));
    //lookAtCameraManip.trackingEnable();
    
    // Open window so camera manipulator's warp pointer request will succeed
    viewer->open();

    viewer->selectCameraManipulator(0);

    osgUtil::SceneView* sceneView = viewer->getViewportSceneView(0);
    sceneView->setBackgroundColor(osg::Vec4(1,1,1,0));
    //osg::Camera* camera = sceneView->getCamera();
    
    osg::ref_ptr<GUI> gui = new GUI();
    gui->setWindowDims(viewer->PositionX(), viewer->PositionY(), viewer->Width(), viewer->Height());
    gui->setFilesystem(app.filesystem());
    gui->init(itest);
    gui->setCameraManipulator(*lookAtCameraManip);
    gui->setITestFile(testSpecFile);
    viewer->getViewportSceneView(0)->setUpdateVisitor(&(*gui));
    
    viewer->run();
    
    delete viewer;
    
    // hack to wait until all windows are properly closed before exiting
    for(Int i=0; i<100; i++)
      Glow::MainLoop();
    
    
  } catch (std::exception& e) {
    Consoleln("caught: " << String(e.what()));
  }


  Consoleln("Exiting.");
  
  return 0;
}

