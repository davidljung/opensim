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
  
  $Id: ViewEnvWindow.cpp 1132 2004-09-28 20:51:07Z jungd $
  $Revision: 1.1 $
  $Date: 2004-09-28 16:51:07 -0400 (Tue, 28 Sep 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <apps/ViewEnvWindow>

#include <base/Math>
#include <base/Time>
#include <base/Application>

#include <robot/Robot>
#include <robot/RobotDescription>

#include <apps/GenericControlInterfaceWidget>
#include <apps/JointPosAndIKControlWidget>
#include <apps/JointPosVelControllerWidget>
#include <apps/LinkProximitySensorsWidget>

using apps::ViewEnvWindow;

using base::Math;
using base::Time;
using base::Application;


using robot::Robot;
using robot::RobotDescription;
using robot::PlatformDescription;
using robot::ManipulatorDescription;
using robot::ControlInterface;
using robot::sim::BasicEnvironment;

using apps::GenericControlInterfaceWidget;
using apps::JointPosAndIKControlWidget;
using apps::JointPosVelControllerWidget;
using apps::LinkProximitySensorsWidget;


// GTKmm
#include <gtkmm.h>

using Gtk::manage;


 


static const Real initialSimStep = 1.0/150.0;


ViewEnvWindow::ViewEnvWindow(Gtk::Main& kit)
    : GUIControlWindow(kit),
      quit(false),
      tophbox(false,5), topvbox(false,5),
      simFrame("Simulation"), simFrameVBox(false,5),
      frameRateHBox(false,5),  
      simSpeedHBox(false,5), 
      simStep(initialSimStep, 1.0/500.0, 1/30.0, 1/250.0, 1/100.0),
      simStepScale(simStep), 
      
      envFrame("Environment"),
      interfaceFrame("Available Interfaces"), interfaceFrameVBox(false,5),
      
      newPanelButton("New Panel"),
      
      interfacesFrame("Realized Interfaces"), 
      interfacesFrameHBox(false,5),
      
      quitButton("Quit"),
      saveFramesButton("Save Frames"),

      kit(kit), saveFrame(false),
      simStepSize(initialSimStep)
{
  set_title( (Application::getLongName()+" "+Application::getVersion()+" - view environment - Control").c_str() );
  set_border_width(5);

  pushStatusMessage(StatusInfo,"OK",99999);
  
  Gtk::VBox* contentVBox = new Gtk::VBox(false,0);
  contentVBox->pack_start(tophbox);
  contentVBox->pack_end(statusBar,Gtk::PACK_SHRINK,5);
  add(*manage(contentVBox));
}
  
  
void ViewEnvWindow::init(ref<BasicEnvironment> env)
{
  this->env = env;

  tophbox.pack_start(topvbox);
  {
  
    topvbox.pack_start(simFrame, Gtk::PACK_SHRINK,5); 
    {
      simFrame.add(simFrameVBox);
      
      simFrameVBox.pack_start(frameRateHBox);
      {
        frameRateHBox.pack_start(*manage(new Gtk::Label("Framerate:",1,0.5))); 
        frameRate.set_size_request(50,0);
        frameRateHBox.pack_start(frameRate); 
      }
      
      simFrameVBox.pack_start(simSpeedHBox);
      {
        simSpeedHBox.pack_start(*manage(new Gtk::Label("Sim speed:",1,0.5))); 
        simSpeed.set_size_request(50,0);
        simSpeedHBox.pack_start(simSpeed);
      }
      
      simFrameVBox.pack_start(simStepHBox);
      {
        simStepHBox.pack_start(*manage(new Gtk::Label("Stepsize: ")));
        simStepScale.set_draw_value(true);
        simStepScale.set_digits(3);
        simStepScale.set_value_pos(Gtk::POS_LEFT);
        simStepScale.set_update_policy(Gtk::UPDATE_DISCONTINUOUS);
        simStepScale.set_size_request(100,-1);
        simStepHBox.pack_start(simStepScale);
      }

    }
    
    topvbox.pack_start(envFrame);
    {
      envWindow.set_border_width(5);
      envWindow.set_policy(Gtk::POLICY_AUTOMATIC, Gtk::POLICY_AUTOMATIC);
      envFrame.add(envWindow);
      {
        // fill out tree with environment data
        envTree = Gtk::TreeStore::create(envTreeColumns);
        envTreeView.set_model(envTree);
        envTreeView.set_border_width(5);
        envTreeView.set_size_request(300,200);      
        fillInEnvTree();
      
        envTreeView.append_column("Object", envTreeColumns.name);
        envWindow.add(envTreeView);
      }
    }
    
    topvbox.pack_start(interfaceFrame);
    {
      interfaceFrame.add(interfaceFrameVBox);
      {
        interfaceWindow.set_border_width(5);
        interfaceWindow.set_policy(Gtk::POLICY_AUTOMATIC, Gtk::POLICY_AUTOMATIC);
        
        interfaceFrameVBox.pack_start(interfaceWindow);
        {
          // prepare list for data
          interfaceList = Gtk::ListStore::create(interfaceListColumns);
          interfaceListView.set_model(interfaceList);
          interfaceListView.set_border_width(5);
          interfaceListView.set_size_request(-1,100);
        
          interfaceListView.append_column("Name", interfaceListColumns.name);
          interfaceListView.append_column("Type", interfaceListColumns.type);
          interfaceWindow.add(interfaceListView);
        }
        
      }
    }
    
    Gtk::HBox* qbox = new Gtk::HBox();
    qbox->pack_start(quitButton, false, false, 5);
    qbox->pack_start(saveFramesButton, false, false, 5);
    qbox->pack_end(newPanelButton, Gtk::PACK_SHRINK,5);
    topvbox.pack_end(*manage(qbox),Gtk::PACK_SHRINK,5);
  }
  
  interfacesFrame.set_border_width(5);
  tophbox.pack_start(interfacesFrame);
  {
    interfacesFrame.add(interfacesFrameHBox);
    // panels are dynamically added to this HBox when the new panel button is clicked
  }
  
  // initially instantiate one panel
  newPanelClicked();  
  
  // connect 
  quitButton.signal_clicked().connect(SigC::slot(*this, &ViewEnvWindow::quitClicked));
  saveFramesButton.signal_clicked().connect(SigC::slot(*this, &ViewEnvWindow::saveFramesClicked));
  simStep.signal_value_changed().connect( SigC::slot(*this, &ViewEnvWindow::simStepSizeChanged) );
  envTreeViewSelection = envTreeView.get_selection();
  envTreeViewSelection->signal_changed().connect(SigC::slot(*this, &ViewEnvWindow::envTreeSelectionChanged));
  interfaceListViewSelection = interfaceListView.get_selection();
  newPanelButton.signal_clicked().connect(SigC::slot(*this, &ViewEnvWindow::newPanelClicked) );

}

  
void ViewEnvWindow::show()
{
  Gtk::Window::show_all();
}
 

void ViewEnvWindow::pushStatusMessage(StatusType type, const String& message, int context_id)
{
  statusBar.push(message,context_id);
  /*
  if (type == StatusNone)
    statusBar.modify_bg(Gtk::STATE_NORMAL, Gdk::Color("grey"));
  else if (type == StatusInfo)
    statusBar.modify_bg(Gtk::STATE_NORMAL, Gdk::Color("green"));
  else if (type == StatusError)
    statusBar.modify_bg(Gtk::STATE_NORMAL, Gdk::Color("red"));
  */
  lastStatusbarChange = Time::now();
}

void ViewEnvWindow::popStatusMessage(int context_id)
{
  statusBar.pop(context_id);
  lastStatusbarChange = Time::now();
}
 
  
void ViewEnvWindow::setFrameRate(Real rate)
{
  // store and average
  frameRates.push_back(rate);
  if (frameRates.size() > 100) frameRates.pop_front();
  
  Real total = 0;
  std::list<Real>::const_iterator fri = frameRates.begin();
  std::list<Real>::const_iterator end = frameRates.end();
  while (fri != end) {
    total += *fri;
    ++fri;
  }
  aveFrameRate = total/frameRates.size();
  
  Real r = Real(Int(aveFrameRate*10.0))/10.0;
  frameRate.set_text(base::realToString(r));
}
  
  
void ViewEnvWindow::setSimSpeed(Real speed)
{
  simSpeeds.push_back(speed);
  if (simSpeeds.size() > 100) simSpeeds.pop_front();

  Real total = 0;
  std::list<Real>::const_iterator ssi = simSpeeds.begin();
  std::list<Real>::const_iterator end = simSpeeds.end();
  while (ssi != end) {
    total += *ssi;
    ++ssi;
  }
  aveSimSpeed = total/simSpeeds.size();

  Real s = Real(Int(aveSimSpeed*100.0))/100.0;
  simSpeed.set_text(base::realToString(s));
}

  
void ViewEnvWindow::update(const base::Time& simTime)
{
  std::list<ControlInterfaceWidget*>::iterator iw = controlInterfaceWidgets.begin();
  std::list<ControlInterfaceWidget*>::iterator end = controlInterfaceWidgets.end();
  while (iw != end) {
    (*iw)->iterate(simTime);
    ++iw;
  }
  
  // periodically pop the last status message from the status bar (context id 0)
  if ((Time::now() - lastStatusbarChange) > 15) 
    popStatusMessage(0);
}
  
  
  
  
void ViewEnvWindow::fillInEnvTree()
{
  // add row for each robot (and child rows for its components)
  for(Int r=0; r<env->numRobots(); r++) { // for each robot
    
    ref<Robot> robot(env->getRobot(r));

    if (robot->isDescriptionProvided()) {
      
      ref<const RobotDescription> rd( robot->getRobotDescription() );
      
      Gtk::TreeModel::Row row = *(envTree->append());
      row[envTreeColumns.name] = rd->getName();
      row[envTreeColumns.type] = EnvTreeColumns::RobotType;
      row[envTreeColumns.robot] = robot;
      row[envTreeColumns.index] = r;

      // add platform row
      ref<const PlatformDescription> pd( rd->platform() );
      Gtk::TreeModel::Row prow = *(envTree->append(row.children()));
      prow[envTreeColumns.name] = pd->getName();
      prow[envTreeColumns.type] = EnvTreeColumns::PlatformType;
      prow[envTreeColumns.robot] = robot;
      prow[envTreeColumns.index] = 0;
      
      // add child row for each manipulator
      const array<ref<const ManipulatorDescription> >& manips( rd->manipulators() );
      for(Int m=0; m<manips.size(); m++) { // for each manip
      
        ref<const ManipulatorDescription> md( manips[m] );
        Gtk::TreeModel::Row mrow = *(envTree->append(prow.children()));
        mrow[envTreeColumns.name] = md->getName();
        mrow[envTreeColumns.type] = EnvTreeColumns::ManipulatorType;
        mrow[envTreeColumns.robot] = robot;
        mrow[envTreeColumns.index] = m;
        
      }
      
    }
    else {
      // add generic robot row (as we have no description)
      Gtk::TreeModel::Row row = *(envTree->append());
      row[envTreeColumns.name] = "untitled-robot";
      row[envTreeColumns.type] = EnvTreeColumns::RobotType;
      row[envTreeColumns.robot] = robot;
      row[envTreeColumns.index] = r;
    }
    
  } // end for each robot
  
  
  // add row for each tool
  for(Int t=0; t<env->numTools(); t++) { // for each tool
    ref<const BasicEnvironment::Tool> tool( env->getTool(t) );
    
    ref<const robot::ToolDescription> td( tool->getToolDescription() );
    Gtk::TreeModel::Row row = *(envTree->append());
    row[envTreeColumns.name] = td->getName();
    row[envTreeColumns.type] = EnvTreeColumns::ToolType;
    row[envTreeColumns.index] = t;
  }
  
  
  
  // add row for each Obstacle
  for(Int o=0; o<env->numObstacles(); o++) {
    ref<const BasicEnvironment::Obstacle> obst(env->getObstacle(o));
    String s = obst->getName();
    if (s.length() == 0) s = String("obstacle");
    switch (obst->type) {
      case BasicEnvironment::Obstacle::BoxObstacle: s += "(box)"; break;
      case BasicEnvironment::Obstacle::SphereObstacle: s += "(sphere)"; break;
      default: ;
    }
    Gtk::TreeModel::Row row = *(envTree->append());
    row[envTreeColumns.name] = s;
    row[envTreeColumns.type] = EnvTreeColumns::ObstacleType;
    row[envTreeColumns.index] = o;
  }

  
}
  
  
static inline bool hasPrefix(const String& str, const String& prefix)  
{
  return (str.substr(0,prefix.length()) == prefix);
}

static inline bool hasPrefixN(const String& str, const String& prefix)  
{
  return (str.substr(0,prefix.length()) == prefix) && ::isdigit(str[prefix.length()]);
}

static Int prefixedIndex(const String& str, const String& prefix)
{
  String remaining( str.substr(prefix.length(), str.length()-prefix.length()) );
  return base::stringToInt( remaining );
}


static bool isPrefixedNotIndex(const String& str, const String& prefix, Int index)
{
  return hasPrefixN(str,prefix) && (prefixedIndex(str,prefix) != index);
}




void ViewEnvWindow::fillInInterfaceList(ref<Robot> robot, EnvTreeColumns::Type type, Int index)
{
  interfaceList->clear();
  
  if (robot != 0) {
    array<std::pair<String,String> > interfaces = robot->controlInterfaces();
    for(Int i=0; i<interfaces.size(); i++) {
      
      String iname = interfaces[i].first;
      String itype = interfaces[i].second;
      
      bool exclude = false;
      // filter out interfaces that aren't relevent to the type of object selected
      //  in the tree.  For example, if a robot is selected, filter out the manipulator
      //  interfaces, as they can be obtained by selecting the manipulator directly
      if (type == EnvTreeColumns::RobotType) {
        if (hasPrefix(iname, "platform")) exclude = true;
        if (hasPrefix(iname, "manipulator")) exclude = true;
        if (hasPrefix(iname, "tool")) exclude = true;
      }
      else if (type == EnvTreeColumns::PlatformType) {
        if (hasPrefix(iname, "manipulator")) exclude = true;
        if (hasPrefix(iname, "tool")) exclude = true;
      }
      else if (type == EnvTreeColumns::ManipulatorType) {
        if (hasPrefix(iname, "platform")) exclude = true;
        if (hasPrefix(iname, "tool")) exclude = true;
        if (isPrefixedNotIndex(iname, "manipulatorPosition",index+1)) exclude = true;
        if (isPrefixedNotIndex(iname, "manipulatorVelocity",index+1)) exclude = true;
        if (isPrefixedNotIndex(iname, "manipulatorToolGrip",index+1)) exclude = true;
        if (isPrefixedNotIndex(iname, "manipulatorProximity",index+1)) exclude = true;
        if (isPrefixedNotIndex(iname, "manipulator",index+1)) exclude = true;
      }
      
      if (!exclude) {
        Gtk::TreeModel::Row row = *(interfaceList->append());
        row[interfaceListColumns.name] = iname;
        row[interfaceListColumns.type] = itype;
      }
    }
  }
}
  
  
  
// Signal handlers
void ViewEnvWindow::envTreeSelectionChanged()
{
  Gtk::TreeModel::iterator iter = envTreeViewSelection->get_selected();
  if (iter) { // if anything selected
    Gtk::TreeModel::Row row = *iter;
    
    ref<Robot> robot = ref<Robot>(row[envTreeColumns.robot]);
    fillInInterfaceList(robot, row[envTreeColumns.type], row[envTreeColumns.index]);
  }
}
  

void ViewEnvWindow::newPanelClicked()
{
  // create a new tabbed panel of ControlInterfaceWidgets
  Gtk::VBox* vbox = new Gtk::VBox(false,5);
  
  Gtk::Notebook* tabPane = new Gtk::Notebook();
  tabPane->set_size_request(250,-1);
  tabPane->set_scrollable(true);
  tabPane->popup_enable();
  
  vbox->pack_start(*manage(tabPane), Gtk::PACK_SHRINK,5);
  
  Gtk::HBox* hbox = new Gtk::HBox(false,5);
  Gtk::Button* realizeButton = new Gtk::Button("Realize selected");
  hbox->pack_start(*manage(realizeButton), false, false, 5);
  Gtk::Button* closeButton = new Gtk::Button("Close");
  hbox->pack_start(*manage(closeButton), Gtk::PACK_SHRINK,5);
  
  vbox->pack_end(*manage(hbox), false, 5);

  realizeButton->signal_clicked().connect( SigC::bind<Int>( SigC::slot(*this, &ViewEnvWindow::realizeClicked), interfacePanels.size() ) );
  closeButton->signal_clicked().connect( SigC::bind<Int>( SigC::slot(*this, &ViewEnvWindow::closeClicked), interfacePanels.size() ) );
  
  interfacesFrameHBox.pack_start(*manage(vbox));
  interfacesFrameHBox.show_all();
  interfacePanels.push_back(tabPane);
}
  
  
void ViewEnvWindow::saveFramesClicked()
{
  saveFrame = saveFramesButton.get_active();
}

  
void ViewEnvWindow::realizeClicked(Int p)
{
  Gtk::TreeModel::iterator iter = interfaceListViewSelection->get_selected();
  if (iter) {
    Gtk::TreeModel::Row row = *iter;
    Glib::ustring name = row[interfaceListColumns.name];
    Glib::ustring type = row[interfaceListColumns.type];
    Gtk::TreeModel::iterator iter = envTreeViewSelection->get_selected();
    if (iter) { 
      Gtk::TreeModel::Row row = *iter;
      ref<Robot> robot = ref<Robot>(row[envTreeColumns.robot]);
//      EnvTreeColumns::Type otype = row[envTreeColumns.type];
      Int index = row[envTreeColumns.index]; // type dependent index of obj (e.g. manip, tool etc.)

      if (robot != 0) {
        // instantiate a new control interface widget
        Glib::ustring objectName = row[envTreeColumns.name];
        String robotName;
        if (robot->isDescriptionProvided()) {
          ref<const RobotDescription> rd(robot->getRobotDescription());
          robotName = rd->getName();
        }
        
        // construct intelligent label for the tab
        String labelString = String()+robotName;
        if (robotName != objectName.raw())
          labelString += String("(")+objectName.raw()+")";
        labelString += "\n-"+name.raw();
        
        // instantiate a widget based on the interface type
        ref<ControlInterface> interface = robot->getControlInterface(name.raw());

        ControlInterfaceWidget* tab; 
        if (type == "JointPositionControl") {
          tab = new JointPosAndIKControlWidget(*this);
          tab->init(env, robot, index, interface);
        }
        else if (type == "JointVelocityControl") {
          tab = new JointPosAndIKControlWidget(*this);
          tab->init(env, robot, index, interface);
        }
        else if (type == "LinkProximitySensors") {
          tab = new LinkProximitySensorsWidget(*this);
          tab->init(env, robot, index, interface);
        }
        else {
          tab = new GenericControlInterfaceWidget(*this);
          tab->init(env, robot, index, interface);
        }
        
        controlInterfaceWidgets.push_back(tab);
        Gtk::Notebook* tabPanel = interfacePanels[p];
        tabPanel->append_page(*manage(tab),labelString);
        tabPanel->show_all();
        tabPanel->set_current_page( tabPanel->get_n_pages()-1 ); // select last page (just added)
      }
    }
  }
  
}


void ViewEnvWindow::closeClicked(Int p)
{
  if (interfacePanels[p]->get_n_pages() == 0) return;
  
  int currentPage = interfacePanels[p]->get_current_page();
  ControlInterfaceWidget* w =  dynamic_cast<ControlInterfaceWidget*>(interfacePanels[p]->get_nth_page(currentPage));
  w->finalize();
  controlInterfaceWidgets.remove(w);
  interfacePanels[p]->remove_page(currentPage);
}

