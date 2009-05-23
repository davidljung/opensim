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
  
  $Id: IKORTest.cpp 1033 2004-02-11 20:47:52Z jungd $
  $Revision: 1.6 $
  $Date: 2004-02-11 15:47:52 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <robot/sim/IKORTest>

#include <base/Externalizer>
#include <base/externalization_error>
#include <base/Expression>
#include <base/Matrix>
#include <base/VDirectory>
#include <base/VFile>

#include <robot/sim/BasicEnvironment>
#include <robot/control/kinematics/FullSpaceSolver>
#include <robot/control/kinematics/IKOR>


using robot::sim::IKORTest;

using base::Vector;
using base::Orient;
using base::Time;
using base::PathName;
using base::VFile;
using base::VDirectory;
using base::externalization_error;
using base::Path;
using base::Trajectory;
using base::dom::DOMNode;
using base::dom::DOMElement;
using robot::sim::BasicEnvironment;
using robot::sim::SimulatedBasicEnvironment;
using robot::control::kinematics::FullSpaceSolver;
using robot::control::kinematics::InverseKinematicsSolver;
using robot::control::kinematics::FullSpaceSolver;
using robot::control::kinematics::IKOR;





IKORTest::IKORTest(ref<base::VFile> testSpecification,
                   ref<base::VFileSystem> fs, ref<base::Cache> cache)
  : filesystem(fs), cache(cache)
{
//  env = ref<TestBasicEnvironment>(NewObj TestBasicEnvironment(filesystem, cache, "IKORTest"));
  env = ref<SimulatedBasicEnvironment>(NewObj SimulatedBasicEnvironment(filesystem, cache, "IKORTest"));

  load(testSpecification, testSpecification->extension(),1.0);
}


void IKORTest::setEnvironment(ref<BasicEnvironment> env)
{
  if (instanceof(*env, SimulatedBasicEnvironment))
    this->env = narrow_ref<SimulatedBasicEnvironment>(env);
  else
    throw std::invalid_argument(Exception("BasicEnvironment env didn't come from IKORTest (i.e. isn't a SimulatedBasicEnvironment)"));
}


void IKORTest::saveResults(bool saveTrajFiles, PathName alternateOutputFileName)
{
  if (saveTrajFiles) {
    // ask each test to output its result to a seperate file
    for(Int i=0; i<tests.size();i++)
      tests[i].saveResult(filesystem);
  }

  // output the results of the all tests
  PathName outputName( alternateOutputFileName );
  if (outputName.empty())  //  (use name of ikortest with _result appended) 
    outputName = PathName(inputPath + PathName(getName()+"_results.xml"));
  Logln("Saving complete test specification and results to file '" << outputName.str() << "'.");
  
  ref<VDirectory> outDir( filesystem->getDirectory(outputName.path()) );
  ref<VFile> outputFile( outDir->createFile(outputName.name()) );
  save(outputFile, outputFile->extension(),1.0);
  outputFile->close();
}
  

void IKORTest::externalize(base::Externalizer& e, String format, Real version)
{
  if (format == "") format = "xml";
  
  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" v"+base::realToString(version)+" unsupported"));
  
  if (format == "xml") {
    
    if (e.isInput()) {

      inputPath = e.getArchivePath();
      
      DOMNode* context = e.context();
      
      DOMElement* ikortestElem = e.getFirstElement(context, "ikortest");

      setName( e.getDefaultedElementAttribute(ikortestElem, "name", "ikortest") );

      // get environment
      e.pushContext(ikortestElem);
      env->externalize(e, format, version);

      if (env->numRobots() == 0)
        throw externalization_error(Exception("the test environment must contain at least one robot (for testing)"));

      env->setDynamic(false); // we only do static tests
      
      // which robot (and which of its manipulators) should be used for the test
      testRobotIndex = testManipulatorIndex = 0; // default to first robot & first manipulator
      DOMElement* robotElem =  e.getFirstChildElement(ikortestElem, "testrobot",false);
      if (robotElem) {
        array<String> robotElemStrings = e.splitAtDelimiter( e.getContainedText(robotElem), ',');
        if (robotElemStrings.size()>2)
          throw externalization_error(Exception("expected '[<robotname>|<robot_env_index>],[<manipulator_name>|<manipulator_index>]' text within element 'testrobot'"));
        
        // find robot
        String robotName(e.removeChar(robotElemStrings[0],' '));
        bool found=false;
        Int index=0;
        while ((index < env->numRobots()) && !found) {
          found = (env->getRobot(index)->getRobotDescription()->getName() == robotName); 
          if (!found) index++;
        }
        if (!found) { // assume numerical index, not name
          if (e.isNumeric(robotName))
            testRobotIndex = base::stringToInt(robotName);
          else
            throw externalization_error(Exception(String("robot name specified in element 'testrobot' ('")+robotName+"'), not found in environment."));
          if (testRobotIndex >= env->numRobots())
            throw externalization_error(Exception(String("robot index specified in element 'testrobot' is out-of-range - should be [0..")+base::intToString(env->numRobots()-1)+"]."));
        }
        else
          testRobotIndex = index;

        
        // now we have the robot, get its manipulator to be tested
        ref<const RobotDescription> rd(env->getRobot(testRobotIndex)->getRobotDescription());
        if (rd->manipulators().size() == 0)
          throw externalization_error(Exception(String("robot specified in element 'testrobot' ('")+rd->getName()+"'), must have a manipulator (for testing)."));
        
        if (robotElemStrings.size()==2) {
          String manipName(e.removeChar(robotElemStrings[1],' '));
          bool found=false;
          Int index=0;
          while ((index < rd->manipulators().size()) && !found) {
            found = (rd->manipulators()[index]->getName() == manipName); 
            if (!found) index++;
          }
          if (!found) {
            if (e.isNumeric(manipName))
              testManipulatorIndex = base::stringToInt(manipName);
            else
              throw externalization_error(Exception(String("robot manipulator name specified in element 'testrobot' ('")+manipName+"'), isn't on robot '"+rd->getName()+"'."));
            if (testManipulatorIndex >= rd->manipulators().size())
              throw externalization_error(Exception(String("manipulator index specified in element 'testrobot' for robot '")+rd->getName()+"' is out-of-range - should be [0.."+base::intToString(rd->manipulators().size()-1)+"]."));
          }
          else
            testManipulatorIndex = index;
        }
        
      }

      
      // read each test
      DOMElement* testElem = e.getFirstChildElement(ikortestElem, "test");
      while (testElem) {

	Test test;
	test.externalize(e, format, version);

	tests.push_back(test);

	testElem = e.getFirstChildElement(ikortestElem, "test", false);
      }
      

      // get display related info
      displayObstacles = true; // defaults
      displayAxes = true;
      displayEEPath = false;
      displayPlatform = true;
      displayStepMod = 1;
      lookAtTarget = Vector3(0,0,0);
      alpha = -250.0;
      theta = 14.0;
      d = 9.0;
      DOMElement* displayElem =  e.getFirstChildElement(ikortestElem, "display",false);
      if (displayElem) {

        // flags
        displayObstacles = (e.getFirstChildElement(displayElem, "obstacles",false) != 0);
        displayAxes = (e.getFirstChildElement(displayElem, "axes",false) != 0);
        displayPlatform = (e.getFirstChildElement(displayElem, "platform",false) != 0);
        displayEEPath = (e.getFirstChildElement(displayElem, "eepath",false) != 0);
        DOMElement* displayStepModElem = e.getFirstChildElement(displayElem, "stepmod",false);
        if (displayStepModElem != 0) {
          displayStepMod = Int(base::stringToInt( e.getContainedText(displayStepModElem,true) ));
          if (displayStepMod == 0) displayStepMod=1;
        }
        
        // camera
        DOMElement* cameraElem = e.getFirstChildElement(displayElem, "camera",false);
        if (cameraElem) {
          lookAtTarget = e.toVector3(e.getElementAttribute(cameraElem, "target",true));
          alpha = base::stringToReal(e.getElementAttribute(cameraElem, "alpha",true));
          theta = base::stringToReal(e.getElementAttribute(cameraElem, "theta",true));
          d = base::stringToReal(e.getElementAttribute(cameraElem, "d",true));
        }
      }


      e.popContext();

      e.removeElement(ikortestElem);
    }
    else { // output

      DOMElement* ikortestElem = e.createElement("ikortest");
      e.setElementAttribute(ikortestElem,"name",getName());

      // output which robot & manipulator to test
      DOMElement* robotElem = e.createElement("testrobot",false);
      String testRobotString(  env->getRobot(testRobotIndex)->getRobotDescription()->getName() );
      if (testManipulatorIndex != 0)
        testRobotString += String(",")+env->getRobot(testRobotIndex)->getRobotDescription()->manipulators()[testManipulatorIndex]->getName();
      e.appendText(robotElem, testRobotString);
      e.appendNode(ikortestElem, robotElem);
      e.appendBreak(ikortestElem);
      
      e.pushContext(ikortestElem);
      
      // output environment
//!!! BUG - this is outputting the env state after the test, we want to reproduce the initial
// env state for output! (somehow)
      e.appendBreak(ikortestElem);
      env->externalize(e, format, version);
      e.appendBreak(ikortestElem);

      // output each test
      for(Int i=0; i<tests.size();i++) {
        tests[i].externalize(e, format, version);
        e.appendBreak(ikortestElem);
      }
      
      
      // output display related info
      DOMElement* displayElem = e.createElement("display");
      
      // flags
      if (displayObstacles) e.appendNode(displayElem, e.createElement("obstacles"));
      if (displayAxes) e.appendNode(displayElem, e.createElement("axes"));
      if (displayPlatform) e.appendNode(displayElem, e.createElement("platform"));
      if (displayEEPath) e.appendNode(displayElem, e.createElement("eepath"));
      if (displayStepMod>1) {
        DOMElement* displayStepModElem = e.createElement("stepmod",false);
        e.appendText(displayStepModElem, base::intToString(displayStepMod));
        e.appendBreak(displayElem);
        e.appendNode(displayElem, displayStepModElem);
      }
      
      // camera manipulator parameters
      DOMElement* cameraElem = e.createElement("camera");
      e.setElementAttribute(cameraElem,"target", e.toString(lookAtTarget));
      e.setElementAttribute(cameraElem, "alpha", base::realToString(alpha));
      e.setElementAttribute(cameraElem, "theta", base::realToString(theta));
      e.setElementAttribute(cameraElem, "d", base::realToString(d));
      e.appendNode(displayElem,cameraElem);
      
      e.appendNode(ikortestElem, displayElem);
      
      e.popContext();

      e.appendElement(ikortestElem);
      e.appendBreak(ikortestElem);

    }
    
  }
}







IKORTest::Test::Test(const String& name)
  : Named(name), initialConfigSpecified(false), toolAttached(false),
    timeIntervalSpecified(false), orientationControl(false), jointWeightsSpecified(false),
    resultsPresent(false), testCompleted(false), displayRangeSpecified(false)
{
}



void IKORTest::Test::saveResult(ref<base::VFileSystem> filesystem, PathName alternateOutputFileName)
{
  // write individual joint trajectory output
  PathName outputPathName( alternateOutputFileName );
  if (alternateOutputFileName.empty()) // none specified via arg 
    outputPathName = outputFileName; // so use filename specified in testspec file
  
  PathName outputPath( outputPathName.path() );
  if (!outputPathName.isAbsolute())
    outputPath = inputFilePath;// output filename is relative, place it in the testspec file's dir
  PathName outputName(outputPathName.name());

  // replace any occurance of the text '$test' with the test name
  String nameString( outputName.str() );
  Int pos = nameString.find("$test", 0);
  if (pos != String::npos ) {
    nameString.replace( pos, 5, getName() );
    outputName = PathName(nameString);
  }
  
  ref<VDirectory> outDir( filesystem->getDirectory(outputPath) );

  ref<VFile> jtrajFile( outDir->createFile(outputName) );
  jtraj = ManipulatorJointTrajectory(qs,times);
  String p(testCompleted?" ":" *partial* ");
  Logln("Saving" << p << "joint trajectory file '" << jtrajFile->pathName().str() << "'.");
  jtraj.save(jtrajFile);
  jtrajFile->close();
}
 




void IKORTest::Test::externalize(base::Externalizer& e, String format, Real version)
{
  if (format == "") format = "xml";
  
  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" v"+base::realToString(version)+" unsupported"));
  
  if (format == "xml") {
    
    if (e.isInput()) {

      DOMNode* context = e.context();
      
      DOMElement* testElem = e.getFirstChildElement(context, "test");
      setName( e.getDefaultedElementAttribute(testElem, "name", "test") );

      DOMElement* toolElem = e.getFirstChildElement(testElem, "attachedtool",false);
      if (toolElem) {
	toolAttached = true;
	toolName = e.getContainedText(toolElem);
      }
      else
	toolAttached = false;

      DOMElement* configElem = e.getFirstChildElement(testElem, "initialconfig",false);
      if (configElem) {
	initialConfigSpecified = true;
	String configText = e.getContainedText(configElem);
	initq = e.toVector(configText);
      }
      else 
	initialConfigSpecified = false;
      
      
      DOMElement* weightElem = e.getFirstChildElement(testElem, "jointweights",false);
      if (weightElem) {
	jointWeightsSpecified = true;
	String weightText = e.getContainedText(weightElem);
	jointWeights = e.toVector(weightText);
      }
      else {
	jointWeightsSpecified = false;
        jointWeights = Vector();
      }
      
      
      // get trajectory (accept either <trajectory> or <path>)
      //  also accept extra attributes for specifying the frame and possibly the time interval
      DOMElement* trajElem = e.getFirstChildElement(testElem, "trajectory",false);
      if (!trajElem) {
	trajElem = e.getFirstChildElement(testElem, "path",false);
	if (!trajElem)
	  throw externalization_error(Exception("either <trajectory> or <path> expected in 'test' element"));
      }
      
      String frameString = e.getDefaultedElementAttribute(trajElem, "frame", "");
      bool frameSpecified = (frameString != "");
      
      String maxdxString = e.getDefaultedElementAttribute(trajElem, "maxdx", "default");
      if (maxdxString != "default") {
	maxdx = base::stringToReal(maxdxString);
	maxdxSpecified = true;
      }
      else
	maxdxSpecified = false;
      

      String timeString = e.getDefaultedElementAttribute(trajElem, "timeinterval", "default");
      if (timeString != "default") {
	timeString = e.removeChar(timeString,' ');
	array<String> timeElts = e.splitAtDelimiter(timeString,':');
	if (timeElts.size() == 1) { // T - time period only 
	  timeInterval.resize(3); // time period
	  timeInterval[0] = 0;
	  timeInterval[1] = base::stringToReal(timeElts[0]);
	  timeInterval[2] = -1; // unspecified
	}
	else if (timeElts.size() == 2) {
	  timeInterval.resize(3); // S:E - time-start : end-time only
	  timeInterval[0] = base::stringToReal(timeElts[0]);
	  timeInterval[1] = base::stringToReal(timeElts[1]);
	  timeInterval[2] = -1; // unspecified
	}
	else if (timeElts.size() == 3) {
	  if (maxdxSpecified)
	    throw externalization_error(Exception(String("can't specify both the delta component of the 'timeinterval' attribute and a 'maxdx' attribute of the '<path>' or '<trajectory>' element - one or the other.")));

	  timeInterval.resize(3); // S:E:d - time-start : end-time : delta (stepsize)
	  timeInterval[0] = base::stringToReal(timeElts[0]);
	  timeInterval[1] = base::stringToReal(timeElts[1]);
	  timeInterval[2] = base::stringToReal(timeElts[2]);
	}
	else
	  throw externalization_error(Exception(String("invalid value for 'timeinterval' attribute for element '") 
						+ e.elementTagName(trajElem) + "' - must be '<starttime>:<endtime>' or '<duration>' or '<starttime>:<endtime>:<stepsize>'"));
	timeIntervalSpecified = true;
      }
      else
	timeIntervalSpecified = false;


      e.pushContext(testElem);
      traj.externalize(e, format, version);
      e.popContext();

      // if frame wasn't specified in test, use frame specified by the trajectory/path (if available)
      //  otherwise default to "world"
      if (!frameSpecified) {
	if (traj.getCoordFrame() != "") 
	  frameString = traj.getCoordFrame();
	else
	  frameString = "world";
      }

      frame = Robot::coordFrame(frameString);
      if (frame == Robot::UnknownFrame)
	throw externalization_error(Exception(String("invalid value for 'frame' attribute ") 
					      + " - must be 'ee', 'eebase', 'base', 'mount', 'platform' or 'world'."));


      // get solution parameters
      DOMElement* solnElem = e.getFirstChildElement(testElem, "solution");

      orientationControl = (e.getDefaultedElementAttribute(solnElem, "orientationcontrol", "true") == "true");

      String method = e.getDefaultedElementAttribute(solnElem, "solnmethod", "pseudoinv");
      if (method == "pseudoinv")
	solutionMethod = PseudoInverse;
      else if (method== "fullspace")
	solutionMethod = FullSpace;
      else
	throw externalization_error(Exception("the 'solnmethod' attribute of the 'solution' element must be either: 'pseudoinv' or 'fullspace'"));

      String optmethod = e.getDefaultedElementAttribute(solnElem, "optmethod", 
			     String((solutionMethod==PseudoInverse)?"pseudoinv":"lagrangian"));
      if (optmethod == "pseudoinv") {
	if (solutionMethod == FullSpace)
	  throw externalization_error(Exception("the solution method is full space parameterizaton; which is incompatible with apseudo inverse optimization method (as that is implicit via the pseudo inverse solution method)"));
	optMethod = InverseKinematicsSolver::PseudoInv;
      }
      else if (optmethod == "lagrangian") {
	if (solutionMethod == PseudoInverse)
	  throw externalization_error(Exception("the pseudo inverse solution method is incompatible with explicit Lagrangian optimization (it implicitly performs optimization using least-norm criteria))"));
	optMethod = IKOR::Lagrangian;
      }
      else if (optmethod == "bangbang") {
	throw externalization_error(Exception("Bang-bang optimization method not currently supported"));
      }
      else if (optmethod == "simplex") {
	throw externalization_error(Exception("Simplex optimization method not currently supported"));
      }
      else
	throw externalization_error(Exception("the 'optmethod' attribute of the 'solution' element must have a value from: 'pseudoinv', 'lagrangian', 'bangbang' or 'simplex'."));

      
      String criteria = e.getDefaultedElementAttribute(solnElem, "criteria", "leastnorm");
      if (criteria == "leastnorm") {
	optCriteria = IKOR::LeastNorm;
      }
      else if (criteria == "leastflow") {
	if (solutionMethod == PseudoInverse)
	  throw externalization_error(Exception("the pseudo inverse solution method implicitly provides a 'leastnorm' optimization criteria - hence is incompatible with 'leastflow'"));
	throw externalization_error(Exception("least flow optimization criteria currently unsupported"));
      }
      else
	throw externalization_error(Exception("the 'criteria' attribute of the 'solution' element must be either 'leastnorm' or 'leastflow'"));


      // constraints
      DOMElement* constElem = e.getFirstChildElement(testElem, "constraints",false);
      if (constElem) {
	DOMElement* jointElem = e.getFirstChildElement(constElem, "jointlimit",false);
	if (jointElem) optConstraints.set(IKOR::JointLimits);

	DOMElement* obstElem = e.getFirstChildElement(constElem, "obstacle",false);
	if (obstElem) optConstraints.set(IKOR::ObstacleAvoidance);

	DOMElement* accElem = e.getFirstChildElement(constElem, "acceleration",false);
	if (accElem) optConstraints.set(IKOR::Acceleration);

	DOMElement* eeimpactElem = e.getFirstChildElement(constElem, "eeimpact",false);
	if (eeimpactElem) optConstraints.set(IKOR::EndEffectorImpact);
      }

      if ((solutionMethod == PseudoInverse) && optConstraints.any())
	throw externalization_error(Exception("constraints are incompatible with the pseudo inverse solution method"));

      
      // get output file
      DOMElement* outputElem = e.getFirstChildElement(testElem, "output",false);
      String output(getName()+"_jtraj.xml"); // default
      if (outputElem) 
	output = e.getContainedText(outputElem);
      outputFileName = PathName(output);

      
      // read in any results recorded from a previous execution
      DOMElement* resultElem = e.getFirstChildElement(testElem, "result",false);
      resultsPresent = (resultElem!=0);
      if (resultsPresent) {
        testCompleted = (e.getDefaultedElementAttribute(resultElem, "completed", "true") == "true");
        if (!testCompleted)
          failureString = e.getDefaultedElementAttribute(resultElem, "failure", "unknown");
        
        // first trajectory info
        DOMElement* trajinfoElem = e.getFirstChildElement(resultElem, "trajectoryinfo");
        
        String dataString = e.getContainedText(trajinfoElem,true);
        array<String> dataLines( e.splitIntoLines(dataString) );
        dataString = "";
        
        times.clear(); qs.clear(); xs.clear(); dxs.clear(); dqs.clear();
        
        for(Int i=0; i<dataLines.size(); i++) {
          array<String> vectorStrings( e.splitAtDelimiter(dataLines[i],',') );
         
          if (!( ((i<dataLines.size()-1) && (vectorStrings.size()==5)) || ((i==dataLines.size()-1) && (vectorStrings.size()==3)) ))
            throw externalization_error(Exception("a line of 'trajectoryinfo' element doesn't contain the correct number of comma seperated vectors"));

          times.push_back( Time(base::stringToReal(vectorStrings[0])) );
          qs.push_back( e.toVector(vectorStrings[1]) );
          xs.push_back( e.toVector(vectorStrings[2]) );
          if (i!=dataLines.size()-1) {
            dxs.push_back( e.toVector(vectorStrings[3]) );
            dqs.push_back( e.toVector(vectorStrings[4]) );
          }
          
        }
        
        
        // next jacobians
        DOMElement* jacobElem = e.getFirstChildElement(resultElem, "jacobians");
        
        dataString = e.getContainedText(jacobElem,true);
        dataLines = e.splitIntoLines(dataString);
        dataString = "";
        
        Js.clear();
        
        for(Int i=0; i<dataLines.size(); i++) {
          // each line consists of a time , J matrix
          array<String> lineCompStrings( e.splitAtDelimiter(dataLines[i],',') );
          Time time( base::stringToReal(lineCompStrings[0]) );
          if (!time.equals(times[i])) 
            throw externalization_error(Exception("time " + base::realToString(time.seconds()) + " of Jacobian in element 'jacobians' doesn't match corresponding time in 'trajectoryinfo' element."));
          
          Js.push_back( e.toMatrix( lineCompStrings[1] ) );
        }
        
        if (qs.size() != Js.size())
          throw externalization_error(Exception("mismatching number step in elements 'trajectoryinfo' and 'jacobians'"));
        
      }

      
      // read any display specific info
      DOMElement* displayElem =  e.getFirstChildElement(testElem, "display",false);
      displayRangeSpecified = (displayElem!=0);
      if (displayRangeSpecified) {
        displayStartIndex = base::stringToInt(e.getDefaultedElementAttribute(displayElem, "startIndex", "0"));
        displayEndIndex = base::stringToInt(e.getDefaultedElementAttribute(displayElem, "endIndex", base::intToString(times.size()-1)));
      }
      

      e.removeElement(testElem);
    }
    else { // output

      DOMElement* testElem = e.createElement("test");
      e.setElementAttribute(testElem, "name", getName());
      
      if (toolAttached) {
        DOMElement* toolElem = e.createElement("attachedtool",false);
        e.appendText(toolElem, toolName);
        e.appendNode(testElem, toolElem);
        e.appendBreak(testElem);
      }
      
      if (initialConfigSpecified) {
        DOMElement* configElem = e.createElement("initialconfig",false);
        e.appendText(configElem, e.toString(initq));
        e.appendNode(testElem, configElem);
        e.appendBreak(testElem);
      }

      
      if (jointWeightsSpecified) {
        DOMElement* weightElem = e.createElement("jointweights",false);
        e.appendText(weightElem, e.toString(jointWeights));
        e.appendNode(testElem, weightElem);
        e.appendBreak(testElem);
      }
      
      
      // the solution element
      DOMElement* solnElem = e.createElement("solution");
      e.setElementAttribute(solnElem, "solnmethod", String((solutionMethod==FullSpace)?"fullspace":"pseudoinv"));
      if (solutionMethod!=PseudoInverse) { // method, criteria implicit for PseudoInverse
        String optMethodString;
        switch (optMethod) {
          case InverseKinematicsSolver::PseudoInv: optMethodString="pseudoinv"; break;
          case IKOR::Lagrangian: optMethodString="lagrangian"; break;
          default: Assertm(false,"unsupported optimization method");
        }
        e.setElementAttribute(solnElem, "optmethod", optMethodString);
        String optCriteriaString;
        switch (optCriteria) {
          case IKOR::LeastNorm: optCriteriaString="leastnorm"; break;
          default: Assertm(false,"unsupported optimization criteria");
        }
        e.setElementAttribute(solnElem, "criteria", optCriteriaString);
      }
      e.setElementAttribute(solnElem, "orientationcontrol", String(orientationControl?"true":"false"));
              
      e.appendNode(testElem, solnElem);
      e.appendBreak(testElem);


      // constraints
      if (solutionMethod!=PseudoInverse) { // can't have constraints for PseudoInverse
        DOMElement* constElem = e.createElement("constraints");
        bool empty = true;
        if (optConstraints.test(IKOR::JointLimits)) {
          e.appendNode(constElem, e.createElement("jointlimit",false) );
          empty = false;
        }
        if (optConstraints.test(IKOR::ObstacleAvoidance)) {
          e.appendNode(constElem, e.createElement("obstacle",false) );
          empty = false;
        }
        if (optConstraints.test(IKOR::Acceleration)) {
          e.appendNode(constElem, e.createElement("acceleration",false) );
          empty = false;
        }
        if (optConstraints.test(IKOR::EndEffectorImpact)) {
          e.appendNode(constElem, e.createElement("eeimpact",false) );
          empty = false;
        }
        
        if (!empty) 
          e.appendNode(testElem,constElem);
      }
      
      
      // output file
      DOMElement* outputElem = e.createElement("output",false);
      e.appendText(outputElem, outputFileName.str());
      e.appendNode(testElem, outputElem);
      e.appendBreak(testElem);
       
       
      // output trajectory (with extra attributes)
      e.appendBreak(testElem);
      
      e.pushContext(testElem);
      traj.externalize(e, format, version);
      e.popContext();
      
      DOMElement* trajElem = e.getFirstChildElement(testElem, "trajectory"); // get element so we can add custom attributes

      e.setElementAttribute(trajElem, "frame", Robot::coordFrame(frame)); // frame that was either in the traj or a specified override
      
      if (maxdxSpecified) 
        e.setElementAttribute(trajElem, "maxdx", base::realToString(maxdx));

      if (timeIntervalSpecified) {
        String timeString( base::realToString(timeInterval[0]) ); // period or start-time
        if (timeInterval.size() >= 2) {
          timeString += String(":")+base::realToString(timeInterval[1]); // start-time:end-time
          if ((timeInterval.size() >= 3) && (!maxdxSpecified) && (timeInterval[2]>0)) 
            timeString += String(":")+base::realToString(timeInterval[2]); // start-time:end-time:delta (stepsize)
        }
        e.setElementAttribute(trajElem, "timeinterval", timeString);
      }
      

      // output test results      
      if (resultsPresent) {
        e.appendBreak(testElem);
        DOMElement* resultElem = e.createElement("result");
        e.setElementAttribute(resultElem, "completed", testCompleted?"true":"false");
        if (!testCompleted)
          e.setElementAttribute(resultElem, "failure", failureString);
        
        // first trajectory info (sequence of time, q, x, dx, dq)
        DOMElement* trajinfoElem = e.createElement("trajectoryinfo");
        
        e.appendComment(trajinfoElem,"time, q, x, dx, dq");
        
        for(Int i=0; i<qs.size(); i++) {
          String line;
          line += base::realToString(times[i].seconds())+", ";
          line += e.toString(qs[i])+", ";
          line += e.toString(xs[i]);
          if (i<qs.size()-1) {
            line += String(", ")+e.toString(dxs[i])+", ";
            line += e.toString(dqs[i]);
          }
          e.appendText(trajinfoElem,line);
          e.appendBreak(trajinfoElem);
        }
        
        e.appendNode(resultElem, trajinfoElem);
        e.appendBreak(resultElem);
        
        // and the Jacobian at each step
        DOMElement* jacobElem = e.createElement("jacobians");
        e.appendComment(jacobElem,"time, J(q)");

        Assert(qs.size() == Js.size());
        for(Int i=0; i<Js.size(); i++) {
          String line;
          line += base::realToString(times[i].seconds())+", ";
          line += e.toString(Js[i]);
          e.appendText(jacobElem, line);
          e.appendBreak(jacobElem);
        }
        
        e.appendNode(resultElem, jacobElem);
        
        e.appendNode(testElem, resultElem);
        e.appendBreak(testElem);
        
      }
      
      // output display related info
      if (displayRangeSpecified) {
        DOMElement* displayElem = e.createElement("display");
        e.setElementAttribute(displayElem, "startIndex", base::intToString(displayStartIndex));
        e.setElementAttribute(displayElem, "endIndex", base::intToString(displayEndIndex));
        
        e.appendNode(testElem, displayElem);
        e.appendBreak(testElem);
      }


      
      
      e.appendElement(testElem);
      e.appendBreak(testElem);      

    }
  }

}
