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
  
  $Id: ikortestrunner.cpp 1023 2004-02-11 20:42:20Z jungd $
  $Revision: 1.2 $
  $Date: 2004-02-11 15:42:20 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <robot/robot>

#include <base/Application>
#include <base/Universe>
#include <base/VFile>
#include <robot/Robot>
#include <robot/sim/BasicEnvironment>
#include <robot/sim/IKORTest>
#include <robot/sim/IKORTester>

using base::Application;
using base::Universe;
using base::PathName;
using base::VFile;
using robot::Robot;
using robot::sim::Environment;
using robot::sim::TestBasicEnvironment;
using robot::sim::IKORTest;
using robot::sim::IKORTester;



int main(int argc, char *argv[])
{
  setDebugOutputID(None);
  //setDebugOutputID(Tmp);
  addDebugOutputID(DJ);
  //addDebugOutputID(IKOR);
  addDebugOutputID(FSP);
  abortOnAssertionFailureEnabled(true);

  try {
    char *homeenv = getenv("OPENSIM_HOME");
    Assertm(homeenv!=0, "environment variable OPENSIM_HOME defined");
    String home(homeenv);

    Application app(home+"/resources",home+"/cache");
    ref<Universe> universe = app.universe();

    //std::cout << std::setiosflags(std::ios_base::fixed) << std::setprecision(2) << std::setw(6);

    // parse command-line args
    String usage("\nUsage: ikortestrunner <test_spec_file>");
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
    ref<IKORTest> test(NewObj IKORTest(testSpecFile, app.filesystem(), universe->cache()));
    
    ref<IKORTester> tester(NewObj IKORTester(app.filesystem(), universe->cache()));

    
    ref<Environment> env(test->getEnvironment());

    // add environment to Universe
    universe->addSimulatable(env);
    universe->enableSimulationRendering(false);
    universe->preSimulate();

    // run tests & save results
    tester->executeTests(test,true);

  } catch (std::exception& e) {
    Consoleln("caught: " << String(e.what()));
  }


  Consoleln("Exiting.");
  
  return 0;
}
