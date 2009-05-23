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
  
  $Id: test.cpp 1028 2004-02-11 20:45:27Z jungd $
  $Revision: 1.5 $
  $Date: 2004-02-11 15:45:27 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <iostream>
#include <iomanip>

#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/ui/text/TestRunner.h>

#include <base/base>
#include <base/Application>
#include <base/Universe>

using base::Application;
using base::Universe;


int main(int argc, char *argv[])
{
  setDebugOutputID(Tmp);
  addDebugOutputID(DJ);
  addDebugOutputID(FSP);
  //addDebugOutputID(JFKE);

  bool wasSucessful = false;

  try {
    char *homeenv = getenv("OPENSIM_HOME");
    Assertm(homeenv!=0, "environment variable OPENSIM_HOME defined");
    String home(homeenv);

    Application app(home+"/resources",home+"/cache");
    ref<Universe> universe = app.universe();


    std::cout << std::setiosflags(std::ios_base::fixed) << std::setprecision(2) << std::setw(6);

    if ((argc > 1) && (String(argv[1]) == "-n")) {
      exceptionOutputEnabled(false);
      abortOnAssertionFailureEnabled(false);
    }
    else {
      exceptionOutputEnabled(true);
      abortOnAssertionFailureEnabled(true);
    }

    String testName;
    if ((argc > 2) && (String(argv[1]) == "-t")) {
      testName = String(argv[2]);
    }


    CppUnit::TextUi::TestRunner runner;
    CppUnit::TestFactoryRegistry &registry = CppUnit::TestFactoryRegistry::getRegistry();
    runner.addTest( registry.makeTest() );
    wasSucessful = runner.run( testName, false );

    Consoleln("Testing complete.");

  } catch (std::exception& e) {
    Consoleln("caught: " << String(e.what()));
  }


  base::MemoryTracer::dumpNamed();
  Consoleln("Exiting.");
  
  return wasSucessful;
}

