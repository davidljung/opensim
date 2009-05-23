/****************************************************************************
  Copyright (C)1996 David Jung <opensim@pobox.com>

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

  $Id: Application.cpp 1093 2004-09-13 17:38:26Z jungd $

****************************************************************************/

#include <base/Application>

#include <iostream>
#include <iomanip>
#include <string>

#include <base/Universe>
#include <base/StdFileSystem>
#include <base/Time>


using base::Application;

Application* Application::app=0;



Application::Application(const String& resourceDirectoryName, const String& cacheDirectoryName)
{
  if (app!=0)
    throw std::runtime_error("Application::Application - Application is a singleton, only one can exist");
  else
    app=this;

  // turn on exceptions & set options for std IO streams
  std::cout.exceptions(std::ios::badbit | std::ios::failbit );
  std::cin.exceptions(std::ios::badbit | std::ios::failbit );

  theTime = new Time();
  theFileSystem = ref<VFileSystem>( NewNamedObj("StdFileSystem") StdFileSystem());
  theUniverse = ref<Universe>(NewNamedObj("Universe") Universe(theFileSystem,resourceDirectoryName,cacheDirectoryName));
}


Application::~Application()
{
  delete theTime;
}


void Application::displayHeader(const String& appName)
{
  if (appName == "") {
    Consoleln("OpenSim v" << getVersion());
    Consoleln(" (C)1996-2004 David Jung. Licensed under the GNU General Public License.");
  }
  else {
    Consoleln(""+appName+" (OpenSim v" << getVersion() << " GPL Licensed)");
  }
}

