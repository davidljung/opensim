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
  
  $Id: Universe.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.4 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/Universe>

#include <base/World>
#include <base/PathName>
#include <functional>
#include <string>


using base::Universe;
using base::Simulatable;
using base::PathName;
using base::World;




Universe::Universe(ref<VFileSystem> fs, const String& resourceDirectoryName, const String& cacheDirectoryName)
  : vfilesystem(fs), renderWorldsDuringSim(true)
{
  masterCache = ref<Cache>( NewNamedObj("masterCache") Cache(fs,PathName(resourceDirectoryName), PathName(cacheDirectoryName)) );
  preSimulate();
}



Universe::~Universe()
{
}



void Universe::renderWorlds()
{
  // renderFrame() all worlds
  reflist<World>::iterator w = worlds.begin();
  reflist<World>::iterator e = worlds.end();
  while (w != e) {
    (*w)->renderFrame();
    ++w;
  }

  frameCount++;
}


void Universe::preSimulate()
{
  startRealTime = Time::now();
  frameCount=0;

  //  call all simulatables
  reflist<Simulatable>::iterator b = simulatables.begin();
  reflist<Simulatable>::iterator e = simulatables.end();
  while (b != e) {
    (*b)->preSimulate();
    ++b;
  }

}


void Universe::simulateForSimTime(const Time& dt)
{
  // simulate all simulatables
  reflist<Simulatable>::iterator current = simulatables.begin();
  reflist<Simulatable>::iterator end = simulatables.end();
  while (current != end) {
    (*current)->simulateForSimTime(dt);
    ++current;
  }
  
  simTime += dt;
  
  // Render the worlds?
  if (renderWorldsDuringSim) {
    Time elapsed(Time::now() - startRealTime);
    if ( elapsed > frameCount*Real(1.0/maxDisplayFrameRate))  { // never render faster than maxDisplayFrameRate Hz
      renderWorlds();
    }
  }
}


void Universe::simulateForRealTime(const Time& dt, Real simTimeStepSize)
{
  Time until(Time::now()+dt);
  do {
    simulateForSimTime(simTimeStepSize);
  } while (Time::now() < until);
}


void Universe::addWorld(ref<World> world)
{
  if (!contains(worlds, world)) {
    worlds.push_back(world);
  }
  else
    throw std::invalid_argument(Exception("the world has already been added"));
}


base::ref<World> Universe::removeWorld(ref<World> world)
{
  worlds.remove(world);
  return world;
}


void Universe::addSimulatable(ref<Simulatable> simulatable)
{
  if (!contains(simulatables, simulatable)) {
    simulatables.push_back(simulatable);
  }
  else
    throw std::invalid_argument(Exception("the simulatable has already been added"));
}


ref<Simulatable> Universe::removeSimulatable(ref<Simulatable> simulatable)
{
  simulatables.remove(simulatable);
  return simulatable;
}



