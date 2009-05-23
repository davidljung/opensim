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

  $Id: Serializable.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.1 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $

****************************************************************************/

#include <base/Serializable>

using base::Serializable;


Serializable::DerivedClassMap Serializable::derivedClasses;


void Serializable::registerSerializableInstantiator(const String& baseClassTypeName, 
						    const String& derivedClassTypeName,
						    const SerializableInstantiator& instantiator)
{
  // find the class in the map
  DerivedClassMap::iterator derivedClassIter(derivedClasses.find(derivedClassTypeName));

  // if found, check it has the same base
  if (derivedClassIter != derivedClasses.end()) {
    std::pair<const String, BaseAndInstantiatorPair>& p(*derivedClassIter);
    if (p.second.first != baseClassTypeName)
      throw std::invalid_argument(Exception(String("Class ")+derivedClassTypeName+" is already registered with a base class of "+baseClassTypeName+"."));
    p.second.second = &instantiator;
  }
  else // create a new entry
    derivedClasses[derivedClassTypeName] = BaseAndInstantiatorPair(baseClassTypeName, &instantiator);
}


const Serializable::SerializableInstantiator& Serializable::getSerializableInstantiator(const String& baseClassTypeName,
											const String& derivedClassTypeName)
{
  DerivedClassMap::iterator derivedClassIter(derivedClasses.find(derivedClassTypeName));
  if (derivedClassIter == derivedClasses.end())
    throw std::invalid_argument(Exception(String("Class ")+derivedClassTypeName+" not registered."));

  if (!baseClassTypeName.empty())
    if (derivedClassIter->second.first != baseClassTypeName)
      throw std::invalid_argument(Exception(String("Class ")+derivedClassTypeName+" was registered with base class "
					    +(derivedClassIter->second.first)+" not "+baseClassTypeName));
  return *derivedClassIter->second.second;
}

