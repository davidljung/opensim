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
  
  $Id: PathName.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.3 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/PathName>

using base::PathName;
using base::PathComponent;


PathName::PathName()
  : relative(true)
{
}

PathName::PathName(const String& name) throw(std::invalid_argument)
{
  // parse path name
  String remainder = name;

  if (remainder==currentDirectory)
    remainder="";

  if (remainder.size() == 0) 
    relative = true;
  else {
    if ((remainder.length() > 1) && remainder[remainder.length()-1] == separator)
      remainder = remainder.substr(0,remainder.length()-1);
    if (remainder[0] == separator) {
      relative = false;
      remainder = remainder.substr(1,remainder.length()-1);
    }
    else
      relative = true;
    
    while (remainder.length()>0) {
      String::size_type p = remainder.find(separator);
      if (p != String::npos) {
	String componentString = remainder.substr(0,p);
	if (componentString.length()==0) {
	  String e("path string has an empty component: "); e += name;
	  throw std::invalid_argument(Exception(e));
	}
	if (componentString != currentDirectory)
	  _path.push_back(PathComponent(componentString));
	remainder = remainder.substr(p+1,remainder.length()-componentString.length()-1);
      }
      else {
	String componentString = remainder;
	if (componentString != currentDirectory)
	  _path.push_back(PathComponent(componentString));
	remainder="";
      }
    }

  }
}


PathName::PathName(const PathName& pn)
  : relative(pn.relative), _path(pn._path)
{
}

PathName::~PathName()
{
}


const String PathName::currentDirectory = ".";
const String PathName::parentDirectory = "..";


void PathName::parent()
{
  if (!relative && (_path.size()==0))
    throw std::runtime_error(Exception("can't find parent directory of an empty relative path [too many '..'s?]"));

  if (_path.size()>0)
    _path.pop_back();
}

void PathName::canonical()
{
  PathName np;
  np+=*this;
  *this=np;
}


String PathName::str() const
{
  String pathName;
  if (!relative)
    pathName+=separator;

  PathComponents::const_iterator pc = _path.begin();
  while (pc != _path.end()) {
    pathName+=(*pc).name();
    ++pc;
    if (pc != _path.end())
      pathName+=separator;
  }
  if (pathName.length()==0)
    pathName=currentDirectory;

  return pathName;
}


PathName PathName::path() const
{
  PathName pn(*this);
  pn.canonical();
  pn._path.pop_back();
  return pn;
}

PathName PathName::name() const
{
  if (_path.size()!=0) 
    return PathName(_path.back().name());
  else
    return relative?PathName(currentDirectory):PathName();
}


String PathName::extension() const
{
  String name = PathName::name().str();

  String::size_type p = name.find(currentDirectory);
  if (p != String::npos)
    return name.substr(p+1,name.length()-p-1);
  else
    return "";
}



bool PathName::operator==(const PathName& pn) const
{
  return (relative==pn.relative) && (_path == pn._path);
}

bool PathName::operator!=(const PathName& pn) const
{
  return (relative!=pn.relative) || (_path != pn._path);
}

bool PathName::operator==(const String& pn) const
{
  return (relative==PathName(pn).isRelative()) && (_path == PathName(pn)._path);
}

bool PathName::operator!=(const String& pn) const
{
  return (relative!=PathName(pn).isRelative()) || (_path != PathName(pn)._path);
}


PathName& PathName::operator=(const PathName& pn)
{
  relative=pn.relative;
  _path=pn._path;
  return *this;
}


PathName& PathName::operator+=(const PathName& pn)
{
  if (!pn.relative) { // if second path is absolute, wipe out this
    _path.clear();
    relative=false;
  }

  PathComponents::const_iterator pc = pn._path.begin();
  while (pc != pn._path.end()) {
    if ( (*pc).name() != parentDirectory )
      _path.push_back(*pc);
    else {
      if (_path.size()>0)
	_path.pop_back();
      else
	if (relative)
	  throw std::runtime_error(Exception("can't find parent directory of an empty relative path [too many '..'s?]"));
    }
    ++pc;
  }

  return *this;
}


base::Object& PathName::clone() const
{
  return *new PathName(*this);
}
