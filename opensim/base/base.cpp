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
  
  $Id: base.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.7 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
  
****************************************************************************/

#include <base/base>

#include <sstream>
#include <iostream>

#ifdef __GNUC__ 
// get namespace abi for className() 
#include <cxxabi.h>
#endif

std::ostream& base::_Debug = std::cerr;
std::ostream& base::_Log = std::cout;
std::ostream& base::_Console = std::cout;


#ifdef DEBUG
bool _abortOnExceptionConstruction = false;
bool _outputExceptionOnConstruction = true;
bool _abortOnAssertionFailure = false;
#endif


void base::assertionFailure(const base::String& errorstring)
{
#ifdef DEBUG
  _Log << "Throwing runtime_error due to assertion failure: " << errorstring << std::endl;
  if (_abortOnAssertionFailure) {
    _Log << "aborting." << std::endl;
    abort();
  }
#endif
  throw std::runtime_error(errorstring);
}


String base::demangleTypeidName(const String& typeidName)
{
#ifdef __GNUC__
  int status;
  char* realname = abi::__cxa_demangle(typeidName.c_str(), 0, 0, &status);
  String srealname(realname);
  free(realname);
  return srealname;
#else
  throw std::runtime_error(Exception("demangling not implemented on the current platform"));
#endif
}
 

String base::className(const std::type_info& ti)
{
  return demangleTypeidName(ti.name());
}


String base::intToString(Int i)
{
  std::ostringstream oss;
  oss << i;
  return oss.str();
}


Int base::stringToInt(const String& s)
{
  std::istringstream iss(s);
  Int v;
  iss >> v;
  return v;
}

Real base::stringToReal(const String& s)
{
  std::istringstream iss(s);
  Real v;
  iss >> v;
  return v;
}

String base::realToString(Real r)
{
  std::ostringstream oss;
  oss << r;
  return oss.str();
}



