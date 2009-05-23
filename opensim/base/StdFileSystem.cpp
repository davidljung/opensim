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
  
  $Id: StdFileSystem.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.4 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/StdFileSystem>

#include <unistd.h>

#include <base/Directory>
#include <base/File>

using base::StdFileSystem;
using base::VFileSystem;
using base::VDirectory;
using base::VFile;
using base::Directory;
using base::File;
using base::ref;


StdFileSystem::StdFileSystem()
{
  try {
    char* cwdbuf = new char[512];
    if (getcwd(cwdbuf, 512) == 0) {
      cwdbuf = new char[8192];
      if (getcwd(cwdbuf, 8192) == 0) {
	delete[] cwdbuf;
	throw std::runtime_error(Exception("current directory path is too long"));
      }
    }
    currentDir = narrow_ref<Directory>(getDirectory(String(cwdbuf)));
    delete[] cwdbuf;
  }
  catch (std::exception&) {
    currentDir = narrow_ref<Directory>(root());
  }
}

StdFileSystem::~StdFileSystem()
{
}


ref<VDirectory> StdFileSystem::root()
{
  ref<Directory> dir( NewObj Directory(String()+PathName::separator,ref<Directory>(0)) );
  return dir;
}

ref<VDirectory> StdFileSystem::temp()
{
  return getDirectory(String()+PathName::separator+"tmp");
}

ref<VDirectory> StdFileSystem::current()
{
  return currentDir;
}

void StdFileSystem::setCurrent(const PathName& path) throw (path_not_found, io_error, std::invalid_argument)
{
  PathName pn(path);
  if (pn.isRelative())
    pn = PathName(currentDir->pathName())+pn;

  currentDir = narrow_ref<Directory>(getDirectory(pn));
  
  if (chdir(currentDir->pathName().str().c_str()) == -1)
      throw io_error(Exception(String("error change current directory to '")+currentDir->pathName().str()+"': "+strerror(errno)));

}



ref<VDirectory> StdFileSystem::getDirectory(const PathName& path) throw (path_not_found, io_error, std::invalid_argument)
{
  const PathName& pn(path);
  Int first = 0;
  ref<VDirectory> d = root();
  if (pn.isRelative())
    d = current();

  if (pn.size() > 0) {
    if (pn[0] == PathName::currentDirectory) {
      d = current();
      first = 1;
    }
    if (pn[0] == PathName::parentDirectory) {
      d = current()->directory(PathName::parentDirectory);
      first = 1;
    }
  }

  for(Int pc=first; pc<pn.size(); pc++) {
    try {
      d = d->directory(pn[pc]);
    } catch (path_not_found&) {
      throw path_not_found(Exception(String("'")+path.str()+"' is an invalid path: '"+pn[pc]+"' doesn't exist in '"+d->pathName().str()+"'"));
    }
  }

  return d;
}



ref<VFile> StdFileSystem::getFile(const PathName& path) throw (path_not_found, io_error, std::invalid_argument)
{
  const PathName& pn(path);
  ref<VDirectory> dir;
  try {
    dir = getDirectory(pn.path());
  } catch (path_not_found& e) {
    throw path_not_found(Exception(String("'")+path.str()+"' file not found: "+e.what()));
  }
  return dir->file(pn.name());
}


bool StdFileSystem::exists(const PathName& path) throw (io_error)
{
  const PathName& pn(path);
  Int first = 0;
  ref<VDirectory> d = root();
  if (pn.isRelative())
    d = current();

  if (pn.size() > 0) {
    if (pn[0] == PathName::currentDirectory) {
      d = current();
      first = 1;
    }
    if (pn[0] == PathName::parentDirectory) {
      d = current()->directory(PathName::parentDirectory);
      first = 1;
    }
  }

  if (pn.size() > 0) {
    for(Int pc=first; pc<pn.size()-1; pc++) {
      if (d->contains(pn[pc]))
        d = d->directory(pn[pc]);
      else
        return false;
    }
  }
  else
    return false;

  if (!d->contains(pn[pn.size()-1]))
    return false;

  return true;
}

