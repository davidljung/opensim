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
  
  $Id: Directory.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.4 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/Directory>

#include <base/File>

extern "C" {
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <errno.h>
}

using base::Directory;
using base::File;
using base::VEntry;
using base::VDirectory;
using base::VFile;
using base::PathName;
using base::path_not_found;



Directory::Directory(ref<const Directory> parent)
  : loaded(false), parent(parent)
{
}


Directory::Directory(const PathName& pathname, ref<const Directory> parent)
  : VDirectory(pathname), loaded(false), parent(parent)
{
}

Directory::Directory(const Directory& d)
  : VDirectory(d), loaded(false), parent(d.parent)
{
}

Directory::~Directory()
{
}

void Directory::operator=(const Directory& d)
{
  pathname=d.pathname;
  parent=d.parent;
  if (loaded) 
    entries.clear();
  loaded=false;
}


void Directory::load() const
{
  if (!loaded) {
    entries.clear();
    DIR* d = opendir(pathname.str().c_str());

    if (d==0) {
      if ((errno == ENOENT) || (errno == ENOTDIR))
	throw path_not_found(Exception(String("directory not found: ")+pathname.str()));
      else
	throw io_error(Exception(String("can't open directory: ")+pathname.str()));
    }

    ref<const Directory> self(this);
    struct dirent* dentry;
    struct stat statbuf;
    do {

      dentry = readdir(d);
      if (dentry!=0) {
	String name(dentry->d_name);

	if ((name != PathName::currentDirectory) && (name != PathName::parentDirectory)) { // ignore "." and ".." 
	  
	  PathName entryPathName(pathname); entryPathName+=PathName(name);
	  if ( stat( entryPathName.str().c_str(), &statbuf) == 0) { // if we can't stat it, skip it
	    try {
	      ref<VEntry> newEntry;

	      if ( S_ISREG(statbuf.st_mode) ) newEntry = ref<File>( NewObj File(entryPathName) );
	      if ( S_ISDIR(statbuf.st_mode) ) newEntry = ref<Directory>( NewObj Directory(entryPathName,self) );
	      
	      if (newEntry!=0) entries.push_back(newEntry);
	    } catch (path_not_found&) {}
	  }
	}

      }

    } while (dentry!=0);

    closedir(d);
    loaded=true;
  }
}


ref<VEntry> Directory::find(const String& name) const
{
  load();
  bool found=false;
  VEntries::iterator it = entries.begin();
  while ((it != entries.end()) && !found) {
    if ( (*it)->name() == name )
      return (*it);
    ++it;
  }
  return ref<VEntry>(0);
}




bool Directory::contains(ref<const VEntry> entry) const
{
  if (entry->path() != pathName()) return false;
  return ( find(entry->name().str()) != 0);
}


bool Directory::contains(const PathName& name) const
{
  if (name.str() == PathName::currentDirectory) return true;
  if ((name.str() == PathName::parentDirectory) && (parent !=0)) return true;
  return ( find(name.str()) != 0 );
}


base::ref<VFile> Directory::file(const PathName& name) const throw(path_not_found)
{
  ref<VEntry> e = find(name.str());
  if (e==0) throw path_not_found(Exception(String("file '")+name.str()+"' doesn't exist in directory: '"+pathname.str()+"'"));
  if (e->isDirectory()) throw path_not_found(Exception(String("'")+name.str()+"' is a directory"));
  return narrow_ref<VFile>(e);
}

base::ref<VDirectory> Directory::directory(const PathName& name) const throw(path_not_found)
{
  if (name.str() == PathName::currentDirectory) 
    return ref<VDirectory>( NewObj Directory(*this) );
  if ((name.str() == PathName::parentDirectory) && (parent!=0)) 
    return ref<VDirectory>( NewObj Directory(*parent) );

  if (!name.isSimple()) 
    throw path_not_found(Exception(String("name '")+name.str()+"' must be simple; not a full path"));

  ref<VEntry> e = find(name.str());
  if (e==0) throw path_not_found(Exception(String("directory '")+name.str()+"' doesn't exist in directory: '"+pathName().str()+"'"));
  if (!e->isDirectory()) throw path_not_found(Exception(String("'")+name.str()+"' is a file"));
  return narrow_ref<VDirectory>(e);
}


ref<VFile> Directory::createFile(const PathName& name) const throw(io_error)
{
  PathName pn(name);
  if (!pn.isRelative() || pn.path() != PathName::currentDirectory)
    throw io_error(Exception(String("'")+name.str()+"' must be a simple file name, not a path"));
  pn = pathname+pn;

  // create it
  int fd = creat(pn.str().c_str(), S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH);

  if (fd == -1)
    throw io_error(Exception(String("error creating file '")+name.str()+"' in '"+pathName().str()+"': "+strerror(errno)));
  close(fd);


  // re-load directory (probably a better way)
  loaded=false;
  load();

  return file(name);
}

void Directory::deleteFile(const PathName& name) const throw(path_not_found, io_error)
{
  ref<VEntry> e = find(name.str());
  if (e==0) throw path_not_found(Exception(String("file '")+name.str()+"' doesn't exist in this directory"));
  if (e->isDirectory()) throw path_not_found(Exception(String("'")+name.str()+"' is a directory"));

  int ret = unlink(e->pathName().str().c_str());
  if (ret == -1)
    throw io_error(Exception(String("error deleting file '")+name.str()+"' in '"+pathName().str()+"': "+strerror(errno)));

  // re-load directory (probably a better way)
  loaded=false;
  load();
}

ref<VDirectory> Directory::createDirectory(const PathName& name) const throw(io_error)
{
  PathName pn(name);
  if (pn.isRelative() || pn.path() != PathName::currentDirectory)
    throw io_error(Exception(String("'")+name.str()+"' must be a simple directory name, not a path"));
  pn = pathname+pn;

  // create it
  int ret = mkdir(pn.str().c_str(), S_IRWXU|S_IRWXG|S_IRWXO);

  if (ret == -1) 
    throw io_error(Exception(String("error creating directory '")+name.str()+"' in '"+pathName().str()+"': "+strerror(errno)));

  // re-load directory (probably a better way)
  loaded=false;
  load();

  return directory(name);
}

void Directory::deleteDirectory(const PathName& name) const throw(path_not_found, io_error)
{
  if (name.str() == PathName::currentDirectory) 
    throw io_error(Exception("can't delete 'this' directory (delete it from parent)"));
  if (name.str() == PathName::parentDirectory)
    throw io_error(Exception("can't delete parent directory (delete it from parent's parent)"));

  if (!name.isSimple()) 
    throw path_not_found(Exception("name must be simple; not a full path"));

  ref<VEntry> e = find(name.str());
  if (e==0) throw path_not_found(Exception(String("directory '")+name.str()+"' doesn't exist in this directory"));
  if (!e->isDirectory()) throw path_not_found(Exception(String("'")+name.str()+"' is a file"));

  int ret = unlink(e->pathName().str().c_str());
  if (ret == -1)
    throw io_error(Exception(String("error deleting directory '")+name.str()+"' in '"+pathName().str()+"': "+strerror(errno)));

  // re-load directory (probably a better way)
  loaded=false;
  load();
}





base::ref<VEntry> Directory::entry(Int i) const
{
  if (!loaded) load();
  return entries[i];
}

Int Directory::size() const 
{
  if (!loaded) load();
  return entries.size();
}
