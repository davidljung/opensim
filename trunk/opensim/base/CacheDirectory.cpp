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
  
  $Id: CacheDirectory.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.3 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/CacheDirectory>

#include <base/File>

extern "C" {
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <errno.h>
}

using base::CacheDirectory;
using base::VEntry;
using base::VDirectory;
using base::VFile;
using base::PathName;


CacheDirectory::CacheDirectory(ref<VDirectory> actualDir, const String& prefix)
  : VDirectory(String()+PathName::separator), 
    actualDir(actualDir), prefix(prefix), loaded(false)
{
  
}


CacheDirectory::CacheDirectory(const CacheDirectory& d)
  : VDirectory(d), 
    actualDir(d.actualDir), prefix(d.prefix), loaded(false)
{
}

CacheDirectory::~CacheDirectory()
{
}


void CacheDirectory::load() const
{
  if (!loaded) {
    // find all the files in the actualDir and add entries for
    //  all starting with the prefix (whose names are the remainder of the 
    //  actual name after removing the prefix)
    entries.clear();
    VDirectory::const_iterator f = actualDir->begin();
    VDirectory::const_iterator end = actualDir->end();
    while (f != end) {
      ref<VEntry> ent = *f;
      
      if (!ent->isDirectory()) { // only interested in files
	ref<VFile> file( narrow_ref<VFile>(ent) );
	String name(file->name().str());
	
	// starts with prefix?
	if (name.find(prefix,0)==0) {
	  String suffix(name.substr(prefix.length(),name.length()-prefix.length()));
	  ref<CacheFile> cfile( NewObj CacheFile(suffix,file) ); // add it
	  entries.push_back(cfile);
	}
	
      }
      ++f;
    }
  }
  loaded = true;
}


ref<VEntry> CacheDirectory::find(const String& name) const
{
  load();
  bool found=false;
  CacheFileEntries::iterator it = entries.begin();
  while ((it != entries.end()) && !found) {
    if ( (*it)->name() == name )
      return (*it);
    ++it;
  }
  return ref<VEntry>(0);
}


void CacheDirectory::operator=(const CacheDirectory& d)
{
  VDirectory::operator=(d);
  actualDir = d.actualDir;
  prefix = d.prefix;
  loaded=false;
}



bool CacheDirectory::contains(ref<const VEntry> entry) const
{
  if (entry->path() != String()+PathName::separator) return false; // CacheDirectories are all root (e.g. "/")
  return ( find(entry->name().str()) != 0);
}

bool CacheDirectory::contains(const PathName& name) const
{
  if (name.str() == PathName::currentDirectory) return true;
  if (name.str() == PathName::parentDirectory)  return false;
  return ( find(name.str()) != 0 );
}



base::ref<VFile> CacheDirectory::file(const PathName& name) const throw(path_not_found)
{
  ref<VEntry> e = find(name.str());
  if (e==0) throw path_not_found(Exception(String("file '")+name.str()+"' doesn't exists in this directory: "+pathname.str()));
  return narrow_ref<VFile>(e);
}

base::ref<VDirectory> CacheDirectory::directory(const PathName& name) const throw(path_not_found)
{
  throw path_not_found(Exception("A CacheDirectory can only contain files"));
}

ref<VFile> CacheDirectory::createFile(const PathName& name) const throw(io_error)
{
  const PathName& pn(name);
  if (!pn.isRelative() || pn.path() != PathName::currentDirectory)
    throw io_error(Exception(String("'")+name.str()+"' must be a simple file name, not a path"));

  String fullname = prefix+name.str();
  loaded=false;
  return actualDir->createFile(fullname);
}

void CacheDirectory::deleteFile(const PathName& name) const throw(path_not_found, io_error)
{
  ref<VEntry> e = find(name.str());
  if (e==0) throw path_not_found(Exception(String("file '")+name.str()+"' doesn't exists in this directory"));
  ref<CacheFile> file = narrow_ref<CacheFile>(e);
  actualDir->deleteFile(file->actualFile->name());
  loaded=false;
}

ref<VDirectory> CacheDirectory::createDirectory(const PathName& name) const throw(io_error)
{
  throw io_error(Exception(String("error creating directory '")+name.str()+"': CacheDirectory can only contain files."));
}

void CacheDirectory::deleteDirectory(const PathName& name) const throw(path_not_found, io_error)
{
  throw path_not_found(Exception(name.str()+" doesn't exist in directory '"+pathName().str()+"' (CacheDirectory can only contain files)"));
}



base::ref<VEntry> CacheDirectory::entry(Int i) const
{
  if (!loaded) load();
  return entries[i];
}

Int CacheDirectory::size() const 
{
  if (!loaded) load();
  return entries.size();
}



