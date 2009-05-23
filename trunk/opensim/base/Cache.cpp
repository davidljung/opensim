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
  
  $Id: Cache.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.3 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/Cache>
#include <base/VFile>
#include <base/VDirectory>
#include <base/CacheDirectory>
#include <base/MD5>

using base::Cache;
using base::ResourceCache;
using base::VFile;
using base::VDirectory;
using base::CacheDirectory;


Cache::Cache(ref<VFileSystem> fs, const PathName& resourceDirectory, const PathName& cacheDirectory)
  : ResourceCache(fs, resourceDirectory, cacheDirectory)
{
}


Cache::~Cache()
{
}


ref<VFile> Cache::getFile(const PathName& name)  throw (path_not_found, io_error)
{
  return filesystem->getFile( name );
}


ref<VFile> Cache::findFile(const PathName& name, const PathName& additionalPath) throw (path_not_found, io_error)
{
  const PathName& pn(name);
  ref<VFile> file;
  if (pn.isRelative()) {
    // try resource directory first, if not found, try the current directory, then additionalPath
    if (filesystem->exists( _resourceDirectory+pn ))
      file = filesystem->getFile( _resourceDirectory+pn );
    if (!additionalPath.empty())
      if (filesystem->exists(additionalPath+pn))
	file = filesystem->getFile(additionalPath+pn);
    if (!file)
      file = filesystem->getFile(pn);
  }
  else 
    file = filesystem->getFile(pn); 
  
  return file;
}


bool Cache::isCached(const PathName& name) throw (path_not_found, io_error)
{
  return !getCache(name)->empty(); // not wonderfully efficient, but it'll do
}

ref<VDirectory> Cache::getCache(const PathName& name) throw (path_not_found, io_error)
{
  // Construct CacheDirectory for this file resource.
  //  It consists of a directory containing a file for each cache file associated
  //  with the real file (cache files can contain anything the caching clients wishs to
  //  associate with a file).
  //  The implementation is to store files in the cache directory that have a prefix
  //   that is the MD5 hash of the file resource, and has a suffix that is used as
  //   the name visible through the CacheDirectory.

  ref<VFile> file = getFile(name);

  MD5 md5(file->istream());
  String prefix( md5.hex_digest());
  prefix = String("md5_")+prefix+"_";

  ref<VDirectory> cache = filesystem->getDirectory(_cacheDirectory);
  ref<CacheDirectory> cacheDir( NewObj CacheDirectory(cache,prefix) );

  return cacheDir;
}


