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
  
  $Id: File.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.3 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/File>

using base::File;
using base::VFile;
using base::PathName;


File::File(const PathName& pathname) throw(path_not_found, io_error)
  : VFile(pathname), opened(false)
{
}


File::File(const String& pathname) throw(path_not_found, io_error, std::invalid_argument)
  : VFile(PathName(pathname)), opened(false)
{
}
    

File::~File()
{
  close();
}


std::istream& File::istream() const throw(io_error)
{
  return iostream(std::iostream::in);
}


std::ostream& File::ostream() const throw(io_error)
{
  return iostream(std::iostream::out);
}


std::iostream& File::iostream(std::iostream::openmode mode) const throw(io_error)
{
  if (!opened) 
    open(mode);
  else 
    if ((this->mode & mode) != mode) {
      close();
      open(mode);
    }
  return *fileStream;
}


void File::close() const throw(io_error)
{
  if (opened) {
    if (mode & std::iostream::out) (*fileStream) << std::flush;
    fileStream->close();
    delete fileStream;
    opened=false;
  }
}


void File::open(std::iostream::openmode mode) const throw(path_not_found, io_error)
{
  if (!opened) {
    fileStream = new std::fstream( pathname.str().c_str(), mode);
    if (!(*fileStream))
      throw io_error(Exception(String("can't open file ")+pathname.str()));
    opened=true;
    this->mode = mode;
  }
}
