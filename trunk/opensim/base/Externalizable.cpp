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

  $Id: Externalizable.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.4 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $

****************************************************************************/

#include <base/Externalizable>

#include <base/Externalizer>
#include <base/externalization_error>

using base::Externalizable;
using base::Externalizer;


void Externalizable::load(ref<VFile> archive, const String& format, Real version)
{
  Externalizer e(Input, archive);
  externalize(e, format, version);
}


void Externalizable::save(ref<VFile> archive, const String& format, Real version)
{
  Externalizer e(Output, archive);
  externalize(e, format, version);
  e.flush();
}         

void Externalizable::externalize(Externalizer& e, String format, Real version) const
{
  if (e.isInput())
    throw base::externalization_error(Exception("can't input to a const Externalizable"));
  
  // cast this to non-const, as we shouldn't change the value as e is Output
  Externalizable* self( const_cast<Externalizable*>(this) );
  self->externalize(e, format, version);
}

