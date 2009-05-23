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
  
  $Id: TriangleContainer.cpp 1030 2004-02-11 20:46:17Z jungd $
  $Revision: 1.2 $
  $Date: 2004-02-11 15:46:17 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <gfx/TriangleContainer>

#include <gfx/TriangleIterator>

using gfx::TriangleContainer;
using gfx::TriangleIterator;


TriangleContainer::const_iterator TriangleContainer::begin() const
{
  return TriangleIterator(*this, true); 
}

TriangleContainer::const_iterator TriangleContainer::end() const 
{
  return TriangleIterator(*this, false); 
}


