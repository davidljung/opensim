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
  
  $Id: TriangleIterator.cpp 1030 2004-02-11 20:46:17Z jungd $
  $Revision: 1.2 $
  $Date: 2004-02-11 15:46:17 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <gfx/TriangleIterator>
#include <gfx/TriangleContainer>
#include <gfx/TriangleIteratorState>

using gfx::TriangleContainer;
using gfx::TriangleIterator;
using gfx::TriangleIteratorState;

TriangleIterator::TriangleIterator(const TriangleIterator& iter) 
  : triangles(iter.triangles), 
    iteratorState( narrow_cast<TriangleIteratorState>(&iter.iteratorState->clone()) )
{
}

TriangleIterator::~TriangleIterator() 
{
  triangles->deleteTriangleIteratorState(iteratorState); 
}


TriangleIterator::TriangleIterator(const TriangleContainer& triangles, bool begin)
  : triangles(&triangles)
{
  iteratorState = triangles.newTriangleIteratorState(begin); 
}


TriangleIterator::reference TriangleIterator::operator*() const
{
  return **iteratorState; 
}

TriangleIterator::pointer   TriangleIterator::operator->() const
{
  return &(**iteratorState); 
}


TriangleIterator& TriangleIterator::operator++()
{
  triangles->nextTriangle(iteratorState);
  return *this; 
}


bool TriangleIterator::operator==(const TriangleIterator& i) const 
{
  if (triangles != i.triangles) return false;

  return triangles->equalStates(iteratorState, i.iteratorState);
}


TriangleIterator& TriangleIterator::operator=(const TriangleIterator& ti)
{
  triangles = ti.triangles;
  iteratorState = narrow_cast<TriangleIteratorState>(&ti.iteratorState->clone());
  return *this;
}
