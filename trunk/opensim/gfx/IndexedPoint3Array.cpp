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
  
  $Id: IndexedPoint3Array.cpp 1030 2004-02-11 20:46:17Z jungd $
  $Revision: 1.2 $
  $Date: 2004-02-11 15:46:17 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <gfx/IndexedPoint3Array>
#include <gfx/TriangleIterator>

using gfx::IndexedPoint3Array;



IndexedPoint3Array::IndexedPoint3Array()
  : points(0,10), index(0,10)
{
}

IndexedPoint3Array::IndexedPoint3Array(const array<Point3>& a)
  : points(0,a.size()), index(0,a.size())
{
  for (Int p=0; p<a.size(); p++)
    addPoint(a[p]);
  points.trim();
}


IndexedPoint3Array::IndexedPoint3Array(const TriangleContainer& tc)
{
  TriangleContainer::const_iterator t = tc.begin();
  TriangleContainer::const_iterator end = tc.end();
  while (t != end) {
    const Triangle3& tri(*t);
    addPoint(tri[0]);
    addPoint(tri[1]);
    addPoint(tri[2]);
    ++t;
  }
}


IndexedPoint3Array::IndexedPoint3Array(const IndexedPoint3Array& ipa)
  : points(points), index(index)
{
}
 
IndexedPoint3Array::~IndexedPoint3Array()
{
}

Int IndexedPoint3Array::addPoint(const Point3& p)
{
  
  Int i;
  bool found = findPoint(p,i);

    // p already in the array, just add existing index
  if (found)
    index.at(index.size()) = i;
  else {
    // p not in the array.  add it and it's index
    points.at(points.size()) = p;
    index.at(index.size()) = points.size()-1;
  }
  return index[index.size()-1];
}


bool IndexedPoint3Array::findPoint(const Point3& p, Int& index)
{
  Int i=0;
  while (i < points.size()) {
    if (points[i] == p) {
      index = i;
      return true;
    }
    ++i;
  }
  return false;
}


Int IndexedPoint3Array::indexOf(const Point3& p) throw(std::invalid_argument)
{
  Int index;
  if (findPoint(p,index))
    return index;
  else
    throw std::invalid_argument(Exception("Point3 p is not a member of the array"));
}


const base::array<base::Point3>& IndexedPoint3Array::getPointArray() const
{
  return points;
}

const base::array<Int>& IndexedPoint3Array::getIndexArray() const
{
  return index;
}

