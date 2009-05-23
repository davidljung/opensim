/****************************************************************************
  Copyright (C)2003 David Jung <opensim@pobox.com>

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
  
  $Id: Quad3.cpp 1030 2004-02-11 20:46:17Z jungd $
  $Revision: 1.1 $
  $Date: 2004-02-11 15:46:17 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <math.h>

#include <gfx/Quad3>

#include <gfx/Plane>

using gfx::Quad3;
using gfx::Point3;
using gfx::Segment3;
using gfx::Triangle3;
using gfx::Plane;


Quad3::Quad3(const Point3& c1, const Point3& c2, const Point3& c3, const Point3& c4)
  : c1(c1), c2(c2), c3(c3), c4(c4) 
{
  // check coplanar
  Plane p(c1,c2,c3);
  Assertm(p.contains(c4),"corners are coplanar");
}  


Real Quad3::distanceTo(const Point3& p) const
{
  return (p-pointClosestTo(p)).length();
}


Point3 Quad3::pointClosestTo(const Point3& p) const
{
  Triangle3 t1(c1,c2,c3);
  Triangle3 t2(c3,c4,c1);
  
  Point3 p1 = t1.pointClosestTo(p);
  Point3 p2 = t2.pointClosestTo(p);
  
  if ((p-p1).norm() < (p-p2).norm())
    return p1;
  else
    return p2;
}

  
Segment3 Quad3::shortestSegmentBetween(const Segment3& s) const
{
  Triangle3 t1(c1,c2,c3);
  Triangle3 t2(c3,c4,c1);

  Segment3 s1 = t1.shortestSegmentBetween(s);
  Segment3 s2 = t2.shortestSegmentBetween(s);
  
  if (s1.norm() < s2.norm())
    return s1;
  else
    return s2;
}
 
 
Real Quad3::distanceTo(const Segment3& s) const
{
  return shortestSegmentBetween(s).length();
}

  
Segment3 Quad3::shortestSegmentBetween(const Triangle3& t) const
{
  Triangle3 t1(c1,c2,c3);
  Triangle3 t2(c3,c4,c1);
  
  Segment3 s1 = t1.shortestSegmentBetween(t);
  Segment3 s2 = t2.shortestSegmentBetween(t);

  if (s1.norm() < s2.norm())
    return s1;
  else
    return s2;
}

  
Real Quad3::distanceTo(const Triangle3& t) const
{
  return shortestSegmentBetween(t).length();
}


Segment3 Quad3::shortestSegmentBetween(const Quad3& q) const
{
  Triangle3 t1(c1,c2,c3);
  Triangle3 t2(c3,c4,c1);
  
  Segment3 s1 = q.shortestSegmentBetween(t1);
  Segment3 s2 = q.shortestSegmentBetween(t2);

  if (s1.norm() < s2.norm())
    return s1;
  else
    return s2;
}


Real Quad3::distanceTo(const Quad3& q) const
{
  return shortestSegmentBetween(q).length();
}
