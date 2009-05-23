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
  
  $Id: Line3.cpp 1030 2004-02-11 20:46:17Z jungd $
  $Revision: 1.2 $
  $Date: 2004-02-11 15:46:17 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <math.h>

#include <gfx/Line3>

using gfx::Line3;

using base::Point3;
using gfx::Segment3;


Segment3 Line3::shortestSegmentBetween(const Line3& l2) const
{
  // minimize the squared dist function
  //  q(s,t) = | l1(s) - l2(t) |^2 where l1(s) = o+s*d & l2(t) = l2.o+t*l2.d
  // which is quadratic with determinant always >= 0  (0 => parallel lines)
  // can be written q(s,t) = as^2 + 2bst + ct^2 +2ds +2et + f
  //  where
  const Line3& l1(*this); // alias 
  Real a = dot(l1.d,l1.d);
  Real b = dot(-l1.d,l2.d);
  Real c = dot(l2.d,l2.d);
  Real d = dot(l1.d,o-l2.o);
  Real e = dot(-l2.d,o-l2.o);
  // & f = dot(o-l2.o,o-l2.o);
  
  Real det = a*c - b*b;
  Real s,t;
  if (Math::equals(det,0)) { // parallel
    t = 0; s = -d/a; // choose
  }
  else { // solution for dq = (0,0)
    Real acb2 = a*c - b*b;
    s = (b*e - c*d) / acb2;
    t = (b*d - a*e) / acb2;
  }
  
  return Segment3( o+s*l1.d, l2.o+t*l2.d );
}
