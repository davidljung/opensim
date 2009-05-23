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
  
  $Id: Disc3.cpp 1030 2004-02-11 20:46:17Z jungd $
  $Revision: 1.1 $
  $Date: 2004-02-11 15:46:17 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <math.h>

#include <gfx/Disc3>

using gfx::Disc3;

using base::Point3;
using gfx::Segment3;


Point3 Disc3::pointClosestTo(const Point3& p) const
{
  // Algorithm from Magic Software web-site ( http://www.magic-software.com/ )
  Point3 q = p - dot(n,p-o)*n; // projection of p onto the plane
  Vector3 oq = q-o;
  Real l = oq.length();
  if (l < r) // q is within disc
    return q;
  else {
    // q falls outside disc, so closest point in on the boundary circle
    return o+(r*oq)/l;    
  }
}




Segment3 Disc3::shortestSegmentBetween(const Segment3& s) const
{
  Unimplemented;
}
