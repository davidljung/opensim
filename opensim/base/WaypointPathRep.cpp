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
  
  $Id: WaypointPathRep.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.5 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/WaypointPathRep>

#include <base/Serializer>

using base::WaypointPathRep;




base::Point3 WaypointPathRep::position(Real s) const
{
  if (flags & ConstPos) 
    return points[0];

  Int i=findIndex(s);
  if (equals(s,si[i])) // fast path if s corresponds to a waypoint
    return points[i];

  Vector3 v( points[i+1] - points[i] );
  Real ts( si[i+1] - si[i] ); // change in s between points[i] to points[i+1]

  Real ds = s-si[i]; 
  Vector3 dv( (ds/ts)*v );

  return points[i] + dv;
}


base::Orient WaypointPathRep::orientation(Real s) const
{
  if (flags & ConstOrient)
    return orients[0];

  Int i=findIndex(s);
  if (Math::equals(s,si[i])) // fast path if s corresponds to a waypoint
    return orients[i];

  // analogous to position, but using Quat4::interpolate()
  Quat4 q1( orients[i].getQuat4() );
  Quat4 q2( orients[i+1].getQuat4() );
  Real ts( si[i+1] - si[i] );

  Real ds = s-si[i];
  Quat4 q( Quat4::interpolate(q1,q2,ds/ts) );

  return Orient(q);
}


Real WaypointPathRep::distinguishedValue(Int i) const
{
  if ((flags & ConstPos) && (flags && ConstOrient)) {
    Math::bound<Int>(i,0,1); 
    return (i==0)?0.0:1.0;
  }
  
  Math::bound<Int>(i,0,si.size()-1); 
  return si[i];
}


Int WaypointPathRep::numDistinguishedValues() const
{
  if ((flags & ConstPos) && (flags && ConstOrient))
    return 2;

  return si.size();
}




void WaypointPathRep::computeSis()
{
  if (points.size() < 2)  flags |= ConstPos;
  if (orients.size() < 2) flags |= ConstOrient;

  // ensure at least one point
  if (points.size() == 0) points.at(0) = Point3();
  if (orients.size() == 0) orients.at(0) = Orient();

  // if not const position, s's range maps to spatial path length
  if (!(flags & ConstPos)) {

    // scan path and assign the incremental path length to si[]
    //  (it will be divided by the total path length next)
    Real pathLength = 0;
    Int i=1;
    si.resize(points.size());
    si[0] = 0;
    while (i<points.size()) {
      pathLength += (points[i] - points[i-1]).length();
      si[i] = pathLength;
      i++;
    }

    if (pathLength > 0)
      for(Int i=0; i<points.size(); i++) 
	si[i] /= pathLength;

  }
  else if (!(flags & ConstOrient)) {
    // if position is constant, but orientation is not, s's range maps to angular path length
    
    Real pathLength = 0;
    Int i=1;
    si.resize(orients.size());
    si[0]=0;
    while (i<orients.size()) {
      // we calculate the angle between two Quats in a round about way - there
      //  is probably a better one
      // transform a unit x-axis vector by each Quat and then use the dot product to get
      // and angle between them
      Quat4 from(orients[i-1].getQuat4());
      Quat4 to(orients[i].getQuat4()); 
      pathLength += Quat4::angleBetween(from,to);
      si[i] = pathLength;
      i++;
    }

    if (pathLength > 0)
      for(Int i=0; i<orients.size(); i++)
	si[i] /= pathLength;
  }

  // if both position & orientation are constant, ti is not used and all position
  //  and orientation queries will return the same point

}


Int  WaypointPathRep::findIndex(Real s) const
{
  // common end cases
  if ((si[0] <= s) && (s <= si[1])) return 0;
  if ((si[si.size()-2] <= s) && (s <= si[si.size()-1])) return si.size()-2;
  
  if (si.size()>4) {
    // binary search
    Int high, i, low;
    for(low=1, high=si.size()-1; high-low > 1;) {
      i = (high+low)/2;
      if (s <= si[i]) high=i; else low=i;
    }
    if ((si[i]<=s) && (s<=si[i+1])) return i; else return i-1;
  }
  else {
    // linear search
    Int i;
    for(i=0; i<si.size(); i++)
      if (si[i] > s) break;
    return i-1;
  }
}



void WaypointPathRep::translate(const Vector3& t)
{
  for(Int i=0; i<points.size(); i++)
    points[i] += t;
}


void WaypointPathRep::rotate(const Quat4& r)
{
  for(Int i=0; i<points.size(); i++) {
    r.rotatePoint(points[i]);
    Quat4 q(orients[i].getQuat4());
    orients[i] = r*q;
  }
}


void WaypointPathRep::transform(const Matrix4& m)
{
  for(Int i=0; i<points.size(); i++) {
    Point4 p(points[i]);
    p = m*p;
    points[i] = Point3(p.x,p.y,p.z);

    Quat4 r;
    r.setRotation(m);
    Quat4 q(orients[i].getQuat4());
    orients[i] = r*q;
  }  
}


void WaypointPathRep::scalePosition(Real s)
{
  for(Int i=0; i<points.size(); i++) {
    points[i] *= s;
  }
}


void WaypointPathRep::serialize(Serializer& s)
{
  s(flags,"flags");
  s(points,"points");
  s(orients,"orients");
  s(si,"si");
}
