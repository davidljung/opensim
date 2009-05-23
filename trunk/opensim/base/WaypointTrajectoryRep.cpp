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
  
  $Id: WaypointTrajectoryRep.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.2 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/WaypointTrajectoryRep>

#include <base/Serializer>

using base::WaypointTrajectoryRep;

using base::Time;


Time WaypointTrajectoryRep::time(Real s) const
{
  Math::bound<Real>(s,0,1);
  return Time( times[0].seconds() +  ( (times[times.size()-1] - times[0]).seconds() * s ) );
}


void WaypointTrajectoryRep::shiftTime(const Time& dt)
{
  for(Int i=0; i<times.size(); i++)
    times[i] += dt;
}


void WaypointTrajectoryRep::scaleTime(Real s)
{
  for(Int i=0; i<times.size(); i++)
    times[i] *= s;
}


Real WaypointTrajectoryRep::gets(const Time& t) const
{
  const Time& st(times[0]);
  const Time& et(times[times.size()-1]);
  Time bt(t);
  Math::bound<Time>(bt,st,et);
  return (bt-st).seconds() / (et-st).seconds();
}

void WaypointTrajectoryRep::serialize(Serializer& s)
{
  WaypointPathRep::serialize(s);
  s(times,"times");
}


void WaypointTrajectoryRep::computeSis()
{
  // ensure at least two points
  if (points.size() == 0) { points.at(0) = points.at(1) = Point3(); }
  if (orients.size() == 0) { orients.at(0) = orients.at(1) = Orient(); }
  if (times.size() == 0) { times.at(0) = 0; times.at(1) = 1; }

  Assert(points.size() == orients.size());
  Assert(points.size() == times.size());

  // s's map [0..1] to the range [start-time..end-time]
  Real T = (times[times.size()-1] - times[0]).seconds();
  Int i = 1;
  si.resize(points.size());
  si[0] = 0;
  while (i<points.size()) {
    if (!(times[i-1] < times[i])) 
      throw std::invalid_argument(Exception("times must be monotonically increasing"));
    si[i] = (times[i] - times[0]).seconds() / T;
    i++;
  }

}
