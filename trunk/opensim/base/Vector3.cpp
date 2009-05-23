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
  
  $Id: Vector3.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.5 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/Vector3>

#include <base/consts>
#include <base/Serializer>

using base::Vector3;
using base::array;


Vector3 Vector3::min(const array<Vector3>& vectors)
{
  Vector3 minimum(consts::maxReal, consts::maxReal,consts::maxReal);

  for(array<Vector3>::size_type i=0; i<vectors.size(); i++) {
    const Vector3& vi = vectors[i];

    if (vi.x < minimum.x)
      minimum.x = vi.x;
    if (vi.y < minimum.y)
      minimum.y = vi.y;
    if (vi.z < minimum.z)
      minimum.z = vi.z;
  }

  return minimum;
}


Vector3 Vector3::max(const array<Vector3>& vectors)
{
  Vector3 maximum(consts::minReal, consts::minReal,consts::minReal);

  for(array<Vector3>::size_type i=0; i<vectors.size(); i++) {
    const Vector3& vi = vectors[i];

    if (vi.x > maximum.x)
      maximum.x = vi.x;
    if (vi.y > maximum.y)
      maximum.y = vi.y;
    if (vi.z > maximum.z)
      maximum.z = vi.z;
  }

  return maximum;
}

void Vector3::minmax(const array<Vector3>& vectors, Vector3& minimum, Vector3& maximum)
{
  minimum = Vector3::min(vectors);
  maximum = Vector3::max(vectors);
}



void Vector3::serialize(Serializer& s)
{ 
  s(x,"x"); s(y,"y"); s(z,"z"); 
}

