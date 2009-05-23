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
  
  $Id: BoundingBox.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.1 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/BoundingBox>

using physics::BoundingBox;

using base::Point3;
using base::Transform;


void BoundingBox::enclose(const BoundingBox& a, const BoundingBox& b) 
{
    Point3 alower(a.lower());
    Point3 blower(b.lower());
    Point3 aupper(a.upper());
    Point3 bupper(b.upper());
    
    Point3 _lower(Math::minimum(alower.x, blower.x),
	          Math::minimum(alower.y, blower.y),
	          Math::minimum(alower.z, blower.z) );
    Point3 _upper(Math::maximum(aupper.x, bupper.x),
	          Math::maximum(aupper.y, bupper.y),
	          Math::maximum(aupper.z, bupper.z) );
    setExtents(_lower, _upper);
}


void BoundingBox::include(const base::Point3& p) 
{
  Point3 _lower(Math::minimum(lower().x, p.x),
                Math::minimum(lower().y, p.y),
                Math::minimum(lower().z, p.z) );
  Point3 _upper(Math::maximum(upper().x, p.x),
                Math::maximum(upper().y, p.y),
                Math::maximum(upper().z, p.z) );
  setExtents(_lower, _upper);
}


void BoundingBox::transform(const Transform& t)
{
  // construct 8 vectices
  Point3 l(lower());
  Point3 u(upper());
  
  array<Point3> v(8);
  v[0]=l;
  v[1]=Point3(l.x,u.y,l.z);
  v[2]=Point3(u.x,u.y,l.z);
  v[3]=Point3(u.x,l.y,l.z);
  
  v[4]=Point3(l.x,l.y,u.z);
  v[5]=Point3(l.x,u.y,u.z);
  v[6]=u;
  v[7]=Point3(u.x,l.y,u.z);
  
  // transform them
  for(Int vi=0; vi<8; vi++)
    t.transformPoint(v[vi]);
  
  // find new bounding box
  setEmpty();
  for(Int vi=0; vi<8; vi++)
    include(v[vi]);
}
