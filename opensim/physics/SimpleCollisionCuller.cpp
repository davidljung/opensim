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
  
  $Id: SimpleCollisionCuller.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.3 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/SimpleCollisionCuller>

using physics::SimpleCollisionCuller;

using physics::Collider;
using physics::Collidable;
using physics::CollidableBody;
using physics::CollidableGroup;


void SimpleCollisionCuller::collide(ref<const Collidable> collidable1, ref<const Collidable> collidable2)
{
  if (!isCollisionEnabled()) return;
  
  Assert(collidable1);
  Assert(collidable2);
  
  if (instanceof(*collidable1, const CollidableBody) && instanceof(*collidable2, const CollidableBody)) {
    if (!isCollisionEnabled(collidable1, collidable2)) return; // cull
    notifyListeners(collidable1, collidable2); // both bodies
    return;
  }
    
  if (instanceof(*collidable1, const CollidableGroup) && instanceof(*collidable2, const CollidableGroup)) {
    // both groups
    
    ref<const CollidableGroup> group1(narrow_ref<const CollidableGroup>(collidable1));
    ref<const CollidableGroup> group2(narrow_ref<const CollidableGroup>(collidable2));
 
    // collide each element of the second group with the whole first group
    if (isCollisionEnabled(collidable1, collidable2)) {
      CollidableGroup::const_iterator_const c = group2->const_begin();
      CollidableGroup::const_iterator_const end = group2->const_end();
      while (c != end) {
        collide(group1, *c); 
        ++c;
      }
    }
   
    // collide each the elements of the groups among themselves
    if (group1->isChildIntercollisionEnabled())
      collide(group1);
    
    if (group2->isChildIntercollisionEnabled())
      collide(group2);
    
  }
  else {
    // one of each, make the group the first one
    if (instanceof(*collidable1, const CollidableBody))
      base::swap(collidable1, collidable2);
    
    // now collidable1 is group & collidable2 is a body
    // Collide each group element with the body
    Assert(instanceof(*collidable1, const CollidableGroup));
    Assert(instanceof(*collidable2, const CollidableBody));
    
    ref<const CollidableGroup> group(narrow_ref<const CollidableGroup>(collidable1));
    ref<const CollidableBody>  body(narrow_ref<const CollidableBody>(collidable2));

    // collide each element of the group with the body
    if (isCollisionEnabled(collidable1, collidable2)) {
      CollidableGroup::const_iterator_const c = group->const_begin();
      CollidableGroup::const_iterator_const end = group->const_end();
      while (c != end) {
        collide(*c, body); 
        ++c;
      }
    }

    if (group->isChildIntercollisionEnabled())
      collide(group); // and collide the group elements among themselves
    
  }

}
