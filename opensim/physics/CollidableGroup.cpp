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
  
  $Id: CollidableGroup.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.3 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/CollidableGroup>

using physics::CollidableGroup;

using physics::Collidable;


ref<const Collidable> CollidableGroup::findNamed(const String& name, bool recurse) const
{
  const_iterator_const c = const_begin();
  const_iterator_const cend = const_end();
  while (c != cend) {
    if ((*c)->getName() == name)
      return *c;
    
    if (instanceof(**c, const CollidableGroup)) {
      ref<const CollidableGroup> group(narrow_ref<const CollidableGroup>(*c));
      ref<const Collidable> found = recurse?group->findNamed(name, recurse):ref<const Collidable>(0);
      if (found) return found;
    }

    ++c;
  }
  return ref<Collidable>(0);  
}


ref<Collidable> CollidableGroup::findNamed(const String& name, bool recurse)
{
  const_iterator c = begin();
  const_iterator cend = end();
  while (c != cend) {
    if ((*c)->getName() == name)
      return *c;
    
    if (instanceof(**c, CollidableGroup)) {
      ref<CollidableGroup> group(narrow_ref<CollidableGroup>(*c));
      ref<Collidable> found = recurse?group->findNamed(name, recurse):ref<Collidable>(0);
      if (found) return found;
    }

    ++c;
  }
  return ref<Collidable>(0);  
}


bool CollidableGroup::find(ref<const Collidable> collidable, bool recurse) const
{
  const_iterator_const c = const_begin();
  const_iterator_const cend = const_end();
  while (c != cend) {
    if ((*c) == collidable)
      return true;
    
    if (instanceof(**c, const CollidableGroup)) {
      ref<const CollidableGroup> group(base::narrow_ref<const CollidableGroup>(*c));
      bool found = recurse?group->find(collidable,recurse):false;
      if (found) return true;
    }

    ++c;
  }
  return false;
}
