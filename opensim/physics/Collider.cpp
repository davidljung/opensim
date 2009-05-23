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

  $Id: Collider.cpp 1157 2004-09-29 17:43:50Z jungd $

****************************************************************************/

#include <physics/Collider>

using physics::Collider;

using base::dynamic_cast_ref;
using physics::Collidable;
using physics::CollidableGroup;



Collider::Collider()
 : enabled(true)
{
}


Collider::~Collider()
{
}

//!!1
static int indent = 0;

void Collider::collide(ref<const Collidable> collidable)
{
  if (!isCollisionEnabled()) return;

  if (!instanceof(*collidable, const CollidableGroup))
    return; // can't collide one Collidable with anything - do nothing

  ref<const CollidableGroup> group(narrow_ref<const CollidableGroup>(collidable));

  if (!group->isChildIntercollisionEnabled()) return;
//!!!
  #ifdef TMPDEBUG
  Debugln(Collision,"Colliding group members:");
  CollidableGroup::const_iterator_const dc1 = group->const_begin();
  CollidableGroup::const_iterator_const dend = group->const_end();
  while (dc1 != dend) {
    Debugcln(Collision,"  "+(*dc1)->getName());
    ++dc1;
  }
  #endif
//!!!
  CollidableGroup::const_iterator_const c1 = group->const_begin();
  CollidableGroup::const_iterator_const end = group->const_end();
  while (c1 != end) {
    ref<const Collidable> collidable1(*c1);

    CollidableGroup::const_iterator_const c2 = group->const_begin();
    while (c2 != end) {
      ref<const Collidable> collidable2(*c2);

      if (&(*collidable1) < &(*collidable2)) {  // i.e. don't collide if c1==c2 or c2,c1 if already did c1,c2
//!for(int i=0;i<indent;i++) Debugc(Tmp," ");
//!Debugcln(Collision,"Collider: " << collidable1->getName() << " & " << collidable2->getName());
//!indent++;
        collide(collidable1, collidable2);
//!indent--;
      }

      ++c2;
    }

    ++c1;
  }
}


void Collider::notifyListeners(ref<const Collidable> collidable1, ref<const Collidable> collidable2)
{
  if (!isCollisionEnabled()) return;

  ListenerList::const_iterator l = listeners.begin();
  ListenerList::const_iterator end = listeners.end();
  while (l != end) {
    (*l)->potentialCollision(collidable1, collidable2);
    ++l;
  }
}


void Collider::resetListeners()
{
  if (!isCollisionEnabled()) return;

  ListenerList::const_iterator l = listeners.begin();
  ListenerList::const_iterator end = listeners.end();
  while (l != end) {
    (*l)->reset();
    ++l;
  }
}

