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
  
  $Id: CollisionDetector.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.8 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/CollisionDetector>

#include <base/Dimension3>
#include <base/Transform>
#include <gfx/Segment3>
#include <physics/Box>
#include <physics/Sphere>

using physics::CollisionDetector;

using base::Dimension3;
using base::Transform;

using gfx::Segment3;

using physics::Collidable;
using physics::CollidableBody;
using physics::CollidableGroup;
using physics::CollisionState;

using physics::Shape;
using physics::Box;
using physics::Sphere;



namespace physics {

// needed by std::map CollisionStateMap
bool operator<(ref<const CollidableBody> s1, const CollidableBody* s2)
{ return &(*s1) < s2; }

}


const Real CollisionDetector::collisionEpsilon = 1e-6;


CollisionDetector::CollisionDetector()
 : queryType(AllContactPoints)
{
}


CollisionDetector::~CollisionDetector()
{
}


Segment3 CollisionDetector::shortestSegmentBetween(ref<const Collidable> collidable1, ref<const Collidable> collidable2)
{
  //!!! this isn't properly implemented at present.  Can use GJK algorithm sometime.
  // Box stuff only uses box verts!
  
  if (instanceof(*collidable1, const CollidableGroup) || instanceof(*collidable2, const CollidableGroup))
    throw std::runtime_error(Exception("distance between CollidableGoups not yet implemented - sorry."));
  
  if (instanceof(*collidable1, const CollidableBody) && instanceof(*collidable1, const CollidableBody)) {
    
    ref<const CollidableBody> body1(narrow_ref<const CollidableBody>(collidable1));
    ref<const CollidableBody> body2(narrow_ref<const CollidableBody>(collidable2));
    
    ref<const Shape> shape1(body1->getShape());
    ref<const Shape> shape2(body2->getShape());
    
    //!!! a map of methods would have been better - probably won't need either when GJK is implemented
    
    if (instanceof(*shape1, const Box)) {
      
      ref<const Box> box1(narrow_ref<const Box>(shape1));
      
      if (instanceof(*shape2, const Box)) {

        ref<const Box> box2(narrow_ref<const Box>(shape2));
        
        return box1->shortestSegmentBetween(      body1->getConfiguration(),
                                            box2, body2->getConfiguration());
      }
      else if (instanceof(*shape2, const Sphere)) {

        ref<const Sphere> sphere2(narrow_ref<const Sphere>(shape2));
        
        // segment between box and sphere center                                           
        Segment3 seg( box1->shortestSegmentBetween( body1->getConfiguration(), 
                                                    body2->getPosition() ) );
        Real r(sphere2->radius());
        // shorten it by r on the e end
        Vector3 dir(seg.e-seg.s);
        seg.e = seg.s + dir*(1-(r/dir.length()));
        
        return seg;         
        
      }
      else
        throw std::runtime_error(Exception("unsupported Shape combination"));
      
    } 
    else if (instanceof(*shape1, const Sphere)) {
      
      ref<const Sphere> sphere1(narrow_ref<const Sphere>(shape1));
      
      if (instanceof(*shape2, const Box)) {
        
        Segment3 minSeg = shortestSegmentBetween(collidable2, collidable1);
        base::swap(minSeg.s, minSeg.e);
        return minSeg;
        
      }
      else if (instanceof(*shape2, const Sphere)) {
        ref<const Sphere> sphere2(narrow_ref<const Sphere>(shape2));
        
        Vector3 vdir( body2->getPosition() - body1->getPosition() ); vdir.normalize();
        Point3 spherep1( body1->getPosition() + vdir*sphere1->radius() );
        Point3 spherep2( body2->getPosition() - vdir*sphere2->radius() );
        return Segment3( spherep1, spherep2 );
      }
      else
        throw std::runtime_error(Exception("unsupported Shape combination"));
      
    }
    else
      throw std::runtime_error(Exception("unsupported Shape"));
    
    
  }
  else {
    throw std::runtime_error(Exception("unsupported Collidable subtype"));
  }
  
}

ref<CollisionState> CollisionDetector::getCollisionState(ref<const CollidableBody> collidableBody1, ref<const CollidableBody> collidableBody2) const
{  
  if (&(*collidableBody1) > &(*collidableBody2))
    base::swap(collidableBody1,collidableBody2);

  CollidableBodyPair contactPair = std::make_pair(collidableBody1, collidableBody2);

  // is there a state entry in the map for this pair?
  CollisionStateMap::iterator entry = collisionStates.find(contactPair);
  if (entry != collisionStates.end()) {
    // yes, use it
    return entry->second;
  }
  else {
    // no, create a new one
    ref<CollisionState> collisionState = newCollisionState(collidableBody1,collidableBody2);
    collisionStates[contactPair] = collisionState; 
    return collisionState;
  }
}


