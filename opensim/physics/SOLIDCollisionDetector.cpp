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
  
  $Id: SOLIDCollisionDetector.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.2 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/SOLIDCollisionDetector>
#include <gfx/Triangle3>
#include <base/ref>
#include <base/Quat4>

using physics::SOLIDCollisionDetector;
using physics::CollisionState;
using physics::CollisionModelProvider;
using base::Vector3;
using base::Matrix3;
using base::transpose;
using base::inverse;
using base::cross;
using gfx::Triangle3;





void SOLIDCollisionDetectorResponse(void *client_data, 
				    DtObjectRef object1, 
				    DtObjectRef object2,
				    const DtCollData *coll_data);


// public


SOLIDCollisionDetector::SOLIDCollisionDetector()
  : collisionDetectionEnabled(true) 
{
  if (!SOLIDResponseFunctionRegistered) {

    dtSetDefaultResponse(SOLIDCollisionDetectorResponse, DT_SIMPLE_RESPONSE, this);

    SOLIDResponseFunctionRegistered = true;
  }
  else {
    Logln("Warning: Only one instance of SOLIDCollisionDetector will function correctly due to SOLID's C API. Sorry.");
  }
    
}


SOLIDCollisionDetector::~SOLIDCollisionDetector()
{
  SOLIDResponseFunctionRegistered=false;
}


template<typename T> 
inline void* cast_void_ptr(T* p) 
{ return const_cast<void*>(static_cast<const void*>(p)); } // blech


SOLIDCollisionDetector::SOLIDCollisionState::SOLIDCollisionState(ref<const Solid> solid1, ref<const Solid> solid2,
								 CollisionModel::CollisionModelType modelType)
  : CollisionState(solid1, solid2)
{
  ref<const SOLIDCollisionModel> cm1 = narrow_ref<const SOLIDCollisionModel>( solid1->getShape()->getCollisionModel(modelType) );
  ref<const SOLIDCollisionModel> cm2 = narrow_ref<const SOLIDCollisionModel>( solid2->getShape()->getCollisionModel(modelType) );

  dtCreateObject( cast_void_ptr(&(*solid1)), cm1->getSOLIDShapeRef() );
  dtCreateObject( cast_void_ptr(&(*solid2)), cm2->getSOLIDShapeRef() );
}

SOLIDCollisionDetector::SOLIDCollisionState::~SOLIDCollisionState()
{
  dtDeleteObject(cast_void_ptr(&(*solid1)));
  dtDeleteObject(cast_void_ptr(&(*solid2)));
}




ref<CollisionState> SOLIDCollisionDetector::newCollisionState(ref<const Solid> solid1, ref<const Solid> solid2) const
{
  SOLIDCollisionState *sp = NewNamedObj("SOLIDCollisionState") SOLIDCollisionState(solid1, solid2, getType());
  return ref<SOLIDCollisionState>(sp);
}

bool SOLIDCollisionDetector::collision(ref<const Solid> solid1, ref<const Solid> solid2, CollisionQueryType queryType)
{
  if (!collisionDetectionEnabled) return false;

  // This must be called before dtTestObjects to ensure that a SOLIDCollisionState 
  //  for the pair has been created
  //  ref<SOLIDCollisionState> cs = narrow_ref<SOLIDCollisionState>( ref<>(&getCollisionState(solid1,solid2)) );
  //  ref<CollisionState> csr(getCollisionState(solid1,solid2));
  //  ref<SOLIDCollisionState> scsr = narrow_ref<SOLIDCollisionState>(csr);

  ref<SOLIDCollisionState> cs(narrow_ref<SOLIDCollisionState>(getCollisionState(solid1,solid2))); 

  /*
  ref<const SOLIDCollisionModel> model1 
    = narrow_ref(const SOLIDCollisionModel,solid1.getCollisionModel(getType()));
  ref<const SOLIDCollisionModel> model2 
    = narrow_ref(const SOLIDCollisionModel,solid2.getCollisionModel(getType()));
  */

  // Do a pair-wise collision check (this bypasses SOLID's coarse collision checking
  //  - we don't use it)
  //  First, position the SOLID objects according to the Solid's

  dtSelectObject( cast_void_ptr(&(*solid1)) );
  dtLoadIdentity();
  Quat4 q1(solid1->getOrientation());
  dtRotate(q1.v.x, q1.v.y, q1.v.z, q1.w);
  Point3 p1(solid1->getPosition());
  dtTranslate(p1.x, p1.y, p1.z);

  dtSelectObject( cast_void_ptr(&(*solid2)) );
  dtLoadIdentity();
  Quat4 q2(solid2->getOrientation());
  dtRotate(q2.v.x, q2.v.y, q2.v.z, q2.w);
  Point3 p2(solid2->getPosition());
  dtTranslate(p2.x, p2.y, p2.z);

  collided=false; // will be set to true if the collisionCallback() is called
                  // (which will also will in the contact state information)
  dtTestObjects( cast_void_ptr(&(*solid1)), cast_void_ptr(&(*solid2)) );
  
  if (collided) {
    
    // consistiency check
    if ((this->solid1 != solid1) || (this->solid2 != solid2)) {
      Debugln("Got unexpected collision callback from another pair of solids.  Ignoring.");
      return false;
    }

    const DtScalar* p1 = collision_data->point1;
    const DtScalar* p2 = collision_data->point2;
    const DtScalar* n = collision_data->normal;
    Point3 cp1(p1[0],p1[1],p1[2]);
    Point3 cp2(p2[0],p2[1],p2[2]);
    Vector3 cn(n[0],n[1],n[2]);
    CollisionState::Contact contact(cp1, cp2,cn);

    cs->contacts.clear();
    cs->contacts.push_back(contact);
    Debugln("Collided!!");
  }
    
  return collided;
}



void SOLIDCollisionDetector::collisionCallback(DtObjectRef object1, DtObjectRef object2,
					       const DtCollData* coll_data)
{
  if (collided) {
    Debugln("collisionCallback() called more than once.  Ignoring collision.");
    return;
  }

  collided = true;
  // careful re-constructing refs from ptrs here (IntrRefCounted works, but other impls may not)
  solid1 = ref<const Solid>(static_cast<const Solid*>(object1));
  solid2 = ref<const Solid>(static_cast<const Solid*>(object2));
  collision_data = coll_data;
}




void SOLIDCollisionDetectorResponse(
   void *client_data, 
   DtObjectRef object1, 
   DtObjectRef object2,
   const DtCollData *coll_data)
{
  SOLIDCollisionDetector* cd = static_cast<SOLIDCollisionDetector*>(client_data);
  cd->collisionCallback(object1, object2, coll_data);
}




bool SOLIDCollisionDetector::SOLIDResponseFunctionRegistered = false;

