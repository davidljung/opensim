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
  
  $Id: GJKCollisionDetector.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.6 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/GJKCollisionDetector>
#include <gfx/Triangle3>
#include <base/ref>
#include <base/Quat4>

using physics::GJKCollisionDetector;
using physics::CollisionState;
using physics::CollisionModelProvider;
using base::Vector3;
using base::Matrix3;
using base::transpose;
using base::inverse;
using base::cross;
using gfx::Triangle3;


#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Transform>
#include <osg/StateSet>
#include <osg/Material>
#include <osg/Array>
#include <osg/PolygonMode>
#include <osg/Geometry>

using osg::Vec4;
using osg::StateSet;




// public


GJKCollisionDetector::GJKCollisionDetector()
  : collisionDetectionEnabled(true), node(0)
{
}


GJKCollisionDetector::~GJKCollisionDetector()
{
}


const Real rel_error = 1e-6; // relative error in the computed distance
const Real abs_error = 1e-10; // absolute error if the distance is almost zero
  


GJKCollisionDetector::GJKCollisionState::GJKCollisionState(ref<const Solid> solid1, ref<const Solid> solid2,
							   CollisionModel::CollisionModelType modelType)
  : CollisionState(solid1, solid2)
{
  ref<const GJKCollisionModel> cm1 = narrow_ref<const GJKCollisionModel>( solid1->getCollisionModel(modelType) );
  ref<const GJKCollisionModel> cm2 = narrow_ref<const GJKCollisionModel>( solid2->getCollisionModel(modelType) );

}

GJKCollisionDetector::GJKCollisionState::~GJKCollisionState()
{
}




ref<CollisionState> GJKCollisionDetector::newCollisionState(ref<const Solid> solid1, ref<const Solid> solid2) const
{
  GJKCollisionState *sp = NewNamedObj("GJKCollisionState") GJKCollisionState(solid1, solid2, getType());
  return ref<GJKCollisionState>(sp);
}

bool GJKCollisionDetector::collision(ref<const Solid> solid1, ref<const Solid> solid2, CollisionQueryType queryType)
{
  if (!collisionDetectionEnabled) return false;
  if (queryType == PotentialCollision) return true;

  // This must be called before GJK intersect/closest_points functions
  // to ensure that a GJKCollisionState for the pair has been created

  ref<GJKCollisionState> cs(narrow_ref<GJKCollisionState>(getCollisionState(solid1,solid2))); 

  // Do a pair-wise collision check

  ref<const GJKCollisionModel> model1 
    = narrow_ref<const GJKCollisionModel>(solid1->getCollisionModel(getType()));
  ref<const GJKCollisionModel> model2 
    = narrow_ref<const GJKCollisionModel>(solid2->getCollisionModel(getType()));
  
  base::Transform t1(solid1->getPosition(), solid1->getOrientation());
  base::Transform t2(solid2->getPosition(), solid2->getOrientation());
  
  // update OSGVisual (if it exists)
  if (node!=0) {
    const Point3 s0(t1(cs->lastSupportPoint[0]));
    const Point3 s1(t2(cs->lastSupportPoint[1]));
    (*verts)[0].set(s0.x,s0.y,s0.z); 
    (*verts)[1].set(s1.x,s1.y,s1.z); 
    geom->setVertexArray(verts);
  }


  // get last support verts
  ref<const Polyhedron::Vertex> support0(cs->lastSupport[0]);
  ref<const Polyhedron::Vertex> support1(cs->lastSupport[1]);

  // perform collision check
  ref<GJKCollisionModel::GJKModelState> ms1(NewObj GJKCollisionModel::GJKModelState(support0));
  ref<GJKCollisionModel::GJKModelState> ms2(NewObj GJKCollisionModel::GJKModelState(support1));

  bool collided = intersect(model1, model2, t1, t2, ms1, ms2, cs);

  cs->lastSupport[0] = ms1->lastSupport;
  cs->lastSupport[1] = ms2->lastSupport;
  cs->lastSupportPoint[0] = ms1->lastSupportPoint;
  cs->lastSupportPoint[1] = ms2->lastSupportPoint;


  if (collided) {

    // construct contacts 

    /*    
    const DtScalar* p1 = collision_data->point1;
    const DtScalar* p2 = collision_data->point2;
    const DtScalar* n = collision_data->normal;
    Point3 cp1(p1[0],p1[1],p1[2]);
    Point3 cp2(p2[0],p2[1],p2[2]);
    Vector3 cn(n[0],n[1],n[2]);
    CollisionState::Contact contact(cp1, cp2,cn);

    cs->contacts.clear();
    cs->contacts.push_back(contact);
    */
    
    std::cout << "Collided.\n"; //  T1=\n" << T1.getBasis() << "\n:" << T1.getOrigin() << "\n";
  }
  else {
    std::cout << "No Collision\n"; // v=" << cs->v << std::endl;
  }
    
  return collided;
}


// GJK algorithm

void GJKCollisionDetector::compute_det() 
{
  static Real dp[4][4];

  for (Int i = 0, bit = 1; i < 4; ++i, bit <<=1) 
    if (bits & bit) dp[i][last] = dp[last][i] = dot(y[i], y[last]);
  dp[last][last] = dot(y[last], y[last]);

  det[last_bit][last] = 1;
  for (Int j = 0, sj = 1; j < 4; ++j, sj <<= 1) {
    if (bits & sj) {
      Int s2 = sj|last_bit;
      det[s2][j] = dp[last][last] - dp[last][j]; 
      det[s2][last] = dp[j][j] - dp[j][last];
      for (Int k = 0, sk = 1; k < j; ++k, sk <<= 1) {
	if (bits & sk) {
	  Int s3 = sk|s2;
	  det[s3][k] = det[s2][j] * (dp[j][j] - dp[j][k]) + 
	               det[s2][last] * (dp[last][j] - dp[last][k]);
	  det[s3][j] = det[sk|last_bit][k] * (dp[k][k] - dp[k][j]) + 
	               det[sk|last_bit][last] * (dp[last][k] - dp[last][j]);
	  det[s3][last] = det[sk|sj][k] * (dp[k][k] - dp[k][last]) + 
	                  det[sk|sj][j] * (dp[j][k] - dp[j][last]);
	}
      }
    }
  }
  if (all_bits == 15) {
    det[15][0] = det[14][1] * (dp[1][1] - dp[1][0]) + 
                 det[14][2] * (dp[2][1] - dp[2][0]) + 
                 det[14][3] * (dp[3][1] - dp[3][0]);
    det[15][1] = det[13][0] * (dp[0][0] - dp[0][1]) + 
                 det[13][2] * (dp[2][0] - dp[2][1]) + 
                 det[13][3] * (dp[3][0] - dp[3][1]);
    det[15][2] = det[11][0] * (dp[0][0] - dp[0][2]) + 
                 det[11][1] * (dp[1][0] - dp[1][2]) +  
                 det[11][3] * (dp[3][0] - dp[3][2]);
    det[15][3] = det[7][0] * (dp[0][0] - dp[0][3]) + 
                 det[7][1] * (dp[1][0] - dp[1][3]) + 
                 det[7][2] * (dp[2][0] - dp[2][3]);
  }
}


inline bool GJKCollisionDetector::valid(Int s) 
{  
  for (Int i = 0, bit = 1; i < 4; ++i, bit <<= 1) {
    if (all_bits & bit) {
      if (s & bit) { if (det[s][i] <= 0) return false; }
      else if (det[s|bit][i] > 0) return false;
    }
  }
  return true;
}

inline void GJKCollisionDetector::compute_vector(Int bits, Vector3& v)
{
  Real sum = 0;
  v.setZero();
  for (Int i = 0, bit = 1; i < 4; ++i, bit <<= 1) {
    if (bits & bit) {
      sum += det[bits][i];
      v += y[i] * det[bits][i];
    }
  }
  v *= 1 / sum;
}

inline void GJKCollisionDetector::compute_points(int bits, Point3& p1, Point3& p2)
{
  Real sum = 0;
  p1.setZero();
  p2.setZero();
  for (Int i = 0, bit = 1; i < 4; ++i, bit <<= 1) {
    if (bits & bit) {
      sum += det[bits][i];
      p1 += p[i] * det[bits][i];
      p2 += q[i] * det[bits][i];
    }
  }
  Real s = 1 / sum;
  p1 *= s;
  p2 *= s;
}

inline bool GJKCollisionDetector::closest(Vector3& v) 
{
  compute_det();
  for (Int s = bits; s; --s) {
    if ((s & bits) == s) {
      if (valid(s|last_bit)) {
	bits = s|last_bit;
 	compute_vector(bits, v);
	return true;
      }
    }
  }
  if (valid(last_bit)) {
    bits = last_bit;
    v = y[last];
    return true;
  }

  return false;
}


bool GJKCollisionDetector::intersect(ref<const GJKCollisionModel> model1, ref<const GJKCollisionModel> model2,
				     const base::Transform& t1, const base::Transform& t2, 
				     ref<GJKCollisionModel::GJKModelState> modelState1,
				     ref<GJKCollisionModel::GJKModelState> modelState2,
				     ref<GJKCollisionState> collisionState)
{
  Vector3 w;
  Vector3& v(collisionState->v);

  bits = 0;
  all_bits = 0;


  do {
    last = 0;
    last_bit = 1;
    while (bits & last_bit) { ++last; last_bit <<= 1; }
    w =   t1(model1->support(modelState1,t1.rotate(-v))) 
        - t2(model2->support(modelState2,t2.rotate( v))); 
    if (dot(v, w) > 0) return false;

    y[last] = w;
    all_bits = bits|last_bit;
    if (!closest(v)) {
      return false;
    }
  } 
  while (bits < 15 && !v.equalsZero() ); 
  return true;
}



/// create a visual for debugging purposes
osg::Node* GJKCollisionDetector::createOSGVisual(Visual::Attributes visualAttributes) const
{
  if (!(visualAttributes & ShowCollisionDetection)) return NewObj osg::Node();

  if ((node!=0) && (attributes==visualAttributes))
    return &(*node);

  // show a line between the last support point and the passed v.
  geom = new osg::Geometry();

  StateSet* state = new osg::StateSet();
  osg::Material* mat = new osg::Material();
  Vec4 col( 1, 1, 0, 1);
  mat->setAmbient( osg::Material::FRONT_AND_BACK, col );
  mat->setDiffuse( osg::Material::FRONT_AND_BACK, col );
  state->setAttribute( mat );
  //geom->setStateSet(state);

  verts = new osg::Vec3Array(2);
  (*verts)[0].set(0,0,0);
  (*verts)[1].set(0,0,0);
  geom->setVertexArray(verts);

  osg::Vec4Array* colors = new osg::Vec4Array(1);
  (*colors)[0].set(1.0f,0.0f,0.0f,1.0f);
  geom->setColorArray(colors);
  geom->setColorBinding(osg::Geometry::BIND_OVERALL);

  osg::DrawArrays* linearray = new osg::DrawArrays(osg::PrimitiveSet::LINES,0,2);
  geom->addPrimitiveSet(linearray);

  osg::Geode* geode = new osg::Geode();
  geode = new osg::Geode();
  geode->setName("Debug");
  geode->addDrawable(&(*geom));

  node = geode;
  attributes = visualAttributes;

  return &(*node);
}

