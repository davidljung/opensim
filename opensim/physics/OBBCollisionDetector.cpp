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
  
  $Id: OBBCollisionDetector.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.3 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/OBBCollisionDetector>
#include <base/Quat4>

using physics::OBBCollisionDetector;
using physics::CollisionState;
using physics::CollisionModelProvider;
using base::Vector3;
using base::Matrix3;
using base::transpose;
using base::inverse;
using base::cross;
using gfx::Triangle3;


// public

bool OBBCollisionDetector::collision(ref<const Solid> solid1, ref<const Solid> solid2, CollisionQueryType queryType)
{
  if (!collisionDetectionEnabled) return false;
  if (queryType == PotentialCollision) return true;

  ref<const OBBCollisionModel> model1 
    = narrow_ref<const OBBCollisionModel>(solid1->getCollisionModel(getType()));
  ref<const OBBCollisionModel> model2 
    = narrow_ref<const OBBCollisionModel>(solid2->getCollisionModel(getType()));

  Matrix3 R1 = Matrix4(solid1->getOrientation()); // a better way to convert? !!
  Matrix3 R2 = Matrix4(solid2->getOrientation());
  const Vector3& T1(solid1->getPosition());
  const Vector3& T2(solid2->getPosition());

  // Collision test
  Collide(R1, T1, model1, R2, T2, model2, queryType);
  //std::cout << "State - box:" << numBoxTests
  //	    << " tri: " << numTriTests
  //	    << " contacts:" << contacts.size() 
  //	    << " contact:" << contacts[0].first << "\n";

  double R1a[3][3];
  R1a[0][0] = R1(1,1); R1a[0][1] = R1(1,2); R1a[0][2] = R1(1,3);
  R1a[1][0] = R1(2,1); R1a[1][1] = R1(2,2); R1a[1][2] = R1(2,3);
  R1a[2][0] = R1(3,1); R1a[2][1] = R1(3,2); R1a[2][2] = R1(3,3);
  double R2a[3][3];
  R2a[0][0] = R2(1,1); R2a[0][1] = R2(1,2); R2a[0][2] = R2(1,3);
  R2a[1][0] = R2(2,1); R2a[1][1] = R2(2,2); R2a[1][2] = R2(2,3);
  R2a[2][0] = R2(3,1); R2a[2][1] = R2(3,2); R2a[2][2] = R2(3,3);

  double T1a[3]; T1a[0] = T1[1]; T1a[1] = T1[2]; T1a[2] = T1[3];
  double T2a[3]; T2a[0] = T2[1]; T2a[1] = T2[2]; T2a[2] = T2[3];
  
  // find the CollisionState
  ref<CollisionState> collisionState = getCollisionState(solid1,solid2);
  ref<OBBCollisionState> state = narrow_ref<OBBCollisionState>(collisionState);
  state->contacts.clear();
  
  // Compute collision points (if required)
  if ( ((queryType == FirstContactPoint) 
	|| (queryType == AllContactPoints))
       && contacts.size() > 0) {
    
    Int i=0;
    do {
      Triangle3 t1 = contacts[i].first;
      Triangle3 t2 = contacts[i].second;
      
      // !!! this is a hack for the contact normal - compute it properly!!
      Vector3 normal1(t1.normal());

      // Transform to world coords
      t1.transform(R1); t1 += T1;
      t2.transform(R2); t2 += T2;
      
      Point3 contactPoint; //!!! fix = t1.intersect(t2);
      
      if (contactPoint == Point3()) {
	//Debugln("Failed to compute contact point!");
	contactPoint = (t2[1]+t2[2]+t2[3])/3.0;
      }
      
      Point3 contact1(contactPoint-T1);//!!! untested
      contact1 = inverse(R1)*contact1;
      Point3 contact2(contactPoint-T2);
      contact2 = inverse(R2)*contact2;

      //      Point3 contact1(gfx::inverse(transform1.getTransform())*contactPoint);
      //      Point3 contact2(gfx::inverse(transform2.getTransform())*contactPoint);
      
      //			Debugln("Contacts:" << i << ": " << contact1 << " and " << contact2 
      //							<< " global:" << contactPoint);

      state->contacts.push_back(CollisionState::Contact(contact1,contact2,normal1));
      
      i++;
    } while ((i < contacts.size()) && (queryType == AllContactPoints));
    
  }
    
  return (contacts.size() > 0);
}







// protected

void OBBCollisionDetector::Collide(const base::Matrix3& R1, const gfx::Vector3& T1, 
				   ref<const OBBCollisionModel> model1,
				   const base::Matrix3& R2, const gfx::Vector3& T2, 
				   ref<const OBBCollisionModel> model2,
				   CollisionQueryType queryType)
{
  Collide(R1,T1,1.0,model1,R2,T2,1.0,model2,queryType);
}


void OBBCollisionDetector::Collide(const base::Matrix3& R1, const gfx::Vector3& T1, Real s1, 
				   ref<const OBBCollisionModel> model1,
				   const base::Matrix3& R2, const gfx::Vector3& T2, Real s2, 
				   ref<const OBBCollisionModel> model2,
				   CollisionQueryType queryType)
{
  // reset the report fields
  numBoxTests = 0;
  numTriTests = 0;
  contacts.clear();

  if (model1->tris.empty() || model2->tris.empty()) return; // no triangles in one of the models

  const OBBCollisionModel::OBB& b1(model1->b[0]);
  const OBBCollisionModel::OBB& b2(model2->b[0]);
  
  firstContact = (queryType==DetectCollision) || (queryType==FirstContactPoint);
  
  Matrix3 tR1, tR2, R;
  Vector3 tT1, tT2, T, U;
  Real s;
	
  // [R1,T1,s1] and [R2,T2,s2] are how the two triangle sets
  // (i.e. models) are positioned in world space.  [tR1,tT1,s1] and
  // [tR2,tT2,s2] are how the top level boxes are positioned in world
  // space
  
  tR1 = R1*b1.pR;
  tT1 = (s1*R1)*b1.pT+T1;
  tR2 = R2*b2.pR;
  tT2 = (s2*R2)*b2.pT+T2;
  
  // (R,T,s) is the placement of b2's top level box within
  // the coordinate system of b1's top level box.
  
  R = transpose(tR1)*tR2;	
  U = tT2-tT1;
  T = (transpose(tR1)*U)/s1;
  
  s = s2/s1;
  
  // To transform tri's from model1's CS to model2's CS use this:
  //    x2 = ms . mR . x1 + mT
  mR = transpose(R2)*R1;
  U = T1-T2;
  mT = (transpose(R2)*U)/s2;
  ms = s1/s2;
  
  // make the call
  collideRecursive(b1, b2, R, T, s);

}


void OBBCollisionDetector::collideRecursive(const OBBCollisionModel::OBB& b1, 
					    const OBBCollisionModel::OBB& b2,
					    const base::Matrix3& R, const base::Vector3& T, double s)
{
  if ((firstContact) && (contacts.size() > 0)) return;
  
  // test top level
  numBoxTests++;
  
  Vector3 d(s*b2.d);
  Int f1 = OBBDisjoint(R, T, b1.d, d);
  
  if (f1 != 0) 
    return;  // stop processing this test, go to top of loop
  
  // contact between boxes
  if (b1.leaf() && b2.leaf()) {
    // it is a leaf pair - compare the polygons therein
    // tri_contact uses the model-to-model transforms stored in
    // mR, mT, ms.
    
    triContact(b1, b2);
    return;
  }
  
  Vector3 U;
  Matrix3 cR;
  Vector3 cT;
  Real cs;
  
  // Currently, the transform from model 2 to model 1 space is
  // given by [B T s], where y = [B T s].x = s.B.x + T.
  
  if (b2.leaf() || (!b1.leaf() && (b1.size() > b2.size()))) {
    // here we descend to children of b1.  The transform from
    // a child of b1 to b1 is stored in [b1.N.pR,b1.N.pT],
    // but we will denote it [B1 T1 1] for short.  Notice that
    // children boxes always have same scaling as parent, so s=1
    // for such nested transforms.
    
    // Here, we compute [B1 T1 1]'[B T s] = [B1'B B1'(T-T1) s]
    // for each child, and store the transform into the collision
    // test queue.
    
    cR = transpose(b1.N->pR)*R;
    U=T-b1.N->pT;
    cT = transpose(b1.N->pR)*U;
    cs = s;
    
    collideRecursive(*b1.N, b2, cR, cT, cs);
    
    cR = transpose(b1.P->pR)*R;
    U=T-b1.P->pT;
    cT=transpose(b1.P->pR)*U;
    cs = s;
    
    collideRecursive(*b1.P, b2, cR, cT, cs);
    
    return;
  }
  else {
    // here we descend to the children of b2.  See comments for
    // other 'if' clause for explanation.
    
    cR = R*b2.N->pR;
    cT = (s*R)*b2.N->pT+T;
    cs = s;
    
    collideRecursive(b1, *b2.N, cR, cT, cs);
    
    cR = R*b2.P->pR;
    cT = (s*R)*b2.P->pT+T;
    cs = s;
    
    collideRecursive(b1, *b2.P, cR, cT, cs);
    
    return; 
  }
}


void OBBCollisionDetector::triContact(const OBBCollisionModel::OBB& b1,
				      const OBBCollisionModel::OBB& b2)
{
  // assume just one triangle in each box.
  
  // the vertices of the tri in b2 is in model1 C.S.  The vertices of
  // the other triangle is in model2 CS.  Use mR, mT, and
  // ms to transform into model2 CS.
  p1 = b1.tr.p1();
  p2 = b1.tr.p2();
  p3 = b1.tr.p3();
  
  Matrix3 msR(ms*mR);
  i1 = msR*p1+mT;
  i2 = msR*p2+mT;
  i3 = msR*p3+mT;
  
  numTriTests++;
  
  if (triContact(i1, i2, i3, b2.tr.p1(), b2.tr.p2(), b2.tr.p3()))
    contacts.push_back(std::pair<Triangle3,Triangle3>(b1.tr, b2.tr));
}


bool OBBCollisionDetector::triContact(const base::Vector3& P1, const base::Vector3& P2, const base::Vector3& P3,
				      const base::Point3& Q1, const base::Point3& Q2, const base::Point3& Q3)
{
  /*
    One triangle is (p1,p2,p3).  Other is (q1,q2,q3).
    Edges are (e1,e2,e3) and (f1,f2,f3).
    Normals are n1 and m1
    Outwards are (g1,g2,g3) and (h1,h2,h3).
    
    We assume that the triangle vertices are in the same coordinate system.
    
    First thing we do is establish a new c.s. so that p1 is at (0,0,0).
    
  */
  z.setZero();
  
  p1 = P1-P1;
  p2 = P2-P1;
  p3 = P3-P1;
  
  q1 = Q1-P1;
  q2 = Q2-P1;
  q3 = Q3-P1;
  
  e1 = p2-p1;
  e2 = p3-p2;
  e3 = p1-p3;
  
  f1 = q2-q1;
  f2 = q3-q2;
  f3 = q1-q3;
  
  n1.cross(e1, e2);
  m1.cross(f1, f2);
  
  g1.cross(e1, n1);
  g2.cross(e2, n1);
  g3.cross(e3, n1);
  h1.cross(f1, m1);
  h2.cross(f2, m1);
  h3.cross(f3, m1);
  
  ef11.cross(e1, f1);
  ef12.cross(e1, f2);
  ef13.cross(e1, f3);
  ef21.cross(e2, f1);
  ef22.cross(e2, f2);
  ef23.cross(e2, f3);
  ef31.cross(e3, f1);
  ef32.cross(e3, f2);
  ef33.cross(e3, f3);
  
  // now begin the series of tests
  
  if (!project6(n1, p1, p2, p3, q1, q2, q3)) return false;
  if (!project6(m1, p1, p2, p3, q1, q2, q3)) return false;
  
  if (!project6(ef11, p1, p2, p3, q1, q2, q3)) return false;
  if (!project6(ef12, p1, p2, p3, q1, q2, q3)) return false;
  if (!project6(ef13, p1, p2, p3, q1, q2, q3)) return false;
  if (!project6(ef21, p1, p2, p3, q1, q2, q3)) return false;
  if (!project6(ef22, p1, p2, p3, q1, q2, q3)) return false;
  if (!project6(ef23, p1, p2, p3, q1, q2, q3)) return false;
  if (!project6(ef31, p1, p2, p3, q1, q2, q3)) return false;
  if (!project6(ef32, p1, p2, p3, q1, q2, q3)) return false;
  if (!project6(ef33, p1, p2, p3, q1, q2, q3)) return false;
  
  if (!project6(g1, p1, p2, p3, q1, q2, q3)) return false;
  if (!project6(g2, p1, p2, p3, q1, q2, q3)) return false;
  if (!project6(g3, p1, p2, p3, q1, q2, q3)) return false;
  if (!project6(h1, p1, p2, p3, q1, q2, q3)) return false;
  if (!project6(h2, p1, p2, p3, q1, q2, q3)) return false;
  if (!project6(h3, p1, p2, p3, q1, q2, q3)) return false;
  
  return true;
}


bool OBBCollisionDetector::project6(const Vector3& ax, 
				    const Vector3& p1, const Vector3& p2, const Vector3& p3,
				    const Vector3& q1, const Vector3& q2, const Vector3& q3) const
{
  Real P1 = ax.dot(p1);
  Real P2 = ax.dot(p2);
  Real P3 = ax.dot(p3);
  Real Q1 = ax.dot(q1);
  Real Q2 = ax.dot(q2);
  Real Q3 = ax.dot(q3);
  
  Real mx1 = Math::maximum(P1, P2, P3);
  Real mn1 = Math::minimum(P1, P2, P3);
  Real mx2 = Math::maximum(Q1, Q2, Q3);
  Real mn2 = Math::minimum(Q1, Q2, Q3);
  
  if (mn1 > mx2) return false;
  if (mn2 > mx1) return false;
  return true;
}


Int OBBCollisionDetector::OBBDisjoint(const base::Matrix3& B, const gfx::Vector3& T,
				      const base::Vector3& a, const gfx::Vector3& b)
{
  Real t, s;
  
  // Bf = fabs(B)
  const Real reps = CollisionDetector::collisionEpsilon;
  Bf.e(1,1) = base::abs(B.e(1,1));  Bf.e(1,1) += reps;
  Bf.e(1,2) = base::abs(B.e(1,2));  Bf.e(1,2) += reps;
  Bf.e(1,3) = base::abs(B.e(1,3));  Bf.e(1,3) += reps;
  Bf.e(2,1) = base::abs(B.e(2,1));  Bf.e(2,1) += reps;
  Bf.e(2,2) = base::abs(B.e(2,2));  Bf.e(2,2) += reps;
  Bf.e(2,3) = base::abs(B.e(2,3));  Bf.e(2,3) += reps;
  Bf.e(3,1) = base::abs(B.e(3,1));  Bf.e(3,1) += reps;
  Bf.e(3,2) = base::abs(B.e(3,2));  Bf.e(3,2) += reps;
  Bf.e(3,3) = base::abs(B.e(3,3));  Bf.e(3,3) += reps;
  
  // if any of these tests are one-sided, then the polyhedra are disjoint
  
  // A1 x A2 = A0
  t = base::abs(T.x);
  
  if (!(t <= (a.x + b.x * Bf.e(1,1) + b.y * Bf.e(1,2) + b.z * Bf.e(1,3)))) return 1;
  
  // B1 x B2 = B0
  s = T.x*B.e(1,1) + T.y*B.e(2,1) + T.z*B.e(3,1);
  t = base::abs(s);
  
  if (!(t <= (b.x + a.x * Bf.e(1,1) + a.y * Bf.e(2,1) + a.z * Bf.e(3,1)))) return 2;
  
  // A2 x A0 = A1
  t = base::abs(T.y);
  
  if (!(t <= (a.y + b.x * Bf.e(2,1) + b.y * Bf.e(2,2) + b.z * Bf.e(2,3)))) return 3;
  
  // A0 x A1 = A2
  t = base::abs(T.z);
  
  if (!(t <= (a.z + b.x * Bf.e(3,1) + b.y * Bf.e(3,2) + b.z * Bf.e(3,3)))) return 4;
  
  // B2 x B0 = B1
  s = T.x*B.e(1,2) + T.y*B.e(2,2) + T.z*B.e(3,2);
  t = base::abs(s);
  
  if (!(t <=(b.y + a.x * Bf.e(1,2) + a.y * Bf.e(2,2) + a.z * Bf.e(3,2)))) return 5;
  
  // B0 x B1 = B2
  s = T.x*B.e(1,3) + T.y*B.e(2,3) + T.z*B.e(3,3);
  t = base::abs(s);
  
  if (!(t <= (b.z + a.x * Bf.e(1,3) + a.y * Bf.e(2,3) + a.z * Bf.e(3,3)))) return 6;
  
  // A0 x B0
  s = T.z * B.e(2,1) - T.y * B.e(3,1);
  t = base::abs(s);
  
  if (!(t <= (a.y * Bf.e(3,1) + a.z * Bf.e(2,1) + b.y * Bf.e(1,3) + b.z * Bf.e(1,2)))) return 7;
  
  // A0 x B1
  s = T.z * B.e(2,2) - T.y * B.e(3,2);
  t = base::abs(s);
  
  if (!(t <= (a.y * Bf.e(3,2) + a.z * Bf.e(2,2) + b.x * Bf.e(1,3) + b.z * Bf.e(1,1)))) return 8;
  
  // A0 x B2
  s = T.z * B.e(2,3) - T.y * B.e(3,3);
  t = base::abs(s);
  
  if (!(t <= (a.y * Bf.e(3,3) + a.z * Bf.e(2,3) + b.x * Bf.e(1,2) + b.y * Bf.e(1,1)))) return 9;
  
  // A1 x B0
  s = T.x * B.e(3,1) - T.z * B.e(1,1);
  t = base::abs(s);
  
  if (!(t <= (a.x * Bf.e(3,1) + a.z * Bf.e(1,1) + b.y * Bf.e(2,3) + b.z * Bf.e(2,2)))) return 10;
  
  // A1 x B1
  s = T.x * B.e(3,2) - T.z * B.e(1,2);
  t = base::abs(s);
  
  if (!(t <=(a.x * Bf.e(3,2) + a.z * Bf.e(1,2) + b.x * Bf.e(2,3) + b.z * Bf.e(2,1)))) return 11;
  
  // A1 x B2
  s = T.x * B.e(3,3) - T.z * B.e(1,3);
  t = base::abs(s);
  
  if (!(t <= (a.x * Bf.e(3,3) + a.z * Bf.e(1,3) + b.x * Bf.e(2,2) + b.y * Bf.e(2,1)))) return 12;
  
  // A2 x B0
  s = T.y * B.e(1,1) - T.x * B.e(2,1);
  t = base::abs(s);
  
  if (!(t <= (a.x * Bf.e(2,1) + a.y * Bf.e(1,1) + b.y * Bf.e(3,3) + b.z * Bf.e(3,2)))) return 13;
  
  // A2 x B1
  s = T.y * B.e(1,2) - T.x * B.e(2,2);
  t = base::abs(s);
  
  if (!(t <=(a.x * Bf.e(2,2) + a.y * Bf.e(1,2) + b.x * Bf.e(3,3) + b.z * Bf.e(3,1)))) return 14;
  
  // A2 x B2
  s = T.y * B.e(1,3) - T.x * B.e(2,3);
  t = base::abs(s);
  
  if (!(t <=(a.x * Bf.e(2,3) + a.y * Bf.e(1,3) + b.x * Bf.e(3,2) + b.y * Bf.e(3,1)))) return 15;
  
  return 0; 
  
}
