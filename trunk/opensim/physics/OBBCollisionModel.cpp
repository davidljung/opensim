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
  
  $Id: OBBCollisionModel.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.6 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/OBBCollisionModel>

#include <physics/Box>
#include <gfx/TriangleContainer>
#include <gfx/TriangleIterator>
#include <gfx/Triangle3>
#include <gfx/Color4>

#include <osg/Node>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/StateSet>
#include <osg/Material>
#include <osg/Vec4>
#include <osg/PolygonMode>



using physics::OBBCollisionModel;
using physics::Box;

using gfx::Triangle3;
using gfx::Color4;
using gfx::TriangleContainer;
using gfx::TriangleIterator;

using base::Matrix3;
using base::transpose;
using base::cross;
using base::array;

using osg::Vec4;
using osg::StateSet;


OBBCollisionModel::OBBCollisionModel(const gfx::TriangleContainer& triangles)
  : b(*new base::array<OBB>()),	tris(*new base::array<Triangle3>())
{
  // Copy all the triangles into an array (tris)
  Int id=0;
  
  TriangleContainer::const_iterator t = triangles.begin();
  TriangleContainer::const_iterator end = triangles.end();
  while (t != end) {
    tris.at(id++) = (*t);
    ++t;
  }
  tris.trim();

  if (!tris.empty()) {
    buildHierarchy(); // Build the model
  }

}

OBBCollisionModel::OBBCollisionModel(const OBBCollisionModel& cm)
  : b(*new base::array<OBB>(cm.b)),
    tris(*new base::array<Triangle3>(cm.tris))
{
}

OBBCollisionModel::~OBBCollisionModel()
{
  delete &tris;
  delete &b;
}


void OBBCollisionModel::buildHierarchy()
{
  array<Moment> moment(tris.size());
  array<OBB>& boxes(b);
  Int OBBsInited = 1; 
  boxes.resize(tris.size()*2);
  
  // Determine initial orientation, mean point, and splitting axis.
  Moment  M;
  Matrix3 C;
  
  Moment::computeMoment(moment, tris, 0, tris.size());

  M.clear();
  for(Int i=0; i<tris.size(); i++) {
    M.accumulate(moment[i]);
  }

  b[0].pT = M.meanFromAccum();
  C = M.covariance();
  
  eigenAndSort1(b[0].pR, C);
  
  // create the index list
  Int* t = new Int[tris.size()];
  for(Int i=0; i<tris.size(); i++) t[i]=i;
  
  // do the build
  OBBsInited = b[0].splitRecurse(tris,moment,boxes,OBBsInited,t,0,tris.size());
  
  delete[] t;
}


Int OBBCollisionModel::eigenAndSort1(Matrix3& evecs, const Matrix3& cov, Int maxIterations)
{
  Vector3 evals;
  
  Int n = cov.eigenJacobi(evecs, evals, maxIterations);

  // not a full sort -- just makes column 1 the largest
  if (evals.z > evals.x) {
    if (evals.z > evals.y) {
      // 3 is largest, swap with column 1
      evecs.swapColumns(3,1);
    }
    else {
      // 2 is largest, swap with column 1
      evecs.swapColumns(2,1);
    }
  }
  else {
    if (evals.x > evals.y) {
      // 1 is largest, do nothing
    }
    else {
      // 2 is largest
      evecs.swapColumns(1,2);
    }
  }
  
  // we are returning the number of iterations eigenJacobi took.
  // too many iterations means our chosen orientation is bad.
  return n; 
}









// OBB implementation

void OBBCollisionModel::OBB::reaccumMoments(Moment& A, base::array<Moment>& moment,
					    Int t[], Int f, Int n)
{
  A.clear();
  for(Int i=0;i<n;i++) 
    A.accumulate(moment[t[f+i]]);
}


Int OBBCollisionModel::OBB::splitRecurse(base::array<gfx::Triangle3>& tri, 
					 base::array<Moment>& moment, 
					 base::array<OBB>& boxes,
					 Int OBBsInited, Int t[], Int f, Int n)
{
  // The orientation for the parent box is already assigned to this->pR.
  // The axis along which to split will be column 0 of this->pR.
  // The mean point is passed in on this->pT.
  
  // When this routine completes, the position and orientation in model
  // space will be established, as well as its dimensions.  Child boxes
  // will be constructed and placed in the parent's CS.
  
  if (n == 1)
    return splitRecurse(tri,moment,boxes,OBBsInited,t,f);
  
  
  // walk along the tris for the box, and do the following:
  //   1. collect the max and min of the vertices along the axes of <or>.
  //   2. decide which group the triangle goes in, performing appropriate swap.
  //   3. accumulate the mean point and covariance data for that triangle.
  
  Moment M1, M2;
  Matrix3 C;
  Vector3 c;
  Vector3 minval, maxval;
  
  Int in;
  Int i;
  Real axdmp;
  Int n1 = 0;  // The number of tris in group 1.  
  // Group 2 will have n - n1 tris.
	
  // project approximate mean point onto splitting axis, and get coord.
  axdmp = (pR.e(1,1) * pT.x + pR.e(2,1) * pT.y + pR.e(3,1) * pT.z);
  
  M1.clear();
  M2.clear();

  Matrix3 pRT = transpose(pR);
  
  c = pRT * tri[t[f]].p1();
  
  minval = c;
  maxval = c;

  for(i=0; i<n; i++) {
    in = t[f+i];
    Triangle3& ptr(tri[in]);
    
    c = pRT*ptr.p1();
    minmax(minval, maxval, c);
    
    c = pRT*ptr.p2();
    minmax(minval, maxval, c);
    
    c = pRT*ptr.p3();
    minmax(minval, maxval, c);
    
    // grab the mean point of the in'th triangle, project
    // it onto the splitting axis (1st column of pR) and
    // see where it lies with respect to axdmp.
    c = moment[in].meanFromMoment();
    
    if ( (((pR.e(1,1)*c.x + pR.e(2,1)*c.y + pR.e(3,1)*c.z) < axdmp)
	  && (n!=2)) || ((n==2) && (i==0)) ) {
      // accumulate first and second order moments for group 1
      M1.accumulate(moment[in]);
      
      // put it in group 1 by swapping t[i] with t[n1]
      Int temp = t[f+i];
      t[f+i] = t[f+n1];
      t[f+n1] = temp;
      n1++;
    }
    else {
      // accumulate first and second order moments for group 2
      M2.accumulate(moment[in]);
      
      // leave it in group 2
      // do nothing...it happens by default
    }
  }
  
  // done using this.pT as a mean point.
  
  // error check!
  if ((n1 == 0) || (n1 == n)) {
    // our partitioning has failed: all the triangles fell into just
    // one of the groups.  So, we arbitrarily partition them into
    // equal parts, and proceed.
    //Debugln("Warning: Triangle partitioning failed. Partitioning equally.");    
    n1 = n/2;
    
    // now recompute accumulated stuff
    reaccumMoments(M1, moment, t, f, n1);
    reaccumMoments(M2, moment, t, f + n1, n - n1);
  }
  
  // With the max and min data, determine the center point and dimensions
  // of the parent box.
  
  c = (minval+maxval)/2.0;
  
  pT = pR*c; // equiv of above?? !!!
  d = (maxval-minval)/2.0;
  
  // allocate new boxes
  P = &boxes[OBBsInited++];
  N = &boxes[OBBsInited++];
	
  // Compute the orienations for the child boxes (eigenvectors of
  // covariance matrix).  Select the direction of maximum spread to be
  // the split axis for each child.
  
  Matrix3 tR;
  
  // do the P child
  if (n1 > 1) {
    P->pT = M1.meanFromAccum();
    C = M1.covariance();
    
    if (OBBCollisionModel::eigenAndSort1(tR, C, 30) >= 30) {
      // unable to find an orientation.  We'll just pick identity.
      tR.setIdentity();
    }
    
    P->pR = tR;
    OBBsInited = P->splitRecurse(tri, moment, boxes, OBBsInited, t, f, n1);
  }
  else {
    OBBsInited = P->splitRecurse(tri, moment, boxes, OBBsInited, t, f);
  }
  C = P->pR; P->pR = transpose(pR)*C;  // and F1 
  c = P->pT - pT; P->pT = transpose(pR)*c;
  
  // do the N child
  if ((n-n1) > 1) {
    N->pT = M2.meanFromAccum();
    C = M2.covariance();
    
    if (OBBCollisionModel::eigenAndSort1(tR, C) > 30) {
      // unable to find an orientation.  We'll just pick identity.
      tR.setIdentity();
    }
    
    N->pR = tR;
    OBBsInited = N->splitRecurse(tri, moment, boxes, OBBsInited, t, f + n1, n - n1);
  }
  else {
    OBBsInited = N->splitRecurse(tri, moment, boxes, OBBsInited, t, f+n1);
  }
  C = N->pR; N->pR = transpose(pR)*C;
  c = N->pT-pT; N->pT = transpose(pR)*c;
  
  return OBBsInited;
}


// specialized for leaf nodes
Int OBBCollisionModel::OBB::splitRecurse(base::array<gfx::Triangle3>& tri, 
					 base::array<Moment>& moment, 
					 base::array<OBB>& boxes,
					 Int OBBsInited, Int t[], Int f)
{
  // For a single triangle, orientation is easily determined.
  // The major axis is parallel to the longest edge.
  // The minor axis is normal to the triangle.
  // The in-between axis is determine by these two.
  
  // this.pR, this.d, and this.pT are set herein.
  
  P = N = 0;
  Triangle3& ptr(tri[t[f]]);
  
  // Find the major axis: parallel to the longest edge.
  Vector3 u12, u23, u31;
  
  // First compute the squared-lengths of each edge
  u12 = ptr.p1()-ptr.p2();
  Real d12 = u12.norm();
  u23 = ptr.p2()-ptr.p3();  
  Real d23 = u23.norm();
  u31 = ptr.p3()-ptr.p1();  
  Real d31 = u31.norm();
  
  // Find the edge of longest squared-length, normalize it to
  // unit length, and put result into a0.
  Vector3 a0;
  if (d12 > d23) {
    if (d12 > d31) 
      a0 = u12 / base::sqrt(d12);
    else 
      a0 = u31 / base::sqrt(d31);
  }
  else {
    if (d23 > d31) 
      a0 = u23 / base::sqrt(d23);
    else 
      a0 = u31 / base::sqrt(d31);
  }
  
  // Now compute unit normal to triangle, and put into a2.
  Vector3 a2;
  a2 = cross(u12,u23);
  a2.normalize();
  
  // a1 is a2 cross a0.
  Vector3 a1;
  a1 = cross(a2,a0);
  
  // Now make the columns of this.pR the vectors a0, a1, and a2.
  pR.e(1,1) = a0.x;  pR.e(1,2) = a1.x;  pR.e(1,3) = a2.x;
  pR.e(2,1) = a0.y;  pR.e(2,2) = a1.y;  pR.e(2,3) = a2.y;
  pR.e(3,1) = a0.z;  pR.e(3,2) = a1.z;  pR.e(3,3) = a2.z;
  
  // Now compute the maximum and minimum extents of each vertex 
  // along each of the box axes.  From this we will compute the 
  // box center and box dimensions.
  Vector3 minval, maxval;
  Vector3 c;
  
  c = transpose(pR)*ptr.p1();
  minval=c; maxval=c;
  
  c = transpose(pR)*ptr.p2();
  minmax(minval, maxval, c);
  
  c = transpose(pR)*ptr.p3();
  minmax(minval, maxval, c);
  
  // With the max and min data, determine the center point and dimensions
  // of the box
  c = (minval+maxval)/2.0;
  
  pT = pR*c; // !!! check equiv of above	
  d = (maxval-minval)/2.0;
  
  // Assign the one triangle to this box
  tr = ptr;
  
  return OBBsInited;
}


void OBBCollisionModel::OBB::minmax(gfx::Vector3& min, gfx::Vector3& max, 
				    const gfx::Vector3& v)
{
  if (v.x < min.x) 
    min.x = v.x;
  else
    if (v.x > max.x)
      max.x = v.x;
  if (v.y < min.y) 
    min.y = v.y;
  else
    if (v.y > max.y)
      max.y = v.y;
  if (v.z < min.z) 
    min.z = v.z;
  else
    if (v.z > max.z)
      max.z = v.z;
}



// Moment implementation

void OBBCollisionModel::Moment::clear()
{
  A = 0;
  m.setZero();
  s.setZero();
}


void OBBCollisionModel::Moment::accumulate(const Moment& b)
{
  m += b.m*b.A;
  s += b.s;
  A += b.A;
}


base::Matrix3 OBBCollisionModel::Moment::covariance() const 
{
  base::Matrix3 C = s;
  C.e(1,1) -= m.x*m.x/A;
  C.e(1,2) -= m.x*m.y/A;
  C.e(1,3) -= m.x*m.z/A;
  C.e(2,1) -= m.y*m.x/A;
  C.e(2,2) -= m.y*m.y/A;
  C.e(2,3) -= m.y*m.z/A;
  C.e(3,1) -= m.z*m.x/A;
  C.e(3,2) -= m.z*m.y/A;
  C.e(3,3) -= m.z*m.z/A;
  return C;
}


void OBBCollisionModel::Moment::computeMoment(Moment& M, const base::Vector3& p,
					      const base::Vector3& q, const base::Vector3& r)
{
  Vector3 u,v,w;
  
  // compute the area of the triangle
  u = q-p;
  v = r-p;
  w = cross(u,v);
  M.A = 0.5*w.length();
  
  if (M.A == 0.0) {
    // This triangle has zero area.  The second order components
    // would be eliminated with the usual formula, so, for the 
    // sake of robustness we use an alternative form.  These are the 
    // centroid and second-order components of the triangle's vertices.
    
    // centroid
    M.m = (p+q+r)/3.0;
    
    // second-order components
    M.s.e(1,1) = (p.x*p.x + q.x*q.x + r.x*r.x);
    M.s.e(1,2) = (p.x*p.y + q.x*q.y + r.x*r.y);
    M.s.e(1,3) = (p.x*p.z + q.x*q.z + r.x*r.z);
    M.s.e(2,2) = (p.y*p.y + q.y*q.y + r.y*r.y);
    M.s.e(2,3) = (p.y*p.z + q.y*q.z + r.y*r.z);
    M.s.e(3,3) = (p.z*p.z + q.z*q.z + r.z*r.z);      
    M.s.e(3,2) = M.s.e(2,3);
    M.s.e(2,1) = M.s.e(1,2);
    M.s.e(3,1) = M.s.e(1,3);
    
    return;
  }
  
  // get the centroid
  M.m = (p+q+r)/3.0;
  
  // get the second order components -- note the weighting by the area
  M.s.e(1,1) = M.A*(9.0*M.m.x*M.m.x+p.x*p.x+q.x*q.x+r.x*r.x)/12.0;
  M.s.e(1,2) = M.A*(9.0*M.m.x*M.m.y+p.x*p.y+q.x*q.y+r.x*r.y)/12.0;
  M.s.e(1,3) = M.A*(9.0*M.m.x*M.m.z+p.x*p.z+q.x*q.z+r.x*r.z)/12.0;
  M.s.e(2,2) = M.A*(9.0*M.m.y*M.m.y+p.y*p.y+q.y*q.y+r.y*r.y)/12.0;
  M.s.e(2,3) = M.A*(9.0*M.m.y*M.m.z+p.y*p.z+q.y*q.z+r.y*r.z)/12.0;
  M.s.e(3,3) = M.A*(9.0*M.m.z*M.m.z+p.z*p.z+q.z*q.z+r.z*r.z)/12.0;
  M.s.e(3,2) = M.s.e(2,3);
  M.s.e(2,1) = M.s.e(1,2);
  M.s.e(3,1) = M.s.e(1,3);
}


void OBBCollisionModel::Moment::computeMoment(base::array<Moment>& M,
					      base::array<gfx::Triangle3>& tris,
					      Int firstTri, Int numTris)
{
  // check implementation. (tri *tris, int num_tris)
  
  // first collect all the moments, and obtain the area of the 
  // smallest nonzero area triangle.
  Real Amin = 0.0;
  bool zero=false;

  Triangle3 tri;
  Moment m;
  for(Int i=firstTri; i<firstTri+numTris; i++) {
    Moment& m(M[i]);
    tri = tris[i];
    computeMoment(m, tri.p1(), tri.p2(), tri.p3());

    if (m.A == 0.0)
      zero=true;
    else {
      if (Amin == 0.0) 
	Amin = m.A;
      else 
	if (m.A < Amin)
	  Amin = m.A;
    }
    
  }
  
  if (zero) {
    Logfln("Warning: Some triangles have zero area.");

    // if there are any zero area triangles, go back and set their area
    // if ALL the triangles have zero area, then set the area thingy
    // to some arbitrary value.
    if (Amin == 0.0) Amin = 1.0;
    
    for(Int i=firstTri; i<firstTri+numTris; i++) {
      Moment& m(M[i]);
      if ( m.A == 0.0) 
	m.A = Amin;
    }
  }
}



osg::Node* OBBCollisionModel::createOSGVisual(Visual::Attributes visualAttributes) const
{
  if (!(visualAttributes & ShowCollisionModel)
      || tris.empty() ) 
    return new osg::Node();
  
  osg::Node* node = createOBBVisualRecurse(b[0], 0);
  node->setName("debug");

  // Set state to be transparent, random colour
  StateSet* state = new osg::StateSet();
  osg::Material* mat = new osg::Material();
  Vec4 col( Math::random(), Math::random(), Math::random(), 1.0);
  mat->setEmission( osg::Material::FRONT_AND_BACK, Vec4(0,0,0,0) );
  mat->setAmbient( osg::Material::FRONT_AND_BACK, col );
  mat->setDiffuse( osg::Material::FRONT_AND_BACK, col );
  mat->setSpecular( osg::Material::FRONT_AND_BACK, Vec4(1,1,1,0) );
  mat->setShininess( osg::Material::FRONT_AND_BACK, 0.3);
  state->setAttribute( mat );
  state->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);

  osg::PolygonMode* polyMode = new osg::PolygonMode;
  polyMode->setMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );
  state->setAttributeAndModes(polyMode,osg::StateAttribute::ON);

  node->setStateSet(state);
  
  return node;
}

osg::Node* OBBCollisionModel::createOBBVisualRecurse(OBB& obb, Int level) const
{
  const Real epsOffset = 0.01;

  osg::MatrixTransform* tn = new osg::MatrixTransform();
  tn->setName("OBB");
  //  tn->setType(osg::Transform::STATIC); // is there an equiv??? !!!
  Matrix4 m(obb.pR);
  m.setTranslationComponent(obb.pT);
  tn->setMatrix(m);
  
  if (!onlyOneLevel || (onlyLevel==level)) {
    ref<Box> box(NewObj Box(2*obb.d.x+epsOffset,2*obb.d.y+epsOffset,2*obb.d.z+epsOffset));
    Debugln(Physics,"box leaf " << level);
    StateSet* state = new osg::StateSet();
    osg::Material* mat = new osg::Material();
    Vec4 col( Math::random(), Math::random(), Math::random(), 1.0);
    mat->setAmbient( osg::Material::FRONT_AND_BACK, col );
    mat->setDiffuse( osg::Material::FRONT_AND_BACK, col );
    state->setAttribute( mat );

    osg::PolygonMode* polyMode = new osg::PolygonMode;
    polyMode->setMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );
    state->setAttributeAndModes(polyMode,osg::StateAttribute::ON);

    osg::Node* boxNode = box->createOSGVisual();
    boxNode->setStateSet(state);
    tn->addChild(boxNode);
  }
  
  if (level < maxLevels) {
    if (obb.N != 0) {
      tn->addChild( createOBBVisualRecurse(*obb.N, level+1) );
    }
    if (obb.P != 0) {
      tn->addChild( createOBBVisualRecurse(*obb.P, level+1) );
    }
  }
  
  return tn;
}
