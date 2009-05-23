/****************************************************************************
  Copyright (C)2002 David Jung <opensim@pobox.com>

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

  $Id: JFKengine.cpp 1039 2004-02-11 20:50:52Z jungd $
  $Revision: 1.5 $
  $Date: 2004-02-11 15:50:52 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $

****************************************************************************/

#include <robot/JFKengine>
#include <base/base>

#include <base/Time>

using robot::JFKengine;
using robot::KinematicChain;

using base::Math;
using base::Time;
using base::Vector;
using base::Vector3;
using base::Matrix;
using base::Expression;
using base::ExpressionMatrix;


JFKengine::JFKengine() 
  : forwardKinematicsCached(false), A(0), T(0), 
    JCached(false), J(1,1)
{
}


ExpressionMatrix JFKengine::getForwardKinematics(const KinematicChain& chain) const
{ 
  const Int n = chain.size(); // no. of links in the chain

  const Real trigEpsilon = 1.0e-5; // if sin or cos of angles are smaller, set sin=0 or cos=0

  if (n < 1) { // no links, return identity
    ExpressionMatrix I(4,4);
    I(0,0)=I(1,1)=I(2,2)=I(3,3)=1;
    return I;
  }

  if ( A.size() != n ) {
    // array resize is non-descructive (except the the elements truncated when resizing smaller)
    A.resize(n);
    T.resize(n);
    chain_AT.resize(n);
  }

  Expression zero = 0;
  Expression one = 1;

  // form A[i], the position and orientation matrix of frame for this link, relative to last one
  Int dof = 0;
  for ( Int l = 0; l < n; l++ ) {
    const KinematicChain::Link& link( chain[l] );
    //Debugcln(JFKE, "\n=====getForwardKinematics loop, joint " << (i+1) << ": " << joint);

    // determine if we're already computed and cached the A[l], the transform between
    //  the last link and this one 
    if (forwardKinematicsCached) { // something is cached, see if it is what we want
      if ( chain_AT[l] != link ) 
	forwardKinematicsCached = false;  // must recompute A & T from this link on, since link is different
    }


    if (!forwardKinematicsCached) {

      A[l].resize(4,4); T[l].resize(4,4);
      chain_AT.setLink(l, link);  // for caching, save parameters used to compute A & T
      //Debugcln(JFKE,"JFKEngine(): FK link " << l << "(dof=" << dof << "):" << link);

      Expression theta, d; // used to set variables for Denavit-Hartenberg notation
      if (link.isDHType()) {	// D-H type joints (Revolute, Prismatic)

	// determine what is constant and what is variable to set expressions for theta and d
	if (link.type() == KinematicChain::Link::Revolute) {
	  if (link.isActive())
	    theta = Expression::p[dof];
	  else
	    theta = link.getTheta(); // inactive joints are considered to be fixed in the home position
	  d = link.getD();
	}
	else { // Prismatic
	  theta = link.getTheta();
	  if (link.isActive())
	    d = Expression::p[dof];
	  else
	    d = link.getD();        // inactive joints are considered to be fixed in the home position
	}

	Real sinAlpha = Math::zeroIfNeighbour( Math::sin(link.getAlpha()), trigEpsilon );
	Real cosAlpha = Math::zeroIfNeighbour( Math::cos(link.getAlpha()), trigEpsilon );
	Expression sinTheta = sin(theta);
	Expression cosTheta = cos(theta);
	
	// form A[i], which represents the position and orientation of frame i relative to frame i-1
	A[l](0,0)=cosTheta; A[l](0,1)=-sinTheta*cosAlpha; A[l](0,2)=sinTheta*sinAlpha;  A[l](0,3)=link.getA()*cosTheta;
	A[l](1,0)=sinTheta; A[l](1,1)=cosTheta*cosAlpha;  A[l](1,2)=-cosTheta*sinAlpha; A[l](1,3)=link.getA()*sinTheta;
	A[l](2,0)=zero;     A[l](2,1)=sinAlpha;           A[l](2,2)=cosAlpha;           A[l](2,3)=d;
	A[l](3,0)=zero;     A[l](3,1)=zero;               A[l](3,2)=zero;               A[l](3,3)=one;

      }	
      else { // non-D-H type joint

	if (link.type() == KinematicChain::Link::Translating) {

	  Expression t = Expression::p[dof]; // translation distance variable
	  Assert(link.getDirection().length() > 0);
	  Vector3 dir( link.getDirection() ); dir.normalize(); // make unit length
	  
	  // simple translation by t*dir
	  A[l](0,0)=one;  A[l](0,1)=zero; A[l](0,2)=zero; A[l](0,3)=dir.x*t;
	  A[l](1,0)=zero; A[l](1,1)=one;  A[l](1,2)=zero; A[l](1,3)=dir.y*t;
	  A[l](2,0)=zero; A[l](2,1)=zero; A[l](2,2)=one;  A[l](2,3)=dir.z*t;
	  A[l](3,0)=zero; A[l](3,1)=zero; A[l](3,2)=zero; A[l](3,3)=one;

	} // end Translating
	else if (link.type() == KinematicChain::Link::FixedTransform) {

	  // this is a 0-dof transforming link
	  A[l] = base::toExpressionMatrix(base::fromMatrix4(link.getTransform()));


	} // end FixedTransform
	else
	  throw std::runtime_error(Exception("unsupported joint type in chain - can't calculate forward kinematic transform"));

      }

      simplify( A[l] );

      if ( l < 1 )
	T[l] = A[l];
      else {
	T[l] = T[l-1] * A[l];
	simplify( T[l] );
      }
   
      //Debugcln(JFKE, "\nA[" << (i+1) << "] matrix after simplification:\n" << A[i]);
      //Debugcln(JFKE, "\nT[" << (i+1) << "] matrix after simplification:\n" << T[i]);
    }

    dof += link.dof();
    // end single link contribution

  } // for l

  
  //Debugc( JFKE, "Homogeneous transformation matrix T[" << n << "]:");
 
  /*
  for (Int iRow=0; iRow<4; iRow++) {
    Debugc( JFKE, "\n");
    for (Int iCol=0; iCol<4; iCol++) {
      Debugcln( JFKE, "  (" << iRow << "," << iCol << ")=" << T[n-1](iRow,iCol));
    }
  }
  */
  
  forwardKinematicsCached = true;

  //Debugcln(KF, "\n" << chain_AT.size() << " Cached JointParamters in chain_AT\n" << chain_AT);
  return T[n-1];
}



ExpressionMatrix JFKengine::getJacobian(const KinematicChain& chain, bool includeOrientation) const
{
  const Int n = chain.size(); 
  const Int chain_dof = chain.dof(); // no. of variables / degrees-of-freedom

  if (chain.dof() < 1) { // no joints, return zero/empty matrix
    ExpressionMatrix Z(includeOrientation?6:3,0);
    return Z;
  }
 
  //Debugln(JFKE, "\n");

  // see if cached J can be used
  if ( chain_J.dof() != chain_dof ) {
    JCached = false;  // must recompute J because not same number of joints as what is cached
  }
  else {
    if (chain_J.hashCode() != chain.hashCode())
      JCached = false;  // must recompute J because joints are not the same as what is cached
    else {
      if (includeOrientation) {
        if (J.size1() != 6)
          JCached = false; // orientation components weren't cached
      }
      else {
        // if cached J includes orientation components we don't need, discard them
        ExpressionMatrix cachedJ(J); // make copy
        J.resize(3,cachedJ.size2()); // destructive resize
        // copy position components back
        for(Int r=0; r<3; r++)
          for(Int c=0; c<J.size2(); c++)
            J(r,c) = cachedJ(r,c);
      }
    }
  }
  

  if (!JCached) { // recompute
    #ifdef DEBUG
    base::Time start( Time::now() );
    #endif

    chain_J = chain;
    J.resize(includeOrientation?6:3, chain_dof);

    getForwardKinematics( chain );  // ensure manipulator arm expression T is cached
 
    Expression zero = 0;
    Expression one = 1;

    for (Int dof=0, l=0; l < n; l++) { // for each link
      const KinematicChain::Link& link( chain[l] );

      if (link.dof() > 0) { 

	// Jacobian rows 1,2,3:  differentiate x,y,z: fourth column of T
	for (Int iVar=0; iVar < 3; iVar++) {
	  //Debugcln(KF, "J(" << iVar << "," << iJoint << ") Before differentiating, expression= " << T[n-1](iVar,3) );
	  J(iVar,dof) = T[n-1](iVar,3).differentiate( Expression::p[dof] );
	  J(iVar,dof).simplify();
	  //Debugcln(KF, "J(" << iVar << "," << iJoint << ") after differentiating =" << J(iVar,iJoint) );
	}
	
	if (includeOrientation) {
	  // Jacobian rows 4,5,6: 
	  const  KinematicChain::Link::LinkType linkType( (chain[l]).type() );
	  switch (linkType) {
	  case KinematicChain::Link::Revolute:	
	    if (dof < 1) {
	      J(3,0) = J(4,0) = zero; // [0,0,1] for first column, by definition
	      J(5,0) = one;
	    }
	    else {
	      J(3,dof) = T[l-1](0,2); // from position in cumulative transformation matrix T
	      J(4,dof) = T[l-1](1,2);
	      J(5,dof) = T[l-1](2,2);
	    }
	    break;
	    
	  case KinematicChain::Link::Prismatic:
	  case KinematicChain::Link::Translating:
	    for (Int i=3; i< 6; i++) {
	      J(i,dof) = zero;  // no angular velocity contribution for prismatic or translating joints
	    }
	    break;
	  default:
	    throw std::runtime_error(Exception("unsupported joint type in chain - can't compute Jacobian"));
	  }
	} // end if(includeOrientation)

      } // if link.dof() > 0

      dof += link.dof();

    } // for each link
    
    /*
    Debugcln( JFKE, "Jacobian matrix: \n");
    for (Int iJoint = 0; iJoint<n; iJoint++) {
      Debugcln( JFKE, "J[" << (iJoint+1) << "]=");
      for (Int iRow=0; iRow<(includeOrientation?6:3); iRow++) {
        Debugcln( JFKE, "\t" << J(iRow,iJoint) << "\n");
      }
    }
    */

    JCached = true;
    //Debugcln(KF, "\n" << chain_J.size() << " Cached JointParamters in chain_J\n" << chain_J);
    #ifdef DEBUG
    Debugln( JFKE, "Symbolic Jacobian computation for " << chain.size() 
                 << " link manipulator took " << (Time::now()-start).seconds() << "s");
    #endif
  }  // end JCached else
  return J;
}



Matrix JFKengine::getForwardKinematics(const KinematicChain& chain, const Vector& q) const
{
  ExpressionMatrix F( getForwardKinematics(chain) );
  return evaluate(F,q);
}


Matrix JFKengine::getJacobian(const KinematicChain& chain, const Vector& q, bool includeOrientation) const
{
  ExpressionMatrix J( getJacobian(chain, includeOrientation) );
  return evaluate(J,q);
}




array<Vector> JFKengine::getJointOrigins(const KinematicChain& chain, const Vector& q ) const
{
  if ( chain.dof() < 1 )
    return array<Vector>(0); // no joints

  Assert(q.size() == chain.dof() );

  // NB: assumes no joints with dof > 1 !  i.e. 1<->1  joints<->dofs
  
  const Int chain_dof = chain.dof();

  array<Vector> locs(chain_dof); 
  getForwardKinematics(chain); // cache T

  for (Int v=0; v < chain_dof; v++ ) {
    locs[v].resize(3);
    Int li = chain.linkIndexOfVariable(v);
    for ( Int iDimension=0; iDimension < 3; iDimension++ ) {
      locs[v][iDimension] = T[li](iDimension,3).evaluate( q ); 
    }
  }
 
  return locs;
}


array<Vector> JFKengine::getLinkOrigins(const KinematicChain& chain, const Vector& q ) const
{
  if ( chain.size() < 1 )
    return array<Vector>(0); // no links

  Assert(q.size() == chain.dof() );

  const Int chain_size = chain.size();

  array<Vector> locs(chain_size); 
  getForwardKinematics(chain); // cache T

  for (Int li=0; li < chain_size; li++ ) {
    locs[li].resize(3);
    for ( Int iDimension=0; iDimension < 3; iDimension++ ) {
      locs[li][iDimension] = T[li](iDimension,3).evaluate( q );  
    }
  }
 
  //Debugln(JFKE, "\ngetLinkOrigins :" << chain );
  //for ( Int i=0; i<chain_size; i++ ) {
  //  Debugcln(JFKE, "link " << i << " location relative to first link: [" << locs[i][0] << ", " << locs[i][1] << ", " << locs[i][2] << "]");
  //}

  return locs;
}
