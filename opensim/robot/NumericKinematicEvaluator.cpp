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

  $Id: NumericKinematicEvaluator.cpp 1119 2004-09-27 22:14:54Z jungd $

****************************************************************************/

#include <robot/NumericKinematicEvaluator>


using robot::NumericKinematicEvaluator;
using robot::KinematicChain;

using base::Math;
using base::Vector;
using base::Matrix;


NumericKinematicEvaluator::NumericKinematicEvaluator()
{
}


void NumericKinematicEvaluator::getLinksForwardKinematics(const KinematicChain& chain, const Vector& q, array<Matrix>& T) const
{
  const Int n = chain.size(); // no. of links in the chain

  const Real trigEpsilon = 1.0e-5; // if sin or cos of angles are smaller, set sin=0 or cos=0

  if (n < 1) { // no links
    T.resize(0);
    return;
  }

  array<Matrix> A(n);  // array of matrices that give position and orientation with respect to previous frame
  T.resize(n);         // array of matrices that give position and orientation with respect to base

  // for A[i], the position and orientation matrix of frame for this link, relative to last one
  Int dof=0;
  for( Int l = 0; l < n; l++ ) {
    const KinematicChain::Link& link( chain[l] );
    A[l].resize(4,4); T[l].resize(4,4);

    Real theta, d;

    if (link.isDHType()) {	// D-H type joints (Revolute, Prismatic)

      // determine what is constant and what is variable to set expressions for theta and d
      if (link.type() == KinematicChain::Link::Revolute) {
        if (link.isActive())
          theta = q[dof];
        else
          theta = link.getTheta(); // inactive joints are considered to be fixed in the home position
          d = link.getD();
      }
      else { // Prismatic
        theta = link.getTheta();
        if (link.isActive())
          d = q[dof];
        else
          d = link.getD();        // inactive joints are considered to be fixed in the home position
      }


      Real sinAlpha = Math::zeroIfNeighbour( Math::sin(link.getAlpha()), trigEpsilon );
      Real cosAlpha = Math::zeroIfNeighbour( Math::cos(link.getAlpha()), trigEpsilon );
      Real sinTheta = Math::sin(theta);
      Real cosTheta = Math::cos(theta);

      // form A[i], which represents the position and orientation of frame i relative to frame i-1
      A[l](0,0)=cosTheta; A[l](0,1)=-sinTheta*cosAlpha; A[l](0,2)=sinTheta*sinAlpha;  A[l](0,3)=link.getA()*cosTheta;
      A[l](1,0)=sinTheta; A[l](1,1)=cosTheta*cosAlpha;  A[l](1,2)=-cosTheta*sinAlpha; A[l](1,3)=link.getA()*sinTheta;
      A[l](2,0)=0;        A[l](2,1)=sinAlpha;           A[l](2,2)=cosAlpha;           A[l](2,3)=d;
      A[l](3,0)=0;        A[l](3,1)=0;                  A[l](3,2)=0;                  A[l](3,3)=1;

    }
    else { // non-D-H type joint

      if (link.type() == KinematicChain::Link::Translating) {

        Real t = q[dof]; // translation distance variable
        Assert(link.getDirection().length() > 0);
        Vector3 dir( link.getDirection() ); dir.normalize(); // make unit length

        // simple translation by t*dir
        A[l](0,0)=1; A[l](0,1)=0; A[l](0,2)=0; A[l](0,3)=dir.x*t;
        A[l](1,0)=0; A[l](1,1)=1; A[l](1,2)=0; A[l](1,3)=dir.y*t;
        A[l](2,0)=0; A[l](2,1)=0; A[l](2,2)=1; A[l](2,3)=dir.z*t;
        A[l](3,0)=0; A[l](3,1)=0; A[l](3,2)=0; A[l](3,3)=1;

      } // end Translating
      else if (link.type() == KinematicChain::Link::FixedTransform) {

        // this is a 0-dof transforming link
        A[l] = base::fromMatrix4(link.getTransform());


      } // end FixedTransform
      else
        throw std::runtime_error(Exception("unsupported joint type in chain - can't calculate forward kinematic transform"));
    }


    if ( l < 1 )
      T[l] = A[l];
    else
      T[l] = T[l-1] * A[l];

    dof += link.dof();

  } // for l

}


Matrix NumericKinematicEvaluator::getForwardKinematics(const KinematicChain& chain, const Vector& q) const
{
  const Int n = chain.size(); // no. of links in the chain

  if (n < 1) // no links, return identity
    return base::identityMatrix(4,4);

  array<Matrix> T;
  getLinksForwardKinematics(chain, q, T);
  return T[n-1];
}


/// \TODO implement
Matrix NumericKinematicEvaluator::getJacobian(const KinematicChain& chain, const Vector& q, bool includeOrientation) const
{
  Unimplemented;
}




/// \TODO implement
array<Vector> NumericKinematicEvaluator::getJointOrigins(const KinematicChain& chain, const Vector& q ) const
{
  Unimplemented;
}


/// \TODO implement
array<Vector> NumericKinematicEvaluator::getLinkOrigins(const KinematicChain& chain, const Vector& q ) const
{
  Unimplemented;
}

