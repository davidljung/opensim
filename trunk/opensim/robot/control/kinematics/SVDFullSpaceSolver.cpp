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

  $Id: SVDFullSpaceSolver.cpp 1080 2004-07-28 19:51:26Z jungd $
  $Revision: 1.4 $
  $Date: 2004-07-28 15:51:26 -0400 (Wed, 28 Jul 2004) $
  $Author: jungd $

****************************************************************************/

#include <robot/control/kinematics/SVDFullSpaceSolver>

using robot::control::kinematics::SVDFullSpaceSolver;

#include <base/SVD>
#include <robot/control/kinematics/solution_error>

//
// Solution coded from Lonnie Love's Matlab code
//


using base::IVector;
using base::zeroIVector;
using base::equals;
using base::SVD;

const Real small = 1.0e-04; // (=def.SMALL)


SVDFullSpaceSolver::SVDFullSpaceSolver()
 : stopOnIllCondition(false)
{
}



const static Real maxCondition = 10000;

base::Matrix SVDFullSpaceSolver::solve(const Matrix& A_in, const Vector& b_in,
                                       array<Int>& dependentRowsEliminated)
{
  const Int N = A_in.rows();
  const Int M = A_in.cols();

  const Vector& b(b_in);
  Assert( b.size() == N);

  bool nullSpace = b.equals(zeroVector(N));

  // decompose A using SVD
  SVD svd(A_in);

  if (!nullSpace && (svd.condition() >= maxCondition)) {
    if (stopOnIllCondition)
      throw solution_error(Exception("FSP unable to complete solution: A is ill-conditioned (has large SVD condition number="
                                            +base::realToString(svd.condition())+")"));
    else
      Logln("Warning: A is ill-conditioned; singular values are:" << svd.diag() << " condition=" << svd.condition());
  }

  const Matrix& U( svd.U() );
  const Matrix  S( svd.S() );
  const Matrix& V( svd.V() );
  Matrix Uinv( transpose(U) );


  // invert S
  Matrix Sinv( zeroMatrix(M,N) );
  for(Int i=0; i< Math::minimum(M, N); i++) {
    if (Math::abs(S(i,i)) > small)
      Sinv(i,i) = 1/S(i,i);
    else {
      if (stopOnIllCondition)
        throw solution_error(Exception(String("FSP unable to complete solution: a singular value of A is near 0 ")
                                        +"(S("+base::intToString(i)+","+base::intToString(i)+")="+base::realToString(S(i,i))+")"));
      else {
//        Logln("Warning: small singular value, setting 1/S(" << base::intToString(i) << "," << base::intToString(i) << ") to 0");
        Sinv(i,i) = 0;
      }
    }
  }


  Matrix g;

  // special case is b=0 (null-space)
  if (nullSpace) {

    // just extract the null-space vectors from V (the last M-N vectors)
    g.resize(M, Math::maximum(SInt(M-N),0) );
    if (g.cols() > 0)
      for(Int i=0; i<M-N; i++)
        matrixColumn(g,i) = matrixColumn(V,N+i);

  }
  else {

    g.resize(M, Math::maximum(SInt(M-N+1),0) );

    Vector Utb(Uinv * b);

    if (g.cols() > 0) {
      matrixColumn(g,0) = V * Sinv * Utb; // particular solution

      // now add homogeneous solution
      for(Int i=0; i<M-N; i++)
        matrixColumn(g,i+1)=matrixColumn(V,N+i) + matrixColumn(g,0);
    }
  }

  // return the indices of rows that were eliminated due to dependency (none for this method)
  dependentRowsEliminated.clear();

  return g;
}









