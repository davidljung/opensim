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

  $Id: AnalyticLagrangianNullSpaceBetaOptimizer.cpp 1036 2004-02-11 20:48:55Z jungd $
  $Revision: 1.1 $
  $Date: 2004-02-11 15:48:55 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $

****************************************************************************/

#include <robot/control/kinematics/AnalyticLagrangianNullSpaceBetaOptimizer>
#include <robot/control/kinematics/ReferenceOpVectorFormObjective>
#include <robot/control/kinematics/solution_error>

using base::transpose;
using robot::control::kinematics::AnalyticLagrangianNullSpaceBetaOptimizer;

using robot::control::kinematics::ReferenceOpVectorFormObjective;
using robot::control::kinematics::BetaFormConstraints;
using robot::control::kinematics::solution_error;



base::Vector AnalyticLagrangianNullSpaceBetaOptimizer::optimize(ref<const Objective> objective,
				                                ref<const Constraints> constraints) const
{
  if (!instanceof(*objective, const ReferenceOpVectorFormObjective))
    throw std::invalid_argument(Exception("objective is not in the required form (ReferenceOpVectorFormObjective)"));
  ref<const ReferenceOpVectorFormObjective> rovfObjective(narrow_ref<const ReferenceOpVectorFormObjective>(objective));

  if (!instanceof(*constraints, const BetaFormConstraints))
    throw std::invalid_argument(Exception("constraints are not of the required form (BetaFormConstraints)"));
  ref<const BetaFormConstraints> betaConstraints(narrow_ref<const BetaFormConstraints>(constraints));

  if (!gs)
    throw std::runtime_error(Exception("no Gs supplied via setGs()"));
  const Matrix& gs(*(this->gs));

  const Int span = gs.size2();


  // first calculate the Matrix G and the Vector H (from paper)
  Matrix B( rovfObjective->getB() );
  Vector dZr( rovfObjective->getdZr() );

  Matrix G(span,span);
  Matrix BtB( transpose(B) * B ); 
  for(Int j=0; j<span; j++) {
    Vector gj( matrixColumn(gs,j) );
    Vector BtBgj( BtB*gj );
    for(Int i=0; i<span; i++) {
      Vector gi( matrixColumn(gs,i) );
      G(i,j) = dot( gi, BtBgj ); // i.e. Gij = gi.Bt.B.gj ; eqn. (1-12)
    }
  }

  Matrix Ginv;
  try { 
    Ginv = Math::inverse(G); 
  } catch (std::exception& e) {
    throw solution_error(Exception("unable to invert Grammian G:"+String(e.what())));
  }
  

  Vector H(span);
  if (equals(dZr,zeroVector(dZr.size()))) {
    H = zeroVector(span); // if dZr=0, so does H
  }
  else {
    Vector dZrtB( dZr*B );
    for(Int k=0; k<span; k++)
      H[k] = dot( dZrtB, matrixColumn(gs,k) ); // eqn. (1-11)
  }

  
  // compute t so that we can get dq.

  // convenience
  o.reset(zeroVector(span));
  e.resize(span);
  for(Int i=0; i<span; i++) e[i]=1;

  Vector t( calct(Ginv, H, betaConstraints) );
  Assert(t.size() == span);

  // finally, we can calculate dq
  Vector dq( gs.size1() );
  dq = zeroVector( gs.size1() );
  for(Int i=0; i<span; i++)
    dq += t[i]*matrixColumn(gs,i);

  return dq;
}



base::Vector AnalyticLagrangianNullSpaceBetaOptimizer::calct(const Matrix& Ginv, const Vector& H, ref<const BetaFormConstraints> betaConstraints) const
{
  //
  // 4 cases:
  //  1)   no betas &   no alpha (alpha is the non-holonomic constraint) 
  //  2)      betas but no alpha
  //  3)   no betas but an alpha
  //  4) both betas and an alpha
  //

  const Int span = Ginv.size1();
  Vector t(span);  // solution coeffs

  
  if (betaConstraints->numBetaConstraints() == 0) { // cases 1&3 - no betas
    
    // trivial.  Since we're after a null-space solution and there are no
    //  contraints to drive the system, the solution is 0
    t = zeroVector(span);      
      
  }
  else { // cases 2 & 4 - with betas

    // computations common to both cases
    
    // compute the intermediates a,b,d and A from the papers
    //  so that we can evaluate the Largange multipliers mu & nu and
    //  finally t 
    
    Matrix betas(betaConstraints->getBetas()); // each beta is a column 
    Int r = betas.size2(); // number of constraints
    
    // d, di = -(1 + betaiT.Ginv.H)
    Vector d(r);
    Vector GinvH( Ginv * H );
    for(Int i=0; i<r; i++) 
      d[i] = -(1.0 + inner_prod( matrixColumn(betas, i), GinvH ));
    

    if (!betaConstraints->isAlphaConstraint()) { // case 2 - betas but no alpha
      
      // A, Aij = betaiT.Ginv.betaj    
      Matrix A(r,r);
      for(Int j=0; j<r; j++) {
        Vector Ginvbetaj( Ginv * Vector(matrixColumn(betas,j)) );
        for(Int i=0; i<r; i++) 
          A(i,j) = inner_prod( matrixColumn(betas,i), Ginvbetaj );
      }
      Matrix Ainv( Math::inverse(A) );
      
      
      // now calculate the multipliers nu and gi coeffs t
      Vector nu( Ainv*d );     // A*nu = d
      
      Vector Bnu( betas*nu );
      t = -Ginv*(Bnu + H); 
      
    }
    else { // case 4 - both betas and an alpha
      
      throw solution_error(Exception("Null-space solution with non-holonomic constraint not implemented."));

    }
    

  }
  
  return t;
  
}


