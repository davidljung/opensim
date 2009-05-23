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

  $Id: AnalyticLagrangianFSBetaOptimizer.cpp 1036 2004-02-11 20:48:55Z jungd $
  $Revision: 1.11 $
  $Date: 2004-02-11 15:48:55 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $

****************************************************************************/

#include <robot/control/kinematics/AnalyticLagrangianFSBetaOptimizer>
#include <robot/control/kinematics/ReferenceOpVectorFormObjective>
#include <robot/control/kinematics/solution_error>

using base::transpose;
using robot::control::kinematics::AnalyticLagrangianFSBetaOptimizer;

using robot::control::kinematics::ReferenceOpVectorFormObjective;
using robot::control::kinematics::BetaFormConstraints;
using robot::control::kinematics::solution_error;



base::Vector AnalyticLagrangianFSBetaOptimizer::optimize(ref<const Objective> objective,
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

  Vector t( calct(Ginv, H, betaConstraints, nullSpace) );
  Assert(t.size() == span);

  // check the tis sum appropriately
  Real s=0;
  for(Int i=0; i<t.size(); i++) s+=t[i];
  if (!Math::equals(s,nullSpace?0:1,0.00001))
    throw solution_error(Exception("failed to find a solution meeting the criteria and constraints (sum of tis=" 
                             + base::realToString(s) +"), but should be "+base::realToString(nullSpace?0:1)));

  // finally, we can calculate dq
  Vector dq( gs.size1() );
  dq = zeroVector( gs.size1() );
  for(Int i=0; i<span; i++)
    dq += t[i]*matrixColumn(gs,i);

  return dq;
}



base::Vector AnalyticLagrangianFSBetaOptimizer::calct(const Matrix& Ginv, const Vector& H, ref<const BetaFormConstraints> betaConstraints, bool nullSpace) const
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
    
    // computations common to both cases
    
    // a = eT.Ginv.e  - i.e. the sum of the elements of Ginv
    Real a = dot( e, Ginv*e );
    Vector GinvH( Ginv * H );
    
    
    if (!betaConstraints->isAlphaConstraint()) { // case 1 - no betas &   no alpha
      
      if (Math::equals(a,0))
        throw solution_error(Exception("optimization failed: a=0"));
      
      Real etGinvH = dot(e, GinvH);
      
      Real b = -( (nullSpace?0:1) + etGinvH)/a; // (NB: this isn't the b from paper - it's analogous to mu)
      
      t = -Ginv*(b*e + H);
      
    }
    else { // case 3 - no betas but an alpha

      Vector alpha( betaConstraints->getAlpha() );
      if (equals(alpha,e))
        throw solution_error(Exception("optimization failed: alpha=e=(1,1,..,1) - undeterminable"));

      // f = eT.Ginv.H
      Real f = dot(e, GinvH); 
      
      // f term: ft = f if null-space solution required, 1+f otherwise
      Real ft = nullSpace?0:1 + f;
      
      // k = alphaT.Ginv.H                          eqn. (2-35)
      Real k = dot(alpha, GinvH);
      
      // l = eT.Ginv.alpha                          eqn. (2-36)
      Vector etGinv( e * Ginv );
      Real l = dot(etGinv, alpha);
      
      // s = alphaT.Ginv.alpha                      eqn. (2-37)
      Real s = dot(alpha, Ginv*alpha);

      // now calculate eta, the multipliers mu, nu and gi coeffs t
      Real l2msa = Math::sqr(l) - s*a;
      Real eta = (a*k - l*ft) / l2msa;       // eqn. (2-33) without terms involving betas
      Real mu = -(ft + eta*l) / a;           // eqn. (2-32) without terms involving betas

      t = -Ginv*(H + mu*e + eta*alpha);      // eqn. (2-31) without terms involving betas
      
      // check that NH constraint holds    
      Assert(alpha.size()==t.size());  
      Real sum=0;
      for(Int i=0; i<alpha.size(); i++)
        sum+= alpha[i]*t[i];
      if (!Math::equals(sum,0,1e-9)) {
        throw solution_error(Exception("Solution generated violates NonHolonomic constraint (Sum alphai.ti != 0)"));
      }

    }


  }
  else { // cases 2 & 4 - with betas

    // computations common to both cases
    
    // compute the intermediates a,b,d and A from the papers
    //  so that we can evaluate the Largange multipliers mu & nu and
    //  finally t 
    
    Matrix betas(betaConstraints->getBetas()); // each beta is a column 
    Int r = betas.size2(); // number of constraints
    
    // a = eT.Ginv.e  - i.e. the sum of the elements of Ginv
    Real a = inner_prod( e, Ginv*e );
    
    if (Math::equals(a,0))
      throw solution_error(Exception("optimization failed: a=0"));
    
    // b, bi = eT.Ginv.betai
    Vector b(r);
    Vector etGinv( e * Ginv );
    for(Int i=0; i<r; i++) 
      b[i] = inner_prod( etGinv, matrixColumn(betas,i) );
    // NB: paper 1 also has c, ci = betaiT.Ginv.e, which is actually = bi, hence redundant
    
    // d, di = 1 + betaiT.Ginv.H
    Vector d(r);
    Vector GinvH( Ginv * H );
    for(Int i=0; i<r; i++) 
      d[i] = 1.0 + inner_prod( matrixColumn(betas, i), GinvH );
    
    // f = eT.Ginv.H
    Real f = inner_prod(e, GinvH); 
    
    // f term: ft = f if null-space solution required, 1+f otherwise
    Real ft = nullSpace?0:1 + f;


    if (!betaConstraints->isAlphaConstraint()) { // case 2 - betas but no alpha
      
      // A, Aij = bi.bj - a.betaiT.Ginv.betaj    (paper 1)
      Matrix A(r,r);
      for(Int j=0; j<r; j++) {
        Vector Ginvbetaj( Ginv * Vector(matrixColumn(betas,j)) );
        for(Int i=0; i<r; i++) 
          A(i,j) = b[i]*b[j] - a*inner_prod( matrixColumn(betas,i), Ginvbetaj );
      }
      Matrix Ainv( Math::inverse(A) );
      
      
      // now calculate the multipliers mu, nu and gi coeffs t
      Vector nu( Ainv*(a*d - b*ft) );     // eqn. (1-15/18 : 2-13)
      Real mu = -( inner_prod(nu,b) + ft)/a; // eqn. (1-16/19 : 2-14)
      
      Vector Bnu( betas*nu );
      t = -Ginv*(mu*e + Bnu + H); // eqn. (1-17 : 2-15)
      
    }
    else { // case 4 - both betas and an alpha
      
      Vector alpha( betaConstraints->getAlpha() );
      if (equals(alpha,e))
        throw solution_error(Exception("optimization failed: alpha=e=(1,1,..,1) - undeterminable"));
      
      // k = alphaT.Ginv.H                          eqn. (2-35)
      Real k = inner_prod(alpha, GinvH);
      
      // l = eT.Ginv.alpha                          eqn. (2-36)
      Real l = inner_prod(etGinv, alpha);
      
      // s = alphaT.Ginv.alpha                      eqn. (2-37)
      Real s = inner_prod(alpha, Ginv*alpha);
      
      // c, ci = betaiT.Ginv.alpha                  eqn. (2-38)
      Vector c(r);
      Vector Ginvalpha( Ginv*alpha );
      for(Int i=0; i<r; i++)
        c[i] = inner_prod( matrixColumn(betas,i), Ginvalpha );
      
      Real l2msa = Math::sqr(l) - s*a;
      
      if (Math::equals(l2msa,0))
        throw solution_error(Exception("optimization failed: l^2-s.a=0"));
      
      // A, Aij = bi.(s.bj - l.cj) + ci.(a.cj - l.bj) + (l^2 - s.a).betaiT.Ginv.betaj   eqn. (2-39)
      Matrix A(r,r);
      for(Int j=0; j<r; j++) {
        Real sbjmlcj = s*b[j] - l*c[j];
        Real acjmlbj = a*c[j] - l*b[j];
        Vector Ginvbetaj( Ginv * Vector(matrixColumn(betas,j)) );
        for(Int i=0; i<r; i++) 
          A(i,j) = b[i]*sbjmlcj + c[i]*acjmlbj + l2msa*inner_prod( matrixColumn(betas,i), Ginvbetaj );
      }
      
      Matrix Ainv( Math::inverse(A) );
      
      // now calculate eta, the multipliers mu, nu and gi coeffs t
      Vector nu( Ainv*((l*k - s*ft)*b - (a*k - l*ft)*c -  l2msa*d) ); // eqn. (2-34)
      Real eta = (a*k - l*ft - inner_prod(nu,l*b-a*c)) / l2msa;       // eqn. (2-33)
      Real mu = -(ft + inner_prod(nu,b) + eta*l) / a;                 // eqn. (2-32)
      
      
      Vector Bnu( betas*nu );
      t = -Ginv*(H + mu*e + Bnu + eta*alpha);   // eqn. (2-31)

    }
    

  }
  
  return t;
  
}


