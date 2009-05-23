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

  $Id: IKOR.cpp 1128 2004-09-28 20:46:58Z jungd $
  $Revision: 1.17 $
  $Date: 2004-09-28 16:46:58 -0400 (Tue, 28 Sep 2004) $
  $Author: jungd $

****************************************************************************/

#include <robot/control/kinematics/IKOR>

#include <robot/control/kinematics/solution_error>
#include <robot/control/kinematics/SVDFullSpaceSolver>
#include <robot/control/kinematics/Optimizer>
#include <robot/control/kinematics/ReferenceOpVectorFormObjective>
#include <robot/control/kinematics/AnalyticLagrangianFSBetaOptimizer>
#include <robot/control/kinematics/AnalyticLagrangianNullSpaceBetaOptimizer>


using robot::control::kinematics::IKOR;

using base::Vector;
using base::Orient;
using base::dot;
using robot::control::kinematics::SVDFullSpaceSolver;
using robot::control::kinematics::ReferenceOpVectorFormObjective;
using robot::control::kinematics::BetaFormConstraints;
using robot::control::kinematics::AnalyticLagrangianFSBetaOptimizer;
using robot::control::kinematics::AnalyticLagrangianNullSpaceBetaOptimizer;


const Real small = 5.0e-05;


IKOR::IKOR(const robot::KinematicChain& chain, const Vector& jointWeights,
           bool nonHolonomicPlatformActive, Real platformL)
  : chain(chain), weights(jointWeights),
    nonHolonomicPlatformActive(nonHolonomicPlatformActive), L(platformL)
{
  if (weights.size() == 0) {
    weights.reset(Vector(chain.dof()));
    for(Int i=0; i<weights.size(); i++) weights[i]=1.0;
  }

  solver = ref<SVDFullSpaceSolver>(NewObj SVDFullSpaceSolver());
}



bool IKOR::isConstraintTypeSupported(OptimizationConstraint  optConstraint,
                                         OptimizationMethod      optMethod,
                                         OptimizationCriterion   optCriterion)
{
  if (optMethod==DefaultMethod) optMethod=Lagrangian;
  if (optCriterion==DefaultCriterion) optCriterion=LeastNorm;

  if (optMethod == Lagrangian) {

    if (optCriterion == LeastNorm) {

      return (   (optConstraint == JointLimits)
              || (optConstraint == ObstacleAvoidance) );

    }

  }

  return false;
}


Vector IKOR::solve(const Vector& dx, Real dt, const Vector& x, const Vector& q,
                   const base::Matrix& J,
                   OptimizationMethod    optMethod,
                   OptimizationCriterion optCriterion,
                   OptimizationConstraints optConstraints)
{
  if (optMethod==DefaultMethod) optMethod=Lagrangian;
  if (optCriterion==DefaultCriterion) optCriterion=LeastNorm;
  if (optConstraints.test(DefaultConstraints)) optConstraints.reset();

  const Int N = J.size1(); // rows
  const Int M = J.size2(); // cols

  Assert(q.size() == M);
  Assert(dx.size() == N);

  bool nullSpaceMotion = base::equals(dx,zeroVector(N),small);

  // First use the FullSpaceSolver to obtain the g Vectors
  array<Int> dependentRowsEliminated;
  const Vector& b(dx);
  Matrix gs;
  try {
    gs = solver->solve(J, b, dependentRowsEliminated); // gis are the columns Vectors of gs
  } catch (std::exception& e) {
    throw solution_error(Exception(String("Unable to generate solution space for the given dx in the current state (impossible motion?) - FSP failed:")+e.what()));
  }

  // Which concrete Optimizer and corresponding Objective and Constraints
  //  get instantiated, depends on the opt<X> parameters
  ref<Optimizer>              optimizer;
  ref<Optimizer::Objective>   objective;
  ref<Optimizer::Constraints> constraints;

  ref<LagrangianOptimizer> lagOptimizer;


  //
  // First we run the optimization with only constraints
  //  due to dependent rows (& the platform non-holonomic constraint if appropriate)
  // Next, we test for violation of any other constrant types
  //  such as joint limit or obstacle constraints.
  //  If any constraints were violated we add them in and run the optimization
  //  again.  This is repeated until all constraints are satisfied.


  if (optMethod==Lagrangian) {

    // First setup the objective function
    //  - the objective is defined in terms of the Matrix B and Vector dZr
    Matrix B(M,M);
    Vector dZr(M);

    switch (optCriterion) {
    case LeastNorm: weightedLeastNorm(B,dZr, weights); break;
    case LeastFlow: throw std::runtime_error(Exception("LeastFlow criteria not implemented for Larganrian method."));
    default:
      throw std::invalid_argument(Exception("unsupported/unknown criteria for Largangian Method"));
    }

    ref<ReferenceOpVectorFormObjective> rovfObjective(NewObj ReferenceOpVectorFormObjective());
    objective = rovfObjective;
    rovfObjective->setB(B);
    rovfObjective->setdZr(dZr);


    // setup constraints for first optimization step
    ref<BetaFormConstraints> betaconstraints(NewObj BetaFormConstraints());
    constraints = betaconstraints;
    betaconstraints->clear();

    if (nonHolonomicPlatformActive)
      addNonholonomicConstraint(gs, q, betaconstraints);


    if (dependentRowsEliminated.size() > 0)
      addDependentRowConstraints(dependentRowsEliminated, gs, J, b, betaconstraints);


    // create the Optimizer
    if (!nullSpaceMotion)
      lagOptimizer = ref<AnalyticLagrangianFSBetaOptimizer>(NewObj AnalyticLagrangianFSBetaOptimizer());
    else
      lagOptimizer = ref<AnalyticLagrangianNullSpaceBetaOptimizer>(NewObj AnalyticLagrangianNullSpaceBetaOptimizer());
    lagOptimizer->setGs(gs);
    optimizer = lagOptimizer;

  }
  else
    throw std::invalid_argument(Exception("unimplemented optimization method"));


  // Now use the optimizer, objective and constraints to narrow the solution
  //  space down to a single dq Vector
  Vector dq(M);

  try {
//Debugln(DJ,"++++++++++++ optimizing1");
    dq = optimizer->optimize(objective, constraints);

  } catch (solution_error& e) {
    dq = zeroVector(M);
    throw solution_error(Exception(String("Unable to solve for the given dx in the current state - optimization failed:")+e.what()),e);
  }

//!!! this loop will probably cause some constraints to be added in more than once - FIX

  // Now loop adding extra constraints for any constraint types that are violated,
  //  until all are satisfied.
  Int numConstraints;
  do {
    numConstraints = constraints->numConstraints(); // initial no. of constraints



    // check for other constraint violations and add them
    if (optMethod == Lagrangian) {

      ref<BetaFormConstraints> betaconstraints(narrow_ref<BetaFormConstraints>(constraints));

      // add constraint that are always active
      betaconstraints->clear();
      if (nonHolonomicPlatformActive)
        addNonholonomicConstraint(gs, q, betaconstraints);
      if (dependentRowsEliminated.size() > 0)
        addDependentRowConstraints(dependentRowsEliminated, gs, J, b, betaconstraints);

      // add constraints for any other constraint types that
      // are violated
      if (optConstraints.test(JointLimits)) {
        addJointLimitConstraints(gs, q, dq, betaconstraints);
      }

      if (optConstraints.test(ObstacleAvoidance))
        addObstacleAvoidanceConstraints(gs, q, dq, betaconstraints);

    }
    else
      throw std::invalid_argument(Exception("unimplemented optimization method"));


    if ( constraints->numConstraints() > numConstraints) {

      // more were added, run the optimizer again
      try {
	//Debugcln(DJ,"optimizing2:");
	//Debugcln(DJ,"constraints:\n" << *constraints);
	//Debugcln(DJ," extra constraints:" << (constraints->numConstraints()-numConstraints));
        dq = optimizer->optimize(objective, constraints);

      } catch (solution_error& e) {
        dq = zeroVector(M);
        throw solution_error(Exception(String("Unable to solve for the given dx in the current state - optimization failed:")+e.what()),e);
      }

    }


  } while ( constraints->numConstraints() > numConstraints); // keep repeating as long as more were added
//Debugcln(DJ,"opt done dq=" << dq);

  return dq;
}


void IKOR::setProximitySensorData(const array<LinkProximityData>& proximityData, Real d)
{
  proximitySensorData = proximityData;
  this->d = d;
}



void IKOR::setParameter(const String& name, Real value)
{
  throw std::invalid_argument(Exception("unknown parameter name"));
}



void IKOR::weightedLeastNorm(Matrix& B, Vector& dZr, Vector weights)
{
  // for least norm criteria, B is the identity & dZr=0
  dZr = zeroVector(dZr.size());

  // set B to identity
  for(Int j=0; j<B.size2(); j++)
    for(Int i=0; i<B.size1(); i++)
      B(i,j) = (i!=j)?0:weights[i];
}


IKOR::RankLossBetaConstraint::RankLossBetaConstraint(Int row, const Matrix& A, const Vector& b, const Matrix& gs)
  : row(row)
{
  setName(className());

  const Int span=gs.size2();
  beta.resize(span);
  Vector Ar(matrixRow(A,row));

  for(Int i=0; i<span; i++)
    beta[i] = dot(Ar, matrixRow(gs,i) ) / b[row];

}


void IKOR::addDependentRowConstraints(const array<Int>& rows, const Matrix& gs, const Matrix& A, const Vector& b, ref<BetaFormConstraints> constraints)
{
  // add one beta per row
  for(Int i=0; i<rows.size(); i++)
    constraints->addConstraint(ref<BetaFormConstraints::BetaFormConstraint>(NewObj RankLossBetaConstraint(rows[i], A, b, gs)) );
}




void IKOR::addJointLimitConstraints(const Matrix& gs, const Vector& q, const Vector& dq, ref<BetaFormConstraints> constraints)
{
  // iterate through the joints and check if they will be moved out of limits by dq.
  //  If so, create a constraint
  const Int M = gs.size1();
  Vector tq(q+dq); // target q

  for(Int v=0; v<M; v++) { // for each variable/component of q

    // check if there are limits
    Real minValue = chain.variableMinLimit(v);
    Real maxValue = chain.variableMaxLimit(v);
    bool hasMinLimit = true;
    bool hasMaxLimit = true;
    if (chain.variableUnitType(v)==KinematicChain::Angle) { // angle
      if (  (Math::equals(minValue, KinematicChain::unboundedMinAngleLimit))
          ||(Math::equals(maxValue, KinematicChain::unboundedMaxAngleLimit)) ) {
        hasMinLimit = hasMaxLimit = false;
      }
    }
    else { // distance
      if (Math::equals(minValue, KinematicChain::unboundedMinDistLimit,999))
        hasMinLimit = false;
      if (Math::equals(maxValue, KinematicChain::unboundedMaxDistLimit,999))
        hasMaxLimit = false;
    }


    if (hasMinLimit && (tq[v] < minValue)) {
      Debugcln(IKOR,"v:" << v << " m" << Math::radToDeg(minValue) << " > " << Math::radToDeg(tq[v]) );
      constraints->addConstraint(
         ref<BetaFormConstraints::BetaFormConstraint>(NewObj JointLimitBetaConstraint(v,minValue-tq[v],gs))
       );
    }

    if (hasMaxLimit && (tq[v] > maxValue)) {
      Debugcln(IKOR,"v:" << v << " m" << Math::radToDeg(maxValue) << " < " << Math::radToDeg(tq[v]) );
      constraints->addConstraint(
        ref<BetaFormConstraints::BetaFormConstraint>(NewObj JointLimitBetaConstraint(v,maxValue-tq[v],gs))
      );
    }

  }
  //Debugcln(IKOR,"");
}



void IKOR::addNonholonomicConstraint(const Matrix& gs, const Vector& q, ref<BetaFormConstraints> constraints)
{
  // assume the platform parameters are the first three components of q, namely xp, yp, and thetap
  //  (i.e. 2D platform) - (in the paper they are the last three)

  Vector alpha(gs.size2());

  Real sqm = Math::sin(q[2]);
  Real cqm = Math::cos(q[2]);
  for(Int i=0; i<gs.size2(); i++) {
    Vector gi( matrixColumn(gs,i) );
    alpha[i] = -sqm*gi[0] + cqm*gi[1] - L*gi[2];
  }
//Debugln(DJ,"alpha=" << alpha);
  constraints->setAlphaConstraint(alpha);
}




IKOR::PushAwayBetaConstraint::PushAwayBetaConstraint(Int v, Real Xvx,
                                                     const Vector& n, Real L,
                                                     const Matrix& gs,
                                                     const robot::KinematicChain& chain, const Vector& q)
  : v(v), Xvx(Xvx), n(n), L(L)
{
  setName(className());

  // see "Resolving Kinematic Redundancy with Constraints Using the FSP (Full Space Parameterization) Approach,
  //   Pin, Tulloch et.al. for a description of the obstacle constraint formulation

  Unimplemented; 
  //!!! removed until published - sorry.

}




void IKOR::addObstacleAvoidanceConstraints(const Matrix& gs, const Vector& q, const Vector& dq, ref<BetaFormConstraints> constraints)
{
#ifdef DEBUG
  return;//!!! disabling obstacle constraints (see comment above)
#endif
  if (proximitySensorData.size() == 0) return; // no proximity sensor data provided

  // if any data provided, must be data for each dof
  Assert( proximitySensorData.size() == chain.dof() );

  // check for obstacle proximity to each 'link'
  for(Int v=0; v<chain.dof(); v++) { // for each variable (i.e. element of q)

    LinkProximityData& pd( proximitySensorData[v] );

    if (pd.distance < d) {
      if (!pd.direction.equals( zeroVector(3) )) { // ignore if direction is unknown
        // something is within the proximity sensor range of this 'link' & within the 'danger' envelope
        Real L = d - pd.distance + 1e-4; // desired 'push-away' distance
        Vector n( -pd.direction ); // normal - direction to push point Xj - point closest to obstacle

        // Before adding the constraint, we need to check for a special case where the end of the link
        //  is the closest to the object.  In this case, the end of the previous link (which is
        //  conincident) may have already created an identical constraint (if it is revolute, the
        //  closest point will be at the joint).  In this case, we don't need a second identical constraint.
        if ( Math::equals(pd.intercept,0) && (chain.linkIndexOfVariable(v) > 0) ) {
          if ( chain[chain.linkIndexOfVariable(v)].type() == KinematicChain::Link::Revolute )
            return; // skip this constraint
        }

	//Debugln(DJ,"proximity: variable " << v << " (l=" << chain.linkIndexOfVariable(v) << "):\nL=" << L << " n=" << n << " lXj=" << pd.intercept << " d=" << d << " dist=" << pd.distance);
        constraints->addConstraint(ref<BetaFormConstraints::BetaFormConstraint>(NewObj PushAwayBetaConstraint(v,pd.intercept, n, L, gs, chain, q)) );
      }

    }

  }

}


