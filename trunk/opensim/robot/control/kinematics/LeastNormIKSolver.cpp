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

  $Id: LeastNormIKSolver.cpp 1129 2004-09-28 20:47:28Z jungd $
  $Revision: 1.7 $
  $Date: 2004-09-28 16:47:28 -0400 (Tue, 28 Sep 2004) $
  $Author: jungd $

****************************************************************************/

#include <robot/control/kinematics/LeastNormIKSolver>

using robot::control::kinematics::LeastNormIKSolver;

using base::Vector;
using base::Orient;
using robot::control::kinematics::MPPseudoInvSolver;


LeastNormIKSolver::LeastNormIKSolver()
{
  solver = ref<MPPseudoInvSolver>(NewObj MPPseudoInvSolver());
}


const Real small = 5.0e-05;


Vector LeastNormIKSolver::solve(const Vector& dx, Real dt, const Vector& x, const Vector& q,
                                const base::Matrix& J,
                                OptimizationMethod    optMethod,
                                OptimizationCriterion optCriterion,
                                OptimizationConstraints optConstraints)
{
  if (optMethod==DefaultMethod) optMethod=PseudoInv;
  if (optCriterion==DefaultCriterion) optCriterion=LeastNorm;
  if (optConstraints.test(DefaultConstraints)) optConstraints.reset();

  Assertm(optCriterion==LeastNorm, "criterion is leastnorm (no optimization involved for this method)");
  Assertm(optMethod==PseudoInv, "method is pseudo-inverse (no other method supported)");
  Assertm(optConstraints.none(), "no constraints (none supported)");

  const Int N = J.size1(); // rows
  const Int M = J.size2(); // cols

  Assert(dx.size() == x.size());
  Assert(q.size() == M);

  // special case - if dx==0 (null-space), only trivial solution (dq=0) is possible
  if (base::equals(dx,zeroVector(N),small))
    return zeroVector(M);

  // solve dq = J(q)dx, for least-norm dq via M-P pseudo-inverse
  return solver->solve(J, dx);
}
