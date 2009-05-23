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
  
  $Id: MPPseudoInvSolver.cpp 1036 2004-02-11 20:48:55Z jungd $
  $Revision: 1.8 $
  $Date: 2004-02-11 15:48:55 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <robot/control/kinematics/MPPseudoInvSolver>


using robot::control::kinematics::MPPseudoInvSolver;



base::Vector MPPseudoInvSolver::solve(const Matrix& A, const Vector& b)
{
  Assert(b.size() == A.size1());

  Matrix siA( Math::pseudoInverse(A) );
  
  return siA*b; // A^-1.b
}
