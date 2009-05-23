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

  $Id: KinematicsTest.cpp 1126 2004-09-28 20:45:32Z jungd $
  $Revision: 1.6 $
  $Date: 2004-09-28 16:45:32 -0400 (Tue, 28 Sep 2004) $
  $Author: jungd $

****************************************************************************/

#include <robot/control/kinematics/KinematicsTest>

using robot::control::kinematics::KinematicsTest;


using robot::TestRobot;

using base::equals;



KinematicsTest::KinematicsTest()
{
}


void KinematicsTest::setUp()
{
  jt.resize(dof);
  alpha.resize(dof);
  a.resize(dof);
  d.resize(dof);
  theta.resize(dof);

  for(Int j=0; j<dof; j++) {
    jt[j] = TestRobot::Revolute;
    alpha[j] = Math::degToRad(20);
    a[j] = 0.2;
    d[j] = 0;
    theta[j] = 0;
  }

  jt2.resize(dof2);
  alpha2.resize(dof2);
  a2.resize(dof2);
  d2.resize(dof2);
  theta2.resize(dof2);

  for(Int j=0; j<dof2; j++) {
    jt2[j] = TestRobot::Revolute;
    alpha2[j] = 0;
    a2[j] = 0.4;
    d2[j] = 0;
    theta2[j] = 0;
  }
  d2[1] = 0.5;



  robot = ref<TestRobot>(NewObj TestRobot(jt, alpha, a, d, theta));
  robot2 = ref<TestRobot>(NewObj TestRobot(jt2, alpha2, a2, d2, theta2));

  chain  =  robot->getRobotDescription()->manipulators()[0]->getKinematicChain();
  chain2 = robot2->getRobotDescription()->manipulators()[0]->getKinematicChain();

  lnsolver = ref<LeastNormIKSolver>(NewObj LeastNormIKSolver());


}


void KinematicsTest::tearDown()
{
}


void KinematicsTest::testLeastNormCriteria()
{

  Vector q(dof);
  for(Int j=0; j<dof; j++)
    q[j] = Math::degToRad(15);

  Vector x(3);
  x = zeroVector(3);

  Vector dx(3);
  dx[0] = -0.01; dx[1] = 0; dx[2] = 0.01;



  // solve via Larganrian, least-norm, no constraints
  ikor = ref<IKOR>(NewObj IKOR(chain));

  Matrix J( chain.getJacobian(q, false) );
  Vector dq ( ikor->solve(dx, 1, x, q, J,
                          IKOR::Lagrangian, IKOR::LeastNorm, 0) );
  CPPUNIT_ASSERT( dq.size() == dof );

  Vector cdq(6); // the correct dqs (calculated using the original IKORv2.0 code)
  cdq[0] = -0.0267338; cdq[1] = 0.0209276; cdq[2] = 0.025021;
  cdq[3] = 0.00535535; cdq[4] = -0.0153997; cdq[5] = -0.0194938;

  CPPUNIT_ASSERT( equals(dq,cdq,0.0000001) );

  // compare the solution obtained with the Pseudo-inverse
  Vector dq2 ( lnsolver->solve(dx, 1, x, q, J,
                               LeastNormIKSolver::DefaultMethod, LeastNormIKSolver::LeastNorm, 0) );

  CPPUNIT_ASSERT( equals(dq2, dq, 0.0000000001) );


}



void KinematicsTest::testImpossibleMotion()
{
  // Try an impossible motion (ask for ee to move back in x while
  //  all joints at 0)

  Vector q(dof2);
  q = zeroVector(dof2);
  Vector x(3);
  x[0] = 2.0; x[1] = 0; x[2]=0.5;

  Matrix f2( chain2.getForwardKinematics(q) ); // check forward kinematics against x
  CPPUNIT_ASSERT( equals(Vector(vectorRange(Vector(matrixColumn(f2,3)), Range(0,3))),x) );

  Vector dx(3);
  dx[0] = -0.01; dx[1] = 0; dx[2] = 0.0;

  // solve via FSP, Larganrian, least-norm, no constraints
  ikor = ref<IKOR>(NewObj IKOR(chain2));
  Matrix J2(chain2.getJacobian(q,false));
  Vector dq ( ikor->solve(dx, 1, x, q, J2,
                          IKOR::Lagrangian, IKOR::LeastNorm, 0) ); // should throw
}



void KinematicsTest::testNoMotion()
{
//!!!!
return;

  // Ask for dx=0 and verify the solution is dq=0
  Vector q(dof);
  q = zeroVector(dof);

  Vector x(3);

  Matrix f( chain.getForwardKinematics(q) ); // calculate x from forward kinematics
  x = vectorRange(Vector(matrixColumn(f,3)), Range(0,3));

  Vector dx(3); dx = zeroVector(3);

  ikor = ref<IKOR>(NewObj IKOR(chain));
  Matrix J(chain.getJacobian(q,false));
  Vector dq ( ikor->solve(dx, 1, x, q, J,
                          IKOR::Lagrangian, IKOR::LeastNorm, 0) );

  CPPUNIT_ASSERT( equals(dq, zeroVector(dof), 0.0000000001) );
}







#ifdef DEBUG
CPPUNIT_TEST_SUITE_REGISTRATION( KinematicsTest );
#endif

