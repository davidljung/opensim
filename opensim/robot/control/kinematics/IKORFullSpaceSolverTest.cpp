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

  $Id: IKORFullSpaceSolverTest.cpp 1079 2004-07-28 19:35:26Z jungd $
  $Revision: 1.4 $
  $Date: 2004-07-28 15:35:26 -0400 (Wed, 28 Jul 2004) $
  $Author: jungd $

****************************************************************************/

#include <robot/control/kinematics/IKORFullSpaceSolverTest>

using robot::control::kinematics::IKORFullSpaceSolverTest;



using base::equals;
using robot::control::kinematics::IKORFullSpaceSolver;


IKORFullSpaceSolverTest::IKORFullSpaceSolverTest()
{
}


void IKORFullSpaceSolverTest::setUp()
{

  fsp = ref<IKORFullSpaceSolver>(NewObj IKORFullSpaceSolver());
}


void IKORFullSpaceSolverTest::tearDown()
{
}


void IKORFullSpaceSolverTest::testByHandCase1()
{
return;//!!!!
  // This is from a case worked out by hand by Lonnie Love
  Matrix A( zeroMatrix(3,6) );
                A(0,1)=-0.325l;
  A(1,0)=2.064;                                            A(1,4)=0.54;
                A(2,1)=1.943;  A(2,2)=1.156; A(2,3)=0.674;

  Vector b(3);
  b[0]=0; b[1]=0; b[2]=1;

  array<Int> dependentRowsEliminated;

  Matrix gi = fsp->solve(A,b,dependentRowsEliminated);

//  Debugln(DJ,"gi=\n" << gi);
//abort();//!!!!!
}


void IKORFullSpaceSolverTest::testSpecialCase1()
{
return;//!!!!
  // This is the example of special case 1 from the
  // paper FSP (Full Space Parameterization), Version 2.0
  //  Fries, Hacker & Pin
  // ORNL/TM-13021

  Matrix A( zeroMatrix(3,9) );
  A(0,6)=1; A(0,8)=-0.61;
  A(1,0)=-1.236; A(1,1)=0.635; A(1,2)=0.88; A(1,5)=0.343; A(1,7)=1; A(1,8)=-0.893;
  A(2,3)=-0.851; A(2,4)=-0.343;

  Vector b( zeroVector(3) );
  b(0)=-0.0101;

  array<Int> dependentRowsEliminated;

  Matrix gi = fsp->solve(A,b,dependentRowsEliminated);

  // now construct the solution given in the paper and compare
  Matrix gi2( zeroMatrix(7,9) );
  gi2(0,6)=-0.0101;
  gi2(1,0)=-0.012; gi2(1,8)=0.0166;
  gi2(2,1)=0.0233; gi2(2,8)=0.0166;
  gi2(3,2)=0.0168; gi2(3,8)=0.0166;
  gi2(4,5)=0.0431; gi2(4,8)=0.0166;
  gi2(5,7)=0.0148; gi2(5,8)=0.0166;
  gi2(6,3)=0.3738; gi2(6,4)=-0.9275; gi2(6,6)=-0.0101;

  gi2.transpose(); // transpose from row vectors to column vectors

  CPPUNIT_ASSERT( gi.equals(gi2, 0.0001) );

}



void IKORFullSpaceSolverTest::testSpecialCase2()
{
return;//!!!!
  // This is the example of special case 2 from the
  // paper FSP (Full Space Parameterization), Version 2.0
  //  Fries, Hacker & Pin
  // ORNL/TM-13021

  Matrix A(3,6);
  A(0,0)=1; A(0,1)=1; A(0,2)=2; A(0,3)=2; A(0,4)=4; A(0,5)=3;
  A(1,0)=2; A(1,1)=5; A(1,2)=4; A(1,3)=0; A(1,4)=9; A(1,5)=6;
  A(2,0)=3; A(2,1)=2; A(2,2)=6; A(2,3)=4; A(2,4)=8; A(2,5)=9;

  Vector b(3);
  b(0)=3; b(1)=7; b(2)=6;

  array<Int> dependentRowsEliminated;

  Matrix gi = fsp->solve(A,b,dependentRowsEliminated);


  // now construct the solution given in the paper and compare
  Matrix gi2( zeroMatrix(4,6) );
  gi2(0,1)=1.4; gi2(0,3)=0.8;
  gi2(1,3)=-0.0556; gi2(1,4)=0.778;
  gi2(2,1)=1.4; gi2(2,2)=-0.8321; gi2(2,3)=0.8; gi2(2,5)=0.5547;
  gi2(3,0)=0.9636; gi2(3,1)=1.4; gi2(3,2)=-0.1482; gi2(3,3)=0.8; gi2(3,5)=-0.2224;
  gi2.transpose(); // transpose from row vectors to column vectors

  Debugln(DJ,"gi (from FSP)=\n" << gi);
  Debugln(DJ,"gi2 (from paper)=\n" << gi2);

  CPPUNIT_ASSERT( gi.equals(gi2, 0.0001) );

}



void IKORFullSpaceSolverTest::testEnduranceExample()
{
  // This is the example of an endurance test from the
  // paper FSP (Full Space Parameterization), Version 2.0
  //  Fries, Hacker & Pin
  // ORNL/TM-13021

  Matrix A(4,8);
  A(0,0)=1; A(0,1)=0; A(0,2)=0; A(0,3)=0; A(0,4)=0; A(0,5)=1; A(0,6)=0; A(0,7)=0;
  A(1,0)=0; A(1,1)=3; A(1,2)=1; A(1,3)=0; A(1,4)=2; A(1,5)=0; A(1,6)=1; A(1,7)=4;
  A(2,0)=0; A(2,1)=6; A(2,2)=0; A(2,3)=0; A(2,4)=4; A(2,5)=0; A(2,6)=0; A(2,7)=8;
  A(3,0)=0; A(3,1)=5; A(3,2)=6; A(3,3)=0; A(3,4)=0; A(3,5)=0; A(3,6)=6; A(3,7)=0;

  Vector b(4);
  b[0]=0; b[1]=1; b[2]=2; b[3]=3;

  array<Int> dependentRowsEliminated;

  Matrix gi = fsp->solve(A,b,dependentRowsEliminated);

  Debugln(DJ,"gi=\n" << gi);
}


#ifdef DEBUG
CPPUNIT_TEST_SUITE_REGISTRATION( IKORFullSpaceSolverTest );
#endif

