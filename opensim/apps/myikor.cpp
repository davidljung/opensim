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
  
  $Id: myikor.cpp 1026 2004-02-11 20:44:02Z jungd $
  $Revision: 1.3 $
  $Date: 2004-02-11 15:44:02 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <robot/control/kinematics/kinematics>
#include <robot/JFKengine>
#include <robot/NumericKinematicEvaluator>
#include <robot/KinematicChain>

#include <robot/TestRobot>


#include <base/TestTest>

using robot::KinematicChain;
using robot::TestRobot;
using robot::KinematicEvaluator;
using robot::JFKengine;
using robot::NumericKinematicEvaluator;
using robot::RobotDescription;


//
// temp includes for testing
//
#include <base/Vector>
#include <base/Matrix>
#include <base/SVD>
#include <robot/ControlInterface>
#include <robot/Robot>


using namespace base;



// helpers for hard-coding Puma Jacobian
// (NB: the non-variable joint parameters could be evaluated here
//  and expressed as constants directly)
Vector p(24);
Expression theta(Int i) { return Expression::p[i-1]; }
Expression alpha(Int i) { return Expression(p[i+6-1]); }
Expression a(Int i) { return Expression(p[i+12-1]); }
Expression d(Int i) { return Expression(p[i+12+6-1]); }
Expression Ci(Int i) { return cos(theta(i)); }
Expression Si(Int i) { return sin(theta(i)); }
Expression Cij(Int i, Int j) { return cos(theta(i)+theta(j)); }
Expression Sij(Int i, Int j) { return sin(theta(i)+theta(j)); }




int main(int argc, char *argv[])
{
  setDebugOutputID(Tmp);


  //
  // scratch test stuff
  //

  // test Expression
  /*
  {
  base::Vector p(3); // 3 element parameter vector
  Expression sub = sin(Expression(3)) + sin(Expression::p[2]); 
  Expression sum = 4*sin(sub)+sub+(2*sub);

  p[0] = 2;
  p[2] = 1.5;

  // sin(double) is only called 3 times (common sub-expressions are not re-evaluated)
  std::cout << sum << " = " << sum.evaluate(p) << " where p=" << p << "\n";

  sum = -sum;
  std::cout << sum << " = " << sum.evaluate(p) << " where p=" << p << "\n";


  // An ExpressionMatrix or ExpressionVector will work in the usual way
  //  e.g. we can invoke matrix multiply etc. and it will work as expected
  std::cout << "ExpressionMatrix & ExpressionVector test\n";
  ExpressionMatrix A(3,3);
  //set_diagonal(A,1);
  A(0,0)=A(1,1)=A(2,2)=1;
  A(1,1) = sub;
  A(1,2) = Expression::p[2];
  std::cout << "A=" << A << std::endl;
  
  ExpressionVector x(3);
  x[0] = 1;
  x[1] = Expression::p[0];
  x[2] = -1;

  std::cout << "x=" << x << std::endl;
  ExpressionVector b = A*x;
  std::cout << "b=A*x=" << b << std::endl;


  std::cout << "b[1]=" << b[1] << "=" << (b[1]).evaluate(p) << std::endl;
  }
  */
  
  // Matrix stuff

  // Test SVD
  /*
  {
  Matrix M(4,3);
  M = zeroMatrix(4,3);
  M(0,1) = 4;
  M(1,2) = 7;
  M(2,1) = -3;
  M(3,2) = 8;

  Matrix MI( Math::pseudoInverse(M) );

  std::cout << "M=" << M << std::endl;
  std::cout << "MI=" << MI << std::endl;
  std::cout << "M*MI=" << (M*MI) << std::endl;
  std::cout << "M*MI*M=" << (M*MI*M) << std::endl;
  }
  */

  //
  // Prototype driver
  //

  /*
  // Hard-code the Forward Kinematic Transform for a Puma robot (Robotics, pp45) (non-redundant)
  Vector p(24); // Puma joint variables/parameters
  // p[0-5] - theta - variable; p[6-11] - alpha, p[12-17] - a, p[18-23] - d

  p[6] = Math::degToRad(-90); // alphas
  p[7] = Math::degToRad(0);
  p[8] = Math::degToRad(90);
  p[9] = Math::degToRad(-90);
  p[10] = Math::degToRad(90);
  p[11] = Math::degToRad(0);

  p[12] = 0; // as
  p[13] = 431.8; // mm
  p[14] = -20.32;
  p[15] = 0;
  p[16] = 0;
  p[17] = 0;

  p[18] = 0; // ds
  p[19] = 149.09; // mm
  p[20] = 0; 
  p[21] = 433.07;
  p[22] = 0; 
  p[23] = 56.25; 
  ::p = p;

  ExpressionMatrix T(4,4);

  Expression nx = Ci(1)*(Cij(2,3)*(Ci(4)*Ci(5)*Ci(6)-Si(4)*Si(6))-Sij(2,3)*Si(5)*Ci(6))-Si(1)*(Si(4)*Ci(5)*Ci(6)+Ci(4)*Si(6));
  Expression ny = Si(1)*(Cij(2,3)*(Ci(4)*Ci(5)*Ci(6)-Si(4)*Si(6))-Sij(2,3)*Si(5)*Ci(6) )+Ci(1)*(Si(4)*Ci(5)*Ci(6)+Ci(4)*Si(6));
  Expression nz = -Sij(2,3)*(Ci(4)*Ci(5)*Ci(6)-Si(4)*Si(6))-Cij(2,3)*Si(5)*Ci(6);
  
  Expression sx = Ci(1)*(-Cij(2,3)*(Ci(4)*Ci(5)*Si(6)+Si(4)*Ci(6))+Sij(2,3)*Si(5)*Si(6))-Si(1)*(-Si(4)*Ci(5)*Si(6)+Ci(4)*Ci(6));
  Expression sy = Si(1)*(-Cij(2,3)*(Ci(4)*Ci(5)*Si(6)+Si(4)*Ci(6))+Sij(2,3)*Si(5)*Si(6))+Ci(1)*(-Si(4)*Ci(5)*Si(6)+Ci(4)*Ci(6));
  Expression sz = Sij(2,3)*(Ci(4)*Ci(5)*Si(6)+Si(4)*Ci(6))+Cij(2,3)*Si(5)*Si(6);

  Expression ax = Ci(1)*(Cij(2,3)*Ci(4)*Si(5)+Sij(2,3)*Ci(5))-Si(1)*Si(4)*Si(5);
  Expression ay = Si(1)*(Cij(2,3)*Ci(4)*Si(5)+Sij(2,3)*Ci(5))+Ci(1)*Si(4)*Si(5);
  Expression az = -Sij(2,3)*Ci(4)*Si(5)+Cij(2,3)*Ci(5);

  Expression px = Ci(1)*(d(6)*(Cij(2,3)*Ci(4)*Si(5)+Sij(2,3)*Ci(5))+Sij(2,3)*d(4)+a(3)*Cij(2,3)+a(2)*Ci(2))-Si(1)*(d(6)*Si(4)*Si(5)+d(2));
  Expression py = Si(1)*(d(6)*(Cij(2,3)*Ci(4)*Si(5)+Sij(2,3)*Ci(5))+Sij(2,3)*d(4)+a(3)*Cij(2,3)+a(2)*Ci(2))+Ci(1)*(d(6)*Si(4)*Si(5)+d(2));
  Expression pz = d(6)*(Cij(2,3)*Ci(5)-Sij(2,3)*Ci(4)*Si(5))+Cij(2,3)*d(4)-a(3)*Sij(2,3)-a(2)*Si(2);


  T(0,0) = nx; T(0,1) = sx; T(0,2) = ax; T(0,3) = px;
  T(1,0) = ny; T(1,1) = sy; T(1,2) = ay; T(1,3) = py;
  T(2,0) = nz; T(2,1) = sz; T(2,2) = az; T(2,3) = pz;
  T(3,0) = T(3,1) = T(3,2) = Expression(0); T(3,3) = Expression(1);
  
  Int addsub=0, multdiv=0, trig=0;
  Int saddsub=0, smultdiv=0, strig=0;
  for(Int r=0; r<4; r++)
    for(Int c=0; c<4; c++) {
      T(r,c).operationCounts(addsub,multdiv,trig);
      T(r,c).simplify();
      T(r,c).operationCounts(saddsub,smultdiv,strig);
    }

  std::cout << "T=" << T << std::endl;
  std::cout << "O(T) addsub:" << addsub << " multdiv:" << multdiv << " trig:" << trig << std::endl;
  std::cout << "after simplify - O(T) addsub:" << saddsub << " multdiv:" << smultdiv << " trig:" << strig << std::endl;

  // test it
  p[0] = Math::degToRad(90); // set thetas
  p[1] = Math::degToRad(0);
  p[2] = Math::degToRad(90);
  p[3] = Math::degToRad(0);
  p[4] = Math::degToRad(0);
  p[5] = Math::degToRad(0);

  // evaluate transform
  Matrix eT(4,4);
  for(Int r=0; r<4; r++)
    for(Int c=0; c<4; c++) 
      eT(r,c) = T(r,c).evaluate(p);

  std::cout << "eT=" << eT << std::endl;
  */

  //
  // Main loop
  //
  
  

//  ref<TestRobot> puma(NewObj TestRobot()); // the robot
//  RobotDescription rd(puma->getRobotDescription());
  // get manipulator kinematics
//  KinematicChain chain(rd.manipulators()[0].getKinematicChain());
  
  // modify chain to make it more interesting
//  chain[2].setA(0.5);
//  KinematicChain link(KinematicChain::Link(KinematicChain::Link::Prismatic, Math::degToRad(45), 0.5, 1, 0));
//  link += chain; chain = link; // prepend
//  chain += KinematicChain::Link(KinematicChain::Link::Prismatic, Math::degToRad(-30), 0.2, 0.2, Math::degToRad(30)); // append
  
  // ljl stuff
  // order of parameters:  alfa, a, d, theta 
  KinematicChain chain;	// empty                                           alfa,    a,      d,       theta
  chain+=KinematicChain::Link(KinematicChain::Link::Prismatic, Math::degToRad(45),  0.5,    0,       0);
  chain+=KinematicChain::Link(KinematicChain::Link::Revolute,  Math::degToRad(-90), 0,      0,       0);
  chain+=KinematicChain::Link(KinematicChain::Link::Revolute,  Math::degToRad(0),   0.4318, 0.14909, 0);
  chain+=KinematicChain::Link(KinematicChain::Link::Revolute,  Math::degToRad(90),  0.5,    0,       0);
  chain+=KinematicChain::Link(KinematicChain::Link::Revolute,  Math::degToRad(-90), 0,      0.43307, 0);
  chain+=KinematicChain::Link(KinematicChain::Link::Revolute,  Math::degToRad(90),  0,      0,       0);
  chain+=KinematicChain::Link(KinematicChain::Link::Revolute,  Math::degToRad(0),   0,      0.05625, 0);
  chain+=KinematicChain::Link(KinematicChain::Link::Prismatic, Math::degToRad(-30), 0.2,    0,       0);

// Puma
//  chain+=KinematicChain::Link(KinematicChain::Link::Revolute, Math::degToRad(0),   0, 0, 0);
//  chain+=KinematicChain::Link(KinematicChain::Link::Revolute, Math::degToRad(-90), 0, 2, 0);
//  chain+=KinematicChain::Link(KinematicChain::Link::Revolute, Math::degToRad(0),   3, 0, 0);
//  chain+=KinematicChain::Link(KinematicChain::Link::Revolute, Math::degToRad(-90), 4, 4, 0);
//  chain+=KinematicChain::Link(KinematicChain::Link::Revolute, Math::degToRad(90),  0, 0, 0);
//  chain+=KinematicChain::Link(KinematicChain::Link::Revolute, Math::degToRad(-90), 0, 0, 0);

//  Stanford arm
//  chain+=KinematicChain::Link(KinematicChain::Link::Revolute, Math::degToRad(-90), 0, 0, 0);
//  chain+=KinematicChain::Link(KinematicChain::Link::Revolute, Math::degToRad(90),  0, 2, 0);
//  chain+=KinematicChain::Link(KinematicChain::Link::Prismatic,Math::degToRad(0),   0, 0, 0);
//  chain+=KinematicChain::Link(KinematicChain::Link::Revolute, Math::degToRad(-90), 0, 0, 0);
//  chain+=KinematicChain::Link(KinematicChain::Link::Revolute, Math::degToRad(90),  0, 0, 0);
//  chain+=KinematicChain::Link(KinematicChain::Link::Revolute, Math::degToRad(0),   0, 6, 0);

  std::cout << chain << std::endl;


  // get Forward Kinematics F
  ref<JFKengine> jfke( NewObj JFKengine() );
  ExpressionMatrix F(jfke->getForwardKinematics(chain));
  
  // get Jacobian 
  ExpressionMatrix J(jfke->getJacobian(chain));

  Int saddsub=0, smultdiv=0, strig=0;
  operationCounts(J, saddsub,smultdiv,strig);

  //Debugln(Tmp,"J(" << J.size1() << "," << J.size2() << ")=\n" << J);
  std::cout << "O(J) addsub:" << saddsub << " multdiv:" << smultdiv << " trig:" << strig << std::endl;

//!!!!
allTests.runAllTests();

  // evaluate the Jacobian at a particular configuration
  //  using both symbolic and numerical methods

  Vector q( zeroVector( chain.dof() ) );
  q[0] = 0.5; q[1] = 0.1; // etc..

  // already have JFKengine instantated above
  //  tell chain to use it for computations
  chain.setKinematicEvaluator(jfke);

  Matrix Jq1( chain.getJacobian(q) );
  std::cout << "\nJq using JFKengine=\n" << Jq1 << std::endl;

  // instantiate the numeric evaluator
  ref<KinematicEvaluator> nke( NewObj NumericKinematicEvaluator() );
  chain.setKinematicEvaluator(nke); // tell chain to use it

  Matrix Jq2( chain.getJacobian(q) );
  std::cout << "\nJq using NumericKinematicEvaluator=\n" << Jq2 << std::endl;


  return 0;
}


