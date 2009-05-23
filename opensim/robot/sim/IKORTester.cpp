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

  $Id: IKORTester.cpp 1103 2004-09-27 21:56:52Z jungd $
  $Revision: 1.14 $
  $Date: 2004-09-27 17:56:52 -0400 (Mon, 27 Sep 2004) $
  $Author: jungd $

****************************************************************************/

#include <robot/sim/IKORTester>

#include <base/Externalizer>
#include <base/externalization_error>
#include <base/Expression>
#include <base/Matrix>
#include <base/VDirectory>
#include <base/VFile>

#include <robot/AggregateControlInterface>
#include <robot/sim/BasicEnvironment>
#include <robot/control/kinematics/solution_error>
#include <robot/control/kinematics/Optimizer>
#include <robot/control/kinematics/ReferenceOpVectorFormObjective>
#include <robot/control/kinematics/AnalyticLagrangianFSBetaOptimizer>
#include <robot/control/kinematics/MPPseudoInvSolver>
#include <robot/control/kinematics/LeastNormIKSolver>
#include <robot/control/kinematics/FullSpaceSolver>
#include <robot/control/kinematics/IKOR>


using robot::sim::IKORTester;

using base::Vector;
using base::Orient;
using base::Time;
using base::PathName;
using base::VFile;
using base::VDirectory;
using base::ExpressionMatrix;
using base::externalization_error;
using base::Path;
using base::Trajectory;
using robot::AggregateControlInterface;
using robot::sim::BasicEnvironment;
using robot::sim::IKORTest;
using robot::control::kinematics::FullSpaceSolver;
using robot::control::kinematics::ReferenceOpVectorFormObjective;
using robot::control::kinematics::BetaFormConstraints;
using robot::control::kinematics::AnalyticLagrangianFSBetaOptimizer;
using robot::control::kinematics::InverseKinematicsSolver;
using robot::control::kinematics::MPPseudoInvSolver;
using robot::control::kinematics::LeastNormIKSolver;
using robot::control::kinematics::FullSpaceSolver;
using robot::control::kinematics::IKOR;





IKORTester::IKORTester(ref<base::VFileSystem> fs, ref<base::Cache> cache)
  : filesystem(fs), cache(cache)
{
}


void IKORTester::executeTests(ref<IKORTest> itest, bool saveResults, base::PathName alternateOutputFileName)
{
  ref<SimulatedRobot> robot( narrow_ref<SimulatedRobot>(itest->getRobot()) );
  Int testManipulatorIndex = itest->getManipulatorIndex();
  Consoleln("Testing robot:" << robot->getRobotDescription()->getName() << " with manipulator:"
                             << robot->getRobotDescription()->manipulators()[testManipulatorIndex]->getName());

  // duplicate the environment, so we can restore the environment to its initial state before saving
  ref<SimulatedBasicEnvironment> env( narrow_ref<SimulatedBasicEnvironment>(itest->getEnvironment()) );
  ref<SimulatedBasicEnvironment> initialenv( &base::clone(*env) );

  env->preSimulate();
  Time simTime(0);
  for(Int i=0; i<itest->numTests(); i++) {
    IKORTest::Test& test( itest->getTest(i) );
    Consoleln("Executing test: " << test.getName());
    simTime = executeTest(test, simTime, env, robot, testManipulatorIndex);
  }

  if (saveResults) {
    itest->setEnvironment(initialenv);
    itest->saveResults(true,alternateOutputFileName);
  }
}








base::Time IKORTester::executeTest(IKORTest::Test& test, Time simTime, ref<SimulatedBasicEnvironment> env,
                                   ref<SimulatedRobot> robot, Int testManipulatorIndex)
{
  // setup
  ref<const RobotDescription> robotDescr(robot->getRobotDescription());
  ref<const ManipulatorDescription> manipDescr(robotDescr->manipulators()[testManipulatorIndex]);

  // get the kinematic chain for the robot and its first manipulator
  Matrix4 platformTransform( robot->getOrientation().getRotationMatrix3() );
  platformTransform.setTranslationComponent( robot->getPosition() );
  KinematicChain platformChain( robotDescr->platform()->getKinematicChain(3,platformTransform) ); // could have 0 dof
  KinematicChain robotChain(robotDescr->getKinematicChain(3,platformTransform,testManipulatorIndex,0));
  KinematicChain manipChain(manipDescr->getKinematicChain());

  // get control interfaces
  String manipIndexStr( base::intToString(testManipulatorIndex+1) );
  ref<ControlInterface> platformControl(robot->getControlInterface("platformPosition"));
  ref<ControlInterface> manipControl(robot->getControlInterface(String("manipulatorPosition")+manipIndexStr));
  ref<ControlInterface> toolControl(robot->getControlInterface(String("toolPosition")+manipIndexStr));
  ref<ControlInterface> toolGripControl(robot->getControlInterface(String("manipulatorToolGrip")+manipIndexStr));

  // put interfaces into an array in kinematic order (for aggregation)
  array<ref<ControlInterface> > interfaces;
  interfaces.push_back(platformControl);
  interfaces.push_back(manipControl);

  KinematicChain manipToolChain(manipChain); // combined manip & tool chain (==manipChain if no tool)
  KinematicChain toolChain;
  if (test.toolAttached) {
    // locate tool
    test.toolAttached = false;
    for(Int i=0; (i<env->numTools()) && !test.toolAttached; i++) {
      ref<const ToolDescription> td( env->getTool(i)->getToolDescription() );

      if (td->getName() == test.toolName) {
        test.toolAttached = true;
        env->placeToolInProximity( env->getTool(i), robot, 0);
        toolGripControl->setOutput(0,1); // tell manipulator to grasp tool
        toolChain = td->getKinematicChain();
//!!! hack
toolControl->setOutputs( zeroVector(toolChain.dof()) );
Logln("debug: setting tool outputs to 0");
//!!!
      }
    }
    if (!test.toolAttached)
      throw std::logic_error(Exception(String("the specified tool '")+test.toolName+"' doesn't exist in the environment"));

    manipToolChain += toolChain;
  }



  KinematicChain chain(robotChain); // kinematic chain from world frame to tool end-effector
  if (test.toolAttached) {
    chain += toolChain;
    interfaces.push_back(toolControl);
  }


  // ControlInterace for all dofs: platform, manipulator & tool
  //  (I/O vector components match dofs of chain)
  ref<ControlInterface> control(NewObj AggregateControlInterface("robotPosition","RobotPositionControl", interfaces));


  Vector q( zeroVector(chain.dof()) );

  // put manipulator into initial configuration (if needed)
  if (test.initialConfigSpecified) {

    if (test.initq.size() != chain.dof())
      throw std::logic_error(Exception(String("the initial configuration specified has ")
                                       + base::intToString(test.initq.size())
                                       + " components, but the platform, manipulator & tool (if any) has "
                                       + base::intToString(chain.dof())+" d.o.f"));

    q = test.initq;

    // convert components of q corresponding to revolute joints to radians
    for(Int v=0; v<chain.dof(); v++)
      if (chain.variableUnitType(v) == KinematicChain::Angle)
        q[v] = Math::degToRad(q[v]);

    Assertm(chain.dof() == control->outputSize(),"KinematicChain dof == aggregated ControlInterface output vector size");
    control->setOutputs(q);
  }
  else  // get config from robot
    q.reset( control->getInputs() );



  Trajectory traj(test.traj);

  // if units were specified for the trajectory, and they are not "meters", then
  //  convert to meters
  if (traj.getUnits() != "") {

    if (traj.getUnits() == "inches")
      traj.scalePosition(consts::metersPerInch);
    else
      if (traj.getUnits() != "meters") {
        Logln("Warning: unknown units specified for trajectory: '" << traj.getUnits() << "' - no conversion performed.");
      }
  }


  // all test are performed in the WorldFrame, so convert the trajectory to world frame coords.
  if (test.frame != Robot::WorldFrame) {

    // make this more elegant later!!!
    //!!! I think this may be broken - overhaul it! !!!
Logln("warning: trajectory/path transformation to world frame not well tested");
    Vector mtq( manipToolChain.dof() );
    mtq = vectorRange(q, Range(platformChain.dof(),q.size()) );

    Matrix T(manipToolChain.getForwardKinematics(mtq)); // manip|tool ee -> manip mount frame transform

    Matrix4 toWorldTransform( robot->coordFrameTransform(test.frame, Robot::WorldFrame, 0, base::toMatrix4(T),
                                                         robot->getPosition(),
                                                         robot->getOrientation()) );


    traj.transform( toWorldTransform );
  }



  // if an overriding time interval was specified, replace the current trajectory time
  //  information with it
  Real stepsize = 0.01; // default
  if (test.timeIntervalSpecified) {
    // achieved by converting the trajectory to a Path (hence loosing the time information)
    //  and then back to a trajectory with the information specified from the interval
Logln("warning: trajectory/path resamping not well tested");
    Path path(traj);
    Real start = test.timeInterval[0];
    Real end = test.timeInterval[1];
    Real duration = end-start;

    traj = Trajectory(path);
    traj.scaleTime(duration);
    traj.shiftTime(start);

    if (test.timeInterval[2] > 0) { // i.e. stepsize given
      stepsize = test.timeInterval[2];
      traj.resample( Int(duration/stepsize) );
    }
//!!! this may break things too!! !!!
  }


  // if a limit on the maximum distance between any two trajectory points was specified, resample
  //  so that the inter-point distances are smaller than the maxdx specified
    if (test.maxdxSpecified)  // limit |dx|
      traj.resample(test.maxdx);



  //
  // finally, iterate over the trajectory, calculating dxs and using inverse-kinematics
  //  to solve for dqs

  ref<InverseKinematicsSolver>     ikSolver;
  if (test.solutionMethod == IKORTest::Test::PseudoInverse) {
    ikSolver = ref<InverseKinematicsSolver>(NewObj LeastNormIKSolver());
    if (test.jointWeightsSpecified) {
      Logln("Warning: joint parameter weights are ignored for the PseudoInverse solution method");
    }
  }
  else if (test.solutionMethod == IKORTest::Test::FullSpace) {
    bool nonHolonomicPlatformActive = (robotDescr->platform()->isMobile() && !robotDescr->platform()->isHolonomic());

    // get joint weights
    Vector jointWeights(test.jointWeightsSpecified? test.jointWeights : Vector() );
    if (jointWeights.size() == 0) { // not specified, set to (1,1,..,1)
      jointWeights.reset( Vector(chain.dof()) );
      for(Int i=0; i<jointWeights.size(); i++) jointWeights[i] = 1.0;
    }
    else
      if (jointWeights.size() != chain.dof())
        throw std::logic_error(Exception(String("the specified joint weight vector has ")
                                       + base::intToString(jointWeights.size())
                                       + " components, but the platform, manipulator & tool (if any) has "
                                       + base::intToString(chain.dof())+" d.o.f"));

    Real L = robotDescr->platform()->L();
    ikSolver = ref<InverseKinematicsSolver>(NewObj IKOR(chain, jointWeights, nonHolonomicPlatformActive, L));
  }
  Assert(ikSolver);


  // check obstacle avoidance is supported, if requested & get manip proximity sensor interface
  ref<ControlInterface> proximitySensorInterface;

  if (test.optConstraints.test(IKOR::ObstacleAvoidance)) {
    if (ikSolver->isConstraintTypeSupported(IKOR::ObstacleAvoidance, test.optMethod, test.optCriteria))
      proximitySensorInterface = robot->getControlInterface(String("manipulatorProximity")+manipIndexStr);
    else
      throw std::logic_error(Exception("obstacle avoidance constraints were requested, but the IK solver doesn't support them with the specificed optimizatio method and criteria"));
  }



  // clear arrays to hold the resulting platform/manipulator/tool trajectory
  test.qs.clear();
  test.xs.clear();
  test.dxs.clear();
  test.dqs.clear();
  test.times.clear();
  test.Js.clear();

  if (simTime.equals(0)) // advance initial simTime if necessary
    if (simTime < traj.time(0))
      simTime = traj.time(0);

  if (simTime > traj.time(0))
    throw std::logic_error(Exception("simulation time at start of trajectory is already greater than the specified start-time of the trajectory"));


  // get initial q and calculate x
  q = control->getInputs();
  Vector x(calcEEVector(test,chain,q));
  test.qs.push_back(q);
  test.xs.push_back(x);
  test.times.push_back(simTime);
  test.Js.push_back( chain.getJacobian(q, test.orientationControl) );

  Logln("initial q=" << q << "  x=" << x);


  // if the initial ee position is coincident with the first trajectory point *and* the
  //  initial time is identical to the current simTime, skip the first trajectory point
  // However, if the positions are not coincident, but the times are - it is an error
  //  (cannot move intantaneously).  If the positions are coincident but the times are
  //  not, the initial dx will be 0.

  bool coincident=false; // position[/orientation] coincident?
  // check if the initial ee is co-incident with the beginning of the trajectory (to within tol)
  const Real tol = 0.01;
  Vector3 pos( base::toVector3(vectorRange(x,Range(0,3))) );
  Orient orient;
  if (!test.orientationControl) {
    if (pos.equals( traj.position(traj.time(0)),tol ))
      coincident=true;
  }
  else {
    orient = Orient( vectorRange(x,Range(3,6)), Orient::EulerRPY );
    if ( pos.equals( traj.position(traj.time(0)),tol ) && orient.equals( traj.orientation(traj.time(0)),tol ) )
      coincident=true;
  }


  if (!coincident) {
    if (traj.time(0) == simTime) {
      String posorient(base::toString(traj.position(0)));
      String eeposorient(base::toString(pos));
      if (test.orientationControl) {
        Orient trajo( traj.orientation(0) ); trajo.changeRepresentation(Orient::EulerRPY);
        Orient eeo(orient); eeo.changeRepresentation(trajo.representation());
        posorient += String("/")+base::toString( trajo );
        eeposorient += String("/")+base::toString( eeo );
      }
      throw std::logic_error(Exception(String("the initial trajectory time is equal to the initial simulation time, but initial trajectory position[/orientation] ")
                +posorient+
                +"(WorldFrame) is not coincident with the initial end-effector "
                +eeposorient+
                +"(WorldFrame) to within tolerance.  Can't move the end-effector instantaneously!"));
    }
  }



  // loop over traj
  bool abort=false;
  Int i=coincident?1:0;
  for(; (i<traj.numDistinguishedValues()) && (!abort); i++) {

    Real s = traj.distinguishedValue(i);
    Real prevs = traj.distinguishedValue( (i-1>0)?i-1:0 );
    Time t = traj.time(s);
    Time prevt = traj.time(prevs);
    Real dt = (t-prevt).seconds();

    // get trajectory pos/orient at time t and convert to Vector targetx
    Point3 targetPos( traj.position(t) );
    Orient targetOrient( test.orientationControl?traj.orientation(t):Orient() );
    Vector targetx(test.orientationControl?6:3);
    vectorRange(targetx, Range(0,3)) = base::fromVector3(targetPos);
    if (test.orientationControl)
      vectorRange(targetx, Range(3,6)) = targetOrient.getVector(Orient::EulerRPY);
    Vector dx( targetx - x );

    if (test.orientationControl) {
      // need to convert delta orientation components into angular velocities, dw (delta omega)
      // Assumption: we can use vector algebra (the vector subtraction above) on the
      //  orientation components (which are of arbitrary representation)
      // Then we use Orient::getBinv() to obtain a matrix Binv, which when multiplied by the
      //  deltas gives dw.
      Orient orient(vectorRange(x,Range(3,x.size())),Orient::EulerRPY); // pull out orientation
      Vector dorient = vectorRange(dx,Range(3,dx.size()));
      Vector domega = orient.getBinv()*Vector(dorient); // w = B(o)^-1 . do

      Vector newdx(6);
      vectorRange(newdx,Range(0,3)) = vectorRange(dx,Range(0,3)); // pos components
      vectorRange(newdx,Range(3,6)) = domega;                     // orient components
      dx = newdx;
    }

//Debugln(DJ,"\nx=" << x << "\ntargetx=" << targetx << "\ndx=" << dx);
    test.dxs.push_back(dx);

    Vector dq( chain.dof() );


    if (test.optConstraints.test(IKOR::ObstacleAvoidance)) {
      // read proximity sensor data from manipulator and give it to the solver so that
      //  obstacle constraints can be imposed as appropriate
      // (note that the interface provides prox data by link of the manipulator, but the
      //  solver expects prox data by dof variable for the whole chain (platform & manip) )
      array<InverseKinematicsSolver::LinkProximityData> proxData( chain.dof() ); // elements initialize to no-proximity

      // indices correspond to links (including base), not parameters (some links may have 0-dof)
      Vector proximityInputs( proximitySensorInterface->getInputs() );

      // fill in proxData from proximityInputs
      //  interface only provides prox data for manip[/tool], so skip over platform dofs (if any)
      for(Int v=0; v < chain.dof(); v++) { // for each platf/manip[/tool] parameter variable

        if (v >= platformChain.dof()+1) { // only handling manip link prox sensors (excluding base)

          Int l = manipToolChain.linkIndexOfVariable(v - platformChain.dof()); // manip[/tool] link index
          InverseKinematicsSolver::LinkProximityData& pd(proxData[v]);

          pd.distance =    proximityInputs[l*5]; // see SimulatedRobot for interface input vector description
          pd.direction.resize(3);
          pd.direction[0] = proximityInputs[l*5+1];
          pd.direction[1] = proximityInputs[l*5+2];
          pd.direction[2] = proximityInputs[l*5+3];
          pd.intercept   = proximityInputs[l*5+4];
        }
      }

      Real d = 0.3;
      ikSolver->setProximitySensorData(proxData, d);

    }

//Debugln(DJ,"t:" << simTime.seconds() << "  q:" << q << "  x:" << x << "  dx:" << dx );
    // attempt IK
    Matrix J;
    try {

      J = chain.getJacobian(q, test.orientationControl);
      dq = ikSolver->solve(dx, dt, x, q, J, test.optMethod, test.optCriteria, test.optConstraints);

//!!! hack for now to limit |dq|
Debugln(DJ,"|dq|=" << dq.magnitude());
if (dq.magnitude() > 0.2) {
  dq /= ( dq.magnitude()/0.2 );
  Debugcln(DJ,"scaling dq");
}
//!!!

    }
    catch (std::exception& e) {
      abort=true;
      test.failureString = e.what();
    }

    if (!abort) {
//Debugcln(DJ,"==dq:" << dq);
      test.dqs.push_back(dq);

      // move the platform/manipulator/tool
      control->setOutputs( q + dq );

      // 'simulate'
      env->simulateForSimTime( t-simTime );
      simTime = t;

      // update q & x next iteration
      q = control->getInputs();
      test.qs.push_back(q);
      test.times.push_back(simTime);
      x = calcEEVector(test,chain,q); // new ee pos/orient
      test.xs.push_back(x);
      test.Js.push_back(J);
    }

  } // end traj loop


  test.testCompleted = !abort;
  if (!test.testCompleted) {
    Logln("IK solution failed after " << i << " trajectory points.");
    Logln("time:" << simTime.seconds() << " trajectory position:"
          << traj.position(simTime) << " (WorldFrame).");
    Logln("failure reason:\n" << test.failureString);
  }
  Logln("  final q=" << q << "  x=" << x << " (" << i << " steps)");

  test.resultsPresent = true;
  return simTime;
}





base::Vector IKORTester::calcEEVector(const IKORTest::Test& test, const KinematicChain& chain, const base::Vector& q) const
{
  // calculate the EE position using the current joint parameters and the forward kinematics transform (Tn)
  Matrix Tn( chain.getForwardKinematics(q) );

  //  the position is the first 3 elements of column 4 of T
  //  the orientation is the 3x3 submatrix of T - converted into a EulerRPY vector
  Vector pos(3); pos = vectorRange(Vector(matrixColumn(Tn,3)), Range(0,3));
  Vector x(test.orientationControl?6:3);
  vectorRange(x, Range(0,3)) = pos;

  if (test.orientationControl) {
    Matrix rot(3,3); rot = matrixRange(Tn, Range(0,3), Range(0,3));
    Vector orient( Orient(rot).getVector(Orient::EulerRPY) );
    vectorRange(x, Range(3,x.size())) = orient;
  }
  return x; // WorldFrame
}

