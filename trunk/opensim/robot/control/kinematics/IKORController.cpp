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

  $Id: IKORController.cpp 1125 2004-09-28 20:44:57Z jungd $

****************************************************************************/

#include <robot/control/kinematics/IKORController>

#include <robot/control/kinematics/MPPseudoInvSolver>
#include <robot/control/kinematics/LeastNormIKSolver>
#include <robot/control/kinematics/FullSpaceSolver>
#include <robot/control/kinematics/IKOR>


using robot::control::kinematics::IKORController;

using base::Matrix;
using base::Orient;
using robot::ControlInterface;
using robot::control::kinematics::LeastNormIKSolver;
using robot::control::kinematics::IKOR;


/// \todo optMethod, etc. should be passed as parameters (or settable via set methods)
IKORController::IKORController(IKMethod method, ref<robot::Robot> robot, Int manipulatorIndex,
                               bool platformActive, bool orientationControl,
                               base::Orient::Representation orientationRepresentation)
  : method(method), robot(robot),
    platformActive(platformActive), orientationControl(orientationControl),
    orientRep(orientationRepresentation),
    d(0.1), lastTime(0)
{
  ref<const RobotDescription> rd( robot->getRobotDescription() );
  if (!rd->platform()->isMobile() && platformActive)
    throw std::invalid_argument(Exception("A fixed Platform cannot be active"));

  array<ref<const ManipulatorDescription> > manipulators( rd->manipulators() );

  if (manipulators.size() == 0)
    throw std::invalid_argument(Exception("Robots without a manipulator not currently supported"));

  if (manipulatorIndex >= manipulators.size())
    throw std::out_of_range(Exception("manipulatorIndex is invalid"));

  ref<const ManipulatorDescription> manipulator( manipulators[manipulatorIndex] );

  if (manipulator->type() != ManipulatorDescription::Serial)
    throw std::invalid_argument(Exception("Only serial manipulators are supported"));

  // obtain descriptions of the kinematic chains
  manipChain = manipulator->getKinematicChain();


  // if the manipulator has proximity sensors, get the interface
  try {
    proxInterface = robot->getControlInterface(String("manipulatorProximity")+base::intToString(manipulatorIndex+1));
  } catch (std::invalid_argument& e) {
    proxInterface = ref<ControlInterface>(0);
  }


  switch (method) {

  case LeastNorm: {
    ikSolver       = ref<InverseKinematicsSolver>(NewObj LeastNormIKSolver());
    optMethod      = LeastNormIKSolver::DefaultMethod;
    optCriterion   = LeastNormIKSolver::LeastNorm;
    optConstraints.set( LeastNormIKSolver::DefaultConstraints );

  }
  break;

  case FSPLagrangian: {
    ikSolver       = ref<InverseKinematicsSolver>(NewObj IKOR(manipChain, Vector(), platformActive));
    optMethod      = IKOR::Lagrangian;
    optCriterion   = IKOR::LeastNorm;
    optConstraints.set(IKOR::JointLimits);
    if (proxInterface != 0)
      optConstraints.set(IKOR::ObstacleAvoidance);
  }
  break;

  default:
    throw std::invalid_argument(Exception("unsupported/unknown IKMethod"));
  }

  // initialize input/outputNames for ControlInterface to return
  inputName.at(0) = "x";
  inputName.at(1) = "y";
  inputName.at(2) = "z";
  for(Int o=0; o<9; o++)
    inputName.at(3+o) = "o"+base::intToString(o);

  outputName.at(0) = "x";
  outputName.at(1) = "y";
  outputName.at(2) = "z";
  for(Int o=0; o<9; o++)
    inputName.at(3+o) = "o"+base::intToString(o);

  x.reset(zeroVector( 3+Orient(orientRep).size() ));
  tx.reset(zeroVector(orientationControl?x.size():3));
}



void IKORController::setControlInterface(ref<ControlInterface> controlInterface)
{
  if (controlInterface->getType() == "JointPositionControl") {
    manipulatorInterface = controlInterface;
    // initially, set the target ee pos to the current ee pos
    calcEEPositionOrientation();
    tx = vectorRange(x,Range(0,tx.size()));
  }
  else {
    Logln("Ignoring useless interface - name:" << controlInterface->getName() << " type:" << controlInterface->getType());
  }
}


bool IKORController::iterate(const base::Time& time)
{
  if (!isConnected()) return false; // nothing to control
  if (lastTime == 0) { lastTime = time; return false; } // first iteration, do nothing

  Real dt = (time-lastTime).seconds();
  if (dt==0) return false; // no time has passed, nothing will have changed

  try {
    calcEEPositionOrientation(); // update q, x
    Vector x2(x);
    if (!orientationControl) {
      x2.resize(3);
      x2 = vectorRange(x,Range(0,3));
    }


    // read proximity sensor data (if any) from manipulator and give it to the solver so that
    //  obstacle constraints can be imposed as appropriate
    // (note that the interface provides prox data by link of the manipulator, but the
    //  solver expects prox data by dof variable for the chain)
    if (proxInterface != 0) {
      array<InverseKinematicsSolver::LinkProximityData> proxData( manipChain.dof() ); // elements initialize to no-proximity

      // indices correspond to links (including base), not parameters (some links may have 0-dof)
      Vector proximityInputs( proxInterface->getInputs() );

      // fill in proxData from proximityInputs
      for(Int v=0; v < manipChain.dof(); v++) { // for each platf/manip[/tool] parameter variable
        Int l = manipChain.linkIndexOfVariable(v); // manip[/tool] link index
        InverseKinematicsSolver::LinkProximityData& pd(proxData[v]);

        pd.distance =    proximityInputs[l*5]; // see SimulatedRobot for interface input vector description
        pd.direction.resize(3);
        pd.direction[0] = proximityInputs[l*5+1];
        pd.direction[1] = proximityInputs[l*5+2];
        pd.direction[2] = proximityInputs[l*5+3];
        pd.intercept   = proximityInputs[l*5+4];
      }

      ikSolver->setProximitySensorData(proxData, d);

    }


    // compute desired delta
    Vector dx = tx-x2;

    if (orientationControl) {
      // need to convert delta orientation components into angular velocities, dw (delta omega)
      // Assumption: we can use vector algebra (the vector subtraction above) on the
      //  orientation components (which are of arbitrary representation)
      // Then we use Orient::getBinv() to obtain a matrix Binv, which when multiplied by the
      //  deltas gives dw.
      Orient orient(vectorRange(x2,Range(3,x2.size())),orientRep); // pull out orientation
      Vector dorient = vectorRange(dx,Range(3,dx.size()));
      Vector domega = orient.getBinv()*Vector(dorient); // w = B(o)^-1 . do

      Vector newdx(6);
      Vector dpos( vectorRange(Vector(dx),Range(0,3)) );
      // NB: force RHS dx to be a const, otherwise we'll just assign one VectorRange to another
      vectorRange(newdx,Range(0,3)) = dpos;   // pos components
      vectorRange(newdx,Range(3,6)) = domega; // orient components
      dx = newdx;
    }
    Assert(dx.size() == orientationControl?6:3);

    // solve IK
    Matrix J( manipChain.getJacobian(q, orientationControl) );
    Vector dq( ikSolver->solve(dx, dt, x2, q, J, optMethod, optCriterion, optConstraints) );

    manipulatorInterface->setOutputs(q+dq); // set joint variables


  } catch (std::exception& e) {
    // just log errors and keep going
    Logln("Warning: controller error:" << e.what());
  }

  lastTime = time;
  return false;
}


base::array<std::pair<String,String> > IKORController::controlInterfaces() const
{
  array<std::pair<String, String> > a;

  a.push_back(std::make_pair<String,String>("manipulatorEEPosition","EndEffectorPositionControl"));
  a.push_back(std::make_pair<String,String>("manipulatorLinkPositions","LinkOriginPositions"));

  return a;
}


ref<ControlInterface> IKORController::getControlInterface(String interfaceName) throw(std::invalid_argument)
{
  if (!isConnected())
    throw std::invalid_argument(Exception("unsupported interface (not yet connected)"));

  if (interfaceName=="") interfaceName="manipulatorEEPosition";

  if (interfaceName=="manipulatorEEPosition")
    return ref<ControlInterface>(NewObj EEPositionControlInterface(ref<IKORController>(this), "manipulatorEEPosition", "EndEffectorPositionControl"));

  if (interfaceName=="manipulatorLinkPositions")
    return ref<ControlInterface>(NewObj LinkPositionsControlInterface(ref<IKORController>(this), "manipulatorLinkPositions","LinkOriginPositions"));

  throw std::invalid_argument(Exception(String("unsupported interface name:")+interfaceName));
}


void IKORController::calcEEPositionOrientation()
{
  // calculate the EE position using the current joint parameters and the forward kinematics transform (Tn)
  q.reset( manipulatorInterface->getInputs() ); // get joint variables
  Matrix T( manipChain.getForwardKinematics(q) ); // 4x4 transform to ee

  //  the position is the first 3 elements of column 4 of T
  //  the orientation is the 3x3 submatrix of T - converted into a vector
  //  in the representation 'orientRep'.
  Vector pos(3); pos = vectorRange(Vector(matrixColumn(T,3)), Range(0,3));
  Matrix rot(3,3); rot = matrixRange(T, Range(0,3), Range(0,3));
  Vector orient( Orient(rot).getVector(orientRep) );
  x.resize(3+Orient::size(orientRep));
  vectorRange(x, Range(0,3)) = pos;
  vectorRange(x, Range(3,x.size())) = orient;
}


base::array<base::String> IKORController::inputName;
base::array<base::String> IKORController::outputName;



// class EEPositionControlInterface

inline Int IKORController::EEPositionControlInterface::inputSize() const
{
  return 3+Orient(c->orientRep).size(); // always provide pos & orient, even when not controlling orientation
}

String IKORController::EEPositionControlInterface::inputName(Int i) const
{
  if (i>=inputSize())
    throw std::out_of_range(Exception("index out of range"));

  return c->inputName[i];
}

Real IKORController::EEPositionControlInterface::getInput(Int i) const
{
  if (i>=inputSize())
    throw std::out_of_range(Exception("index out of range"));

  c->calcEEPositionOrientation();
  return c->x[i];
}

const base::Vector& IKORController::EEPositionControlInterface::getInputs() const
{
  c->calcEEPositionOrientation();
  return c->x;
}

inline Int IKORController::EEPositionControlInterface::outputSize() const
{
  return (c->orientationControl?inputSize():3);
}

String IKORController::EEPositionControlInterface::outputName(Int i) const
{
  if (i>=outputSize())
    throw std::out_of_range(Exception("index out of range"));

  return c->inputName[i];
}

void IKORController::EEPositionControlInterface::setOutput(Int i, Real value)
{
  if (i>=outputSize())
    throw std::out_of_range(Exception("index out of range"));

  c->calcEEPositionOrientation();
  c->tx[i] = value;
}

void IKORController::EEPositionControlInterface::setOutputs(const Vector& values)
{
  if (values.size() != outputSize())
    throw std::out_of_range(Exception("dimension not equal to output dimension"));

  c->calcEEPositionOrientation();
  c->tx = values;
}




// class LinkPositionsControlInterface

inline Int IKORController::LinkPositionsControlInterface::inputSize() const
{
  return c->manipChain.dof()*3; // no. joints * 3 (i.e. x,y,z)
}

String IKORController::LinkPositionsControlInterface::inputName(Int i) const
{
  if (i>=inputSize())
    throw std::out_of_range(Exception("index out of range"));

  Int j = i/3;
  Int c = i - (j*3);
  String coord( (c==0)?"x":((c==1)?"y":"z") );
  return coord+base::intToString(j);
}

Real IKORController::LinkPositionsControlInterface::getInput(Int i) const
{
  if (i>=inputSize())
    throw std::out_of_range(Exception("index out of range"));

  const Vector& linkPositions( getInputs() );
  return linkPositions[i];
}


/// \todo fix for q.size() != chain.size()
const base::Vector& IKORController::LinkPositionsControlInterface::getInputs() const
{
  // get positions of all joints
  //  convert from an array<Vector> to a flat Vector
  c->q.reset( c->manipulatorInterface->getInputs() );
  array<Vector> jointPositions( c->manipChain.getLinkOrigins(c->q) );
  c->linkPositions.resize(jointPositions.size()*3);
  for(Int j=0; j<jointPositions.size(); j++) {
    c->linkPositions[3*j]   = jointPositions[j][0]; // x
    c->linkPositions[3*j+1] = jointPositions[j][1]; // y
    c->linkPositions[3*j+2] = jointPositions[j][2]; // z
  }
  return c->linkPositions;
}

inline Int IKORController::LinkPositionsControlInterface::outputSize() const
{
  return 0;
}

String IKORController::LinkPositionsControlInterface::outputName(Int i) const
{
  throw std::out_of_range(Exception("index out of range (interface has no outputs)"));
}

void IKORController::LinkPositionsControlInterface::setOutput(Int i, Real value)
{
  throw std::out_of_range(Exception("index out of range (interface has no outputs)"));
}

void IKORController::LinkPositionsControlInterface::setOutputs(const Vector& values)
{
  if (values.size() != 0)
    throw std::out_of_range(Exception("dimension not equal to output dimension (0 - no outputs)"));
}
