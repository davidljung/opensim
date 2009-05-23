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
  
  $Id: ManipulatorPIDPositionController.cpp 1037 2004-02-11 20:50:18Z jungd $
  $Revision: 1.9 $
  $Date: 2004-02-11 15:50:18 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <robot/control/ManipulatorPIDPositionController>

using robot::control::ManipulatorPIDPositionController;
using robot::ControlInterface;

ManipulatorPIDPositionController::ManipulatorPIDPositionController(const robot::KinematicChain& chain)
  : chain(chain), Kp(10), Ki(0.1), Kd(0)
{
  //!!! NB: creates a cyclic reference!
  positionInterface = ref<PositionInterface>(NewObj PositionInterface(ref<ManipulatorPIDPositionController>(this)));
}



void ManipulatorPIDPositionController::setControlInterface(ref<ControlInterface> controlInterface)
{
  if (controlInterface->getType()=="JointVelocityControl") {
    manipInterface = controlInterface;
    Int inputSize = manipInterface->inputSize();
    lastInputSize = inputSize;
    if (inputSize > 0) {
      R.reset(zeroVector(inputSize));
      pe.reset(zeroVector(inputSize));
      s.reset(zeroVector(inputSize));
    }
  }
  else {
    Logln("Ignoring useless ControlInterface name:" << controlInterface->getName() << " type:" << controlInterface->getType());
  }
}

bool ManipulatorPIDPositionController::isConnected() const
{
  return (manipInterface!=0);
}

bool ManipulatorPIDPositionController::iterate(const base::Time& time)
{
  if (!isConnected()) return false;

  Int inputSize = manipInterface->inputSize();
  if (inputSize != lastInputSize) {
    pe.reset(zeroVector(inputSize));
    s.reset(zeroVector(inputSize));
    lastInputSize = inputSize;
  }

  Int outputSize = manipInterface->outputSize();
  if (outputSize != lastOutputSize) {
    R.reset(zeroVector(inputSize));
    lastOutputSize = outputSize;
  }


  for(Int i=0; i<inputSize; i++) {

    // perform discrete PID step
    Real Y = manipInterface->getInput(i); // actual position
    Real e;
    // if the joint is revolute and has no joint limits,
    //  take the shortest angular path, not the long way around
    bool angWrap = false;
    if (chain.linkOfVariable(i).type() == KinematicChain::Link::Revolute) 
      if (chain.variableMaxLimit(i) > consts::Pi)
	if (chain.variableMinLimit(i) < consts::Pi)
	  angWrap=true;
    if (!angWrap)
      e = R[i]-Y; // error = reference - actual 
    else
      e = Math::angleDifference(R[i],Y); // error = reference - actual 

    s[i] += e;
    Real u = Kp*e + Ki*s[i] + Kd*(e - pe[i]); // PID
    manipInterface->setOutput(i, u);

    if (i==inputSize-1) {
      //Debugcln(DJ,"i::" << i << "  R=" << R[i] << " Y=" << Y << " e=" << e << " s[i]=" << s[i] << " e-pe[i]=" << (e-pe[i]) << " Kp*e=" << Kp*e << " Ki*s[i]=" << Ki*s[i] << " u=" << u);
    }
    pe[i] = e;
  }

  return false;
}


base::ref<ControlInterface> ManipulatorPIDPositionController::getControlInterface(String interfaceName) throw(std::invalid_argument)
{
  if (!manipInterface)
    throw std::invalid_argument(Exception(String("unknown interface ")+interfaceName+" (not connected)"));

  if (interfaceName=="") interfaceName = positionInterface->getName();

  if (interfaceName == positionInterface->getName())
    return positionInterface;

  throw std::invalid_argument(Exception(String("unknown interface name:")+interfaceName));
}




Int ManipulatorPIDPositionController::PositionInterface::inputSize() const
{
  return c->manipInterface->inputSize();
}


/// \todo implement properly (or is it OK?)
base::String ManipulatorPIDPositionController::PositionInterface::inputName(Int i) const
{
  return c->manipInterface->inputName(i);
}


Real ManipulatorPIDPositionController::PositionInterface::getInput(Int i) const 
{
  return c->manipInterface->getInput(i);
}


const base::Vector& ManipulatorPIDPositionController::PositionInterface::getInputs() const 
{
  return c->manipInterface->getInputs();
}


Int ManipulatorPIDPositionController::PositionInterface::outputSize() const
{
  return c->manipInterface->outputSize();
}


/// \todo implement properly
base::String ManipulatorPIDPositionController::PositionInterface::outputName(Int i) const
{
  return c->manipInterface->outputName(i); //!!! fixme to subst force/torque;vel/ang.vel names for angle/dist names
}


void ManipulatorPIDPositionController::PositionInterface::setOutput(Int i, Real value) 
{
  Int outputSize = c->manipInterface->outputSize();
  if (i >= outputSize)
    throw std::out_of_range(Exception("index out of range"));

  if (c->lastOutputSize != outputSize) {
    c->R.reset(zeroVector(outputSize));
    c->lastOutputSize = outputSize;
  }

  c->R[i] = value;
}


void ManipulatorPIDPositionController::PositionInterface::setOutputs(const Vector& values) 
{
  Int outputSize = c->manipInterface->outputSize();
  if (values.size() != outputSize)
    throw std::invalid_argument(Exception("values Vector has wrong dimension"));

  if (c->lastOutputSize != outputSize) {
    c->R.reset(zeroVector(outputSize));
    c->lastOutputSize = outputSize;
  }

  c->R = values;
}


