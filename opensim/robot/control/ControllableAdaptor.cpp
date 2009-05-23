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
  
  $Id: ControllableAdaptor.cpp 1037 2004-02-11 20:50:18Z jungd $
  $Revision: 1.6 $
  $Date: 2004-02-11 15:50:18 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <robot/control/ControllableAdaptor>

using robot::control::ControllableAdaptor;
using robot::ControlInterface;


ControllableAdaptor::ControllableAdaptor(AdaptorType type, ref<Controllable> controllable, 
					 const String& interfaceName, const String& adaptedInterfaceName,
					 const String& adaptedInterfaceType)
  : type(type), rangesSet(false), stridesSet(false), controllable(controllable), 
    newInterfaceName(adaptedInterfaceName), newInterfaceType(adaptedInterfaceType)
{
  adaptedInterface = controllable->getControlInterface(interfaceName);
  if (newInterfaceName=="") newInterfaceName=adaptedInterface->getName();
  if (newInterfaceType=="") newInterfaceType=adaptedInterface->getType();
  // create an instance of the ControlInterface.  By storing the ref and passing a ref to ourself
  //  we create a cyclic reference - which we break in onUnreference() when appropriate
  adaptorInterface = ref<AdaptorControlInterface>(NewObj AdaptorControlInterface(ref<ControllableAdaptor>(this)));
  enableOnUnreferenceCall(true);

  if (type == PassThrough) {
    adaptorInterface->inputs.resize(adaptedInterface->inputSize());
    adaptorInterface->outputsSize = adaptedInterface->outputSize();
  }
}


ControllableAdaptor::ControllableAdaptor(const ControllableAdaptor& c)
  : type(c.type), rangesSet(c.rangesSet), 
    inputStart(c.inputStart), outputStart(c.outputStart),
    inputEnd(c.inputEnd), outputEnd(c.outputEnd),
    stridesSet(c.stridesSet), inputStride(c.inputStride), outputStride(c.outputStride),
    controllable(c.controllable), adaptedInterface(c.adaptedInterface), adaptorInterface(c.adaptorInterface),
    newInterfaceName(c.newInterfaceName)
{
  // NB: this shares the adaptorInterface, so if this class is ever modified to be mutable after
  //  construction, the adaptorInterface will been a NewObj
}


void ControllableAdaptor::onUnreference() const
{
  if ((referenceCount()==1) && (adaptorInterface->referenceCount()==1)) {
    // if the only references are to each other, then break the cyclic link
    Release(adaptorInterface);
  }
}




void ControllableAdaptor::setRanges(Int inputStart, Int outputStart, SInt inputEnd, SInt outputEnd)
{
  if ((inputStart >= adaptedInterface->inputSize()) || (outputStart >= adaptedInterface->outputSize()))
    throw std::out_of_range(Exception("inputStart or outputStart out of range"));

  this->inputStart = inputStart;
  this->outputStart = outputStart;
  this->inputEnd = inputEnd;
  this->outputEnd = outputEnd;
  if (inputEnd == End)
    this->inputEnd = adaptedInterface->inputSize()-1;
  if (outputEnd == End)
    this->outputEnd = adaptedInterface->inputSize()-1;
  rangesSet = true;
  adaptorInterface->inputs.resize(this->inputEnd-inputStart+1);
  adaptorInterface->outputsSize = this->outputEnd-outputStart+1;
}

void ControllableAdaptor::setStrides(Int inputStart, Int outputStart, Int inputStride, Int outputStride)
{
  this->inputStart = inputStart;
  this->outputStart = outputStart;
  this->inputStride = inputStride;
  this->outputStride = outputStride;
  stridesSet = true;
  adaptorInterface->inputs.resize( (adaptedInterface->inputSize()-inputStart)/inputStride );
  adaptorInterface->outputsSize = (adaptedInterface->outputSize()-outputStart)/outputStride;
}




void ControllableAdaptor::setControlInterface(ref<ControlInterface> controlInterface)
{
  // ignore (constructor uses interfaceName of Controllable)
}

bool ControllableAdaptor::iterate(const base::Time& time)
{
  return false;
}


ref<ControlInterface> ControllableAdaptor::getControlInterface(String interfaceName) throw(std::invalid_argument)
{
  if ( ((type==Range)&&!rangesSet) || ((type==Stride)&&!stridesSet))
    throw std::invalid_argument(Exception(String("unknown interface ")+interfaceName+" (adaptor parameters not set)"));

  if ((interfaceName != "") && (interfaceName != adaptorInterface->getName()))
    throw std::invalid_argument(Exception(String("unknown interface ")+interfaceName));

  return adaptorInterface;
}



inline Int ControllableAdaptor::adaptInputIndex(Int i) const
{
  Int ai;
  switch (type) {
  case PassThrough: ai=i; break;
  case Range: ai = i + inputStart; break;
  case Stride: ai = inputStart + i*inputStride; break;
  default:
    throw std::runtime_error(Exception("AdaptorType not handled"));    
  }

  if (ai >= adaptedInterface->inputSize()) indexOutOfRange();
  return ai;
}

inline Int ControllableAdaptor::adaptOutputIndex(Int i) const
{
  Int ai = 0;
  switch (type) {
  case PassThrough: ai=i; break;
  case Range: ai = i + outputStart; break;
  case Stride: ai = outputStart + i*outputStride; break;
  default:
    throw std::runtime_error(Exception("AdaptorType not handled"));    
  }

  if (ai >= adaptedInterface->outputSize()) indexOutOfRange();
  return ai;
}




// class AdaptorControlInterface

base::Int ControllableAdaptor::AdaptorControlInterface::inputSize() const
{
  return inputs.size();
}

base::String ControllableAdaptor::AdaptorControlInterface::inputName(Int i) const
{
  return c->adaptedInterface->inputName(c->adaptInputIndex(i));
}

inline base::Real ControllableAdaptor::AdaptorControlInterface::getInput(Int i) const
{
  return c->adaptedInterface->getInput(c->adaptInputIndex(i));
}

const base::Vector& ControllableAdaptor::AdaptorControlInterface::getInputs() const
{
  ref<ControlInterface> adaptedInterface(c->adaptedInterface);
  for(Int i=0; i<inputs.size(); i++)
    inputs[i] = adaptedInterface->getInput(c->adaptInputIndex(i));

  return inputs;
}

base::Int ControllableAdaptor::AdaptorControlInterface::outputSize() const
{
  return outputsSize;
}

base::String ControllableAdaptor::AdaptorControlInterface::outputName(Int i) const
{
  return c->adaptedInterface->outputName(c->adaptOutputIndex(i));
}

inline void ControllableAdaptor::AdaptorControlInterface::setOutput(Int i, Real value) 
{
  c->adaptedInterface->setOutput(c->adaptOutputIndex(i),value);
}

void ControllableAdaptor::AdaptorControlInterface::setOutputs(const Vector& values)
{
  ref<ControlInterface> adaptedInterface(c->adaptedInterface);
  for(Int i=0; i<values.size(); i++)
    adaptedInterface->setOutput(c->adaptOutputIndex(i), values[i]);
}

 

