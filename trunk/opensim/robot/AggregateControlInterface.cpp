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
  
  $Id: AggregateControlInterface.cpp 1039 2004-02-11 20:50:52Z jungd $
  $Revision: 1.1 $
  $Date: 2004-02-11 15:50:52 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <robot/AggregateControlInterface>

using robot::AggregateControlInterface;



AggregateControlInterface::AggregateControlInterface(const String& name, const String& type,
                                                     array<ref<ControlInterface> > interfaces)
 : ControlInterface(name,type), interfaces(interfaces)
{
  if (interfaces.size() < 1)
    throw std::invalid_argument(Exception("must supply at least one ControlInterface"));
  
  inputOffsets.resize(interfaces.size()+1);
  outputOffsets.resize(interfaces.size()+1);
}
                            
                            
AggregateControlInterface::AggregateControlInterface(const String& name, const String& type,
                                                     ref<ControlInterface> interface1, 
                                                     ref<ControlInterface> interface2)
  : ControlInterface(name,type)
{
  Assert(interface1);
  Assert(interface2);
  
  interfaces.resize(2);
  interfaces[0] = interface1;
  interfaces[1] = interface2;
  
  inputOffsets.resize(3);
  outputOffsets.resize(3);
}


AggregateControlInterface::AggregateControlInterface(const AggregateControlInterface& aci)
  : ControlInterface(aci), interfaces(aci.interfaces)
{
  inputOffsets.resize(interfaces.size()+1);
  outputOffsets.resize(interfaces.size()+1);
}



inline void AggregateControlInterface::inputIndex(Int aindex, Int& interfaceIndex, Int& index) const
{
  interfaceIndex=1;
  while (inputOffsets[interfaceIndex] < aindex) interfaceIndex++;
  interfaceIndex--;
  index = aindex - inputOffsets[interfaceIndex];
}


inline void AggregateControlInterface::outputIndex(Int aindex, Int& interfaceIndex, Int& index) const
{
  interfaceIndex=1;
  while (outputOffsets[interfaceIndex] < aindex) interfaceIndex++;
  interfaceIndex--;
  index = aindex - outputOffsets[interfaceIndex];
}



String AggregateControlInterface::inputName(Int i) const
{
  recomputeInputSize();
  Assert( i < numInputs);
  Int interfaceIndex, index;
  inputIndex(i, interfaceIndex, index);
  return interfaces[interfaceIndex]->inputName(index);
}


Real AggregateControlInterface::getInput(Int i) const
{
  recomputeInputSize();
  Assert( i < numInputs);
  Int interfaceIndex, index;
  inputIndex(i, interfaceIndex, index);
  return interfaces[interfaceIndex]->getInput(index);
}


const base::Vector& AggregateControlInterface::getInputs() const 
{
  recomputeInputSize();
  lastInputs.resize(numInputs);
  
  Int interfaceIndex=0;
  Int interfaceOffset=0;
  ref<ControlInterface> interface(interfaces[0]);
  for(Int i=0; i<numInputs; i++) {
    if (i == inputOffsets[interfaceIndex+1]) {
      interfaceIndex++;
      interface = interfaces[interfaceIndex];
      interfaceOffset = inputOffsets[interfaceIndex];
    }
    lastInputs[i] = interface->getInput( i - interfaceOffset);
  }
  return lastInputs;
}


String AggregateControlInterface::outputName(Int i) const
{
  recomputeOutputSize();
  Assert( i < numOutputs);
  Int interfaceIndex, index;
  outputIndex(i, interfaceIndex, index);
  return interfaces[interfaceIndex]->outputName(index);
}


void AggregateControlInterface::setOutput(Int i, Real value)
{
  recomputeOutputSize();
  Assert( i < numOutputs);
  Int interfaceIndex, index;
  outputIndex(i, interfaceIndex, index);
  interfaces[interfaceIndex]->setOutput(index, value );
}


void AggregateControlInterface::setOutputs(const Vector& values) 
{
  recomputeOutputSize();
  Assert(values.size() == numOutputs);
  
  Int interfaceIndex=0;
  Int interfaceOffset=0;
  ref<ControlInterface> interface(interfaces[0]);
  for(Int i=0; i<numOutputs; i++) {
    if (i == outputOffsets[interfaceIndex+1]) {
      interfaceIndex++;
      interface = interfaces[interfaceIndex];
      interfaceOffset = outputOffsets[interfaceIndex];
    }
    interface->setOutput( i - interfaceOffset, values[i]);
  }
}


void AggregateControlInterface::recomputeInputSize() const
{
  numInputs=0;
  for(Int i=0; i<interfaces.size(); i++) {
    inputOffsets[i] = numInputs;
    numInputs += interfaces[i]->inputSize();
  }
  inputOffsets[interfaces.size()] = numInputs;
}


void AggregateControlInterface::recomputeOutputSize() const
{
  numOutputs=0;
  for(Int i=0; i<interfaces.size(); i++) {
    outputOffsets[i] = numOutputs;
    numOutputs += interfaces[i]->outputSize();
  }
  outputOffsets[interfaces.size()] = numOutputs;
}


