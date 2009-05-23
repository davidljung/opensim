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
  
  $Id: PlatformDescription.cpp 1098 2004-09-27 21:50:10Z jungd $
  $Revision: 1.6 $
  $Date: 2004-09-27 17:50:10 -0400 (Mon, 27 Sep 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <robot/PlatformDescription>

#include <base/Externalizer>
#include <base/Matrix>
#include <base/Matrix4>

#include <cstdio>

using robot::PlatformDescription;

using base::Externalizer;
using base::Expression;
using base::Matrix;
using base::Matrix4;
using base::ExpressionMatrix;
using base::dom::DOMNode;
using base::dom::DOMElement;
using robot::KinematicChain;


KinematicChain PlatformDescription::getKinematicChain(Int platformDOF, const base::Matrix4& platformTransform) const
{
  KinematicChain chain;
  if ((platformDOF == 0) || (!isMobile())) {
    chain += KinematicChain::Link(platformTransform);
  }
  else if (platformDOF == 3) {
    
    // currently we don't extract the rotation components from platformTransform 
    //  to use as static transforms in the chain, so give a warning if these
    //  components are not 0 in the initial platformTransform
    Matrix3 platformRot(platformTransform);

    if (!platformRot.equals(Matrix3().setIdentity())) { 

      Logln("warning: an initial non-zero orientation isn't well tested for the 3-dof platform KinematicChain");
    }

    chain += KinematicChain::Link(Vector3(1,0,0),platformTransform(1,4));  // first dof translates along x-axis
    chain += KinematicChain::Link(Vector3(0,1,0),platformTransform(2,4));  // second dof translates along y-axis
    Matrix4 translateZAxis; translateZAxis.setToTranslation(Vector3(0,0,platformTransform(3,4)));
    chain += KinematicChain::Link(translateZAxis);  // static translation along z-axis
    Orient r(platformRot); r.changeRepresentation(Orient::EulerZYXs);
    Real rotAboutZAxis = r[0];
    chain += KinematicChain::Link(KinematicChain::Link::Revolute, 0,0,0,rotAboutZAxis); // third dof rotates about z-axis
    r[0]=0;
    chain += KinematicChain::Link(r.getRotationMatrix3());
  }
  else if (platformDOF == 6) {
    Unimplemented;
  }
  else 
    throw std::invalid_argument(Exception("unsuported platformDOF"));
  return chain;
}



Real PlatformDescription::requiredSteeringAngle(const Vector& q, const Vector& qp) const
{
  Assert(q.size() == qp.size());
  
  Real dof=q.size();
  
  if ((dof==0) || (!isMobile())) return 0;
  
  Assert((dof == 3) || (dof == 6));
  
  if (dof == 3) { // 2D x-y positioned platform with theta rot about Z (up)
  
    if (base::equals(q,qp)) return 0; // special case 
  
    Real theta = q[2];
    Vector dP(2);  // delta change in platform origin between states q and qp
    dP = vectorRange(qp,Range(0,2)) - vectorRange(q,Range(0,2)); 
    Real dtheta = qp[2] - q[2];
    
    Vector dN(2); // get delta N (center of steeting rotation) from delta platform center P
    dN[0] = dP[0] + (L()-W())*dtheta*sin(theta);
    dN[1] = dP[1] + (L()-W())*dtheta*cos(theta);
    
    // world to platform frame (2D)
    Real a = theta;//+consts::Pi;
    Matrix T(2,2);
    T(0,0) = cos(a); T(0,1) = -sin(a);
    T(1,0) = sin(a); T(1,1) = cos(a);
    Vector pdN( T*dN );
    
    Real sa = (base::equals(pdN[1],0))?consts::Pi : atan(pdN[0]/pdN[1]);
//Debugln(DJ,"sa=" << Math::radToDeg(sa) << " pdN=" << pdN << " dN=" << dN << " theta=" << Math::radToDeg(theta));
//!!! finish    
    return sa; 
  }
  else if (dof == 6) {
    Unimplemented;
  }
 
  return 0; 
}




void PlatformDescription::externalize(base::Externalizer& e, String format, Real version)
{
  if (format == "") format = "xml";

  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" v"+base::realToString(version)+" unsupported"));

  if (e.isOutput()) {

    DOMElement* platfElem = e.createElement("platform");
    e.setElementAttribute(platfElem,"name", getName() );
    e.setElementAttribute(platfElem,"mobile", isMobile()?"true":"false" );
    e.setElementAttribute(platfElem,"holonomic", isHolonomic()?"true":"false" );
    
    DOMElement* dimElem = e.createElement("dimensions",false);
    e.appendText(dimElem, e.toString(dimensions()) );
    e.appendNode(platfElem, dimElem);
    e.appendBreak(platfElem);

    DOMElement* offsetElem = e.createElement("originoffset",false);
    e.appendText(offsetElem, e.toString(originOffset()) );
    e.appendNode(platfElem, offsetElem);
    e.appendBreak(platfElem);

    if (!isHolonomic()) { // currently, only the non-holonomic platform has any parameters
      DOMElement* paramsElem = e.createElement("params",false);
      e.setElementAttribute(paramsElem, "L", base::realToString(L()));
      e.setElementAttribute(paramsElem, "W", base::realToString(W()));
      e.appendNode(platfElem, paramsElem);
      e.appendBreak(platfElem);
    }

    
    e.appendElement(platfElem);
    
  } else { // input
//!!! link handling??
    DOMNode* context = e.context();
    
    DOMElement* platfElem = e.getFirstElement(context, "platform");

    setName( e.getDefaultedElementAttribute(platfElem, "name", "platform") );
    setMobile( e.getDefaultedElementAttribute(platfElem, "mobile", "false") == "true" );
    setHolonomic( e.getDefaultedElementAttribute(platfElem, "holonomic", "false") == "true" );

    DOMElement* dimElem = e.getFirstChildElement(platfElem, "dimensions");
    String dimText = e.getContainedText(dimElem);
    setDimensions( e.toVector3(dimText) );

    DOMElement* offsetElem = e.getFirstChildElement(platfElem, "originoffset",false);
    if (offsetElem) {
      String offsetText = e.getContainedText(offsetElem);
      setOriginOffset( e.toVector3(offsetText) );
    }
    else
      setOriginOffset( Vector3() );

    DOMElement* paramsElem = e.getFirstChildElement(platfElem, "params",false);
    if (paramsElem) {
      setL( base::stringToReal(e.getDefaultedElementAttribute(paramsElem, "L", "0.0")) );
      setW( base::stringToReal(e.getDefaultedElementAttribute(paramsElem, "W", "0.0")) );
    }
    
    
    e.removeElement(platfElem);

  }
}
