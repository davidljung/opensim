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
  
  $Id: KinematicChain.cpp 1039 2004-02-11 20:50:52Z jungd $
  $Revision: 1.9 $
  $Date: 2004-02-11 15:50:52 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <robot/KinematicChain>

#include <base/MD5>
#include <base/Externalizer>
#include <base/externalization_error>
#include <robot/JFKengine>

#include <cstdio>

using robot::KinematicChain;

using base::Byte;
using base::Matrix;
using base::Vector;
using base::ExpressionMatrix;
using base::Externalizer;
using base::externalization_error;
using base::dom::DOMNode;
using base::dom::DOMElement;
using base::MD5;
using robot::KinematicEvaluator;
using robot::JFKengine;



const Real KinematicChain::unboundedMinAngleLimit = -consts::Pi - consts::epsilon;
const Real KinematicChain::unboundedMaxAngleLimit = +consts::Pi + consts::epsilon;
const Real KinematicChain::unboundedMinDistLimit  = consts::maxReal;
const Real KinematicChain::unboundedMaxDistLimit  = -consts::maxReal;

  
  
KinematicChain::Link& KinematicChain::Link::operator=(const KinematicChain::Link& l)
{
  linkType=l.linkType;
  switch (linkType) {
    case Translating:
      direction=l.direction;
      t=l.t;
      break;
    case FixedTransform:
      transform=l.transform;
      break;
    case Prismatic: 
    case Revolute:
      alpha=l.alpha; a=l.a; d=l.d; theta=l.theta;
      break;
    default: Assertm(false,"known joint type");
  }
  jminLimit=l.jminLimit; jmaxLimit=l.jmaxLimit;
  jminAccel=l.jminAccel; jmaxAccel=l.jmaxAccel;
  active=l.active;
  dirtyHash(); 
  return *this;
}


bool KinematicChain::Link::operator==(const Link& l) const
{
  if (linkType != l.linkType) return false;
  if (this == &l) return true;
  
  switch (linkType) {
    case Prismatic:
    case Revolute:
      return    Math::equals(alpha,l.alpha) && Math::equals(theta,l.theta) 
             && Math::equals(d,l.d) && Math::equals(a,l.a) && (active==l.active);
    case Translating:
      return direction.equals(l.direction) && (active==l.active);
    case FixedTransform:
      return transform.equals(l.transform);
    default:
      throw std::runtime_error(Exception("unhandled KinematicChain::Link type"));
  }
  
  return false;
}


base::Matrix KinematicChain::Link::kinematicTransform(const base::Vector& q) const
{
  Assert(q.size() == dof());
  Matrix A(4,4);
  
  if (isDHType()) {
    Real d = ((linkType == Prismatic) && active)?q[0]:this->d;
    Real theta = ((linkType == Revolute) && active)?q[0]:this->theta;
    
    // Compute the transformation that transforms the coord. frame of
    //  this link into the frame of the previous one
    // (see a ref on Denavit-Hartenberg parameters - e.g. "Robot Manipulators" by Richard P. Paul.)
    Real sinAlpha = Math::sin(alpha);
    Real cosAlpha = Math::cos(alpha);
    Real sinTheta = Math::sin(theta);
    Real cosTheta = Math::cos(theta);
    
    A(0,0) = cosTheta; A(0,1) = -sinTheta*cosAlpha; A(0,2) = sinTheta*sinAlpha;  A(0,3) = a*cosTheta; 
    A(1,0) = sinTheta; A(1,1) = cosTheta*cosAlpha;  A(1,2) = -cosTheta*sinAlpha; A(1,3) = a*sinTheta;
    A(2,0) = 0;        A(2,1) = sinAlpha;           A(2,2) = cosAlpha;           A(2,3) = d;
    A(3,0) = 0;        A(3,1) = 0;                  A(3,2) = 0;                  A(3,3) = 1;
  }
  else if (linkType == Translating) {
    A = base::identityMatrix(4,4);
    Assert(direction.length() > 0);
    Vector3 dir(direction);
    dir.normalize();
    Real v = active?q[0]:t;
    A(0,3) = dir.x * v;
    A(1,3) = dir.y * v;
    A(2,3) = dir.z * v;
  }
  else if (linkType == FixedTransform) {
    A = base::fromMatrix4(transform);
  }
  else
    throw std::runtime_error(Exception("unhandled KinematicChain::Link type"));
  
  return A;
}


base::array<base::Byte> KinematicChain::Link::hashCode() const
{
  if (!hashDirty) return hash;
  
  // create an MD5 hash over relevant link parameters
  MD5 md5;
  md5.update( (unsigned char *)(&linkType), sizeof(LinkType) );
  md5.update( (unsigned char *)(&active), sizeof(bool) );
  
  switch (linkType) {
    case Translating:
      md5.update( (unsigned char *)(&direction.x), sizeof(Real) );
      md5.update( (unsigned char *)(&direction.y), sizeof(Real) );
      md5.update( (unsigned char *)(&direction.z), sizeof(Real) );
      md5.update( (unsigned char *)(&t), sizeof(Real) );
      break;
    case FixedTransform:
      for (Int r=1; r<=4; r++)
        for(Int c=1; c<=4; c++) {
          Real e = transform(r,c);
          md5.update( (unsigned char *)(&e), sizeof(Real) );
        }
      break;
    case Prismatic: 
    case Revolute:
      md5.update( (unsigned char *)(&alpha), sizeof(Real) );
      md5.update( (unsigned char *)(&a), sizeof(Real) );
      md5.update( (unsigned char *)(&d), sizeof(Real) );
      md5.update( (unsigned char *)(&theta), sizeof(Real) );
      break;
    default: Assertm(false,"known joint type");
  }
  
  md5.finalize();
  hash.resize(16); // MD5 implies 16 byte hash
  unsigned char *digest = md5.raw_digest();
  for(Int b=0; b<16; b++) hash[b] = digest[b];
  
  return hash;
}









bool KinematicChain::isDHType() const
{
  for(Int i=0; i<links.size(); i++)
    if (!links[i].isDHType()) return false;
  return true;
}

  
KinematicChain KinematicChain::subChain(Int first, Int count) const
{
  Assert( first < size() );
  Assert( first+count <= size() );
  LinkArray la(count);
  for(Int i=0; i<count; i++) la[i]=links[first+i];
  return KinematicChain(la);
}

    
KinematicChain& KinematicChain::operator+=(const KinematicChain& kc)
{
  if (kc.size() > 0) {
    for(Int i=0; i<kc.size(); i++)
      links.push_back(kc[i]);
    computeVariables(); 
  }
  dirtyHash();
  return *this;
}


KinematicChain& KinematicChain::insert(Int pos, const KinematicChain& kc)
{
  Assert( pos <= size() );
  KinematicChain newkc( subChain(0,pos) );
  newkc += kc;
  newkc += subChain(pos,size()-pos);
  *this = newkc;
  computeVariables(); 
  dirtyHash();
  return *this;
}


KinematicChain& KinematicChain::insert(Int pos, const Link& l)
{
  Assert( pos <= size() );
  KinematicChain newkc( subChain(0,pos) );
  newkc += l;
  newkc += subChain(pos,size()-pos);
  *this = newkc;
  computeVariables(); 
  dirtyHash();
  return *this;
}  


KinematicChain& KinematicChain::erase(Int pos)
{
  Assert( pos < size() );
  KinematicChain newkc( size()-1);
  for(Int i=0; i<size(); i++)
    if (i!=pos) newkc.push_back(links[i]);
  *this = newkc;
  computeVariables(); 
  dirtyHash();
  return *this;
}


KinematicChain& KinematicChain::erase(Int first, Int last)
{
  Assert( first <= last );
  Assert( first < size() );
  Assert( last <= size() );
  KinematicChain newkc( size()-(last-first) );
  for(Int i=0; i<size(); i++)
    if (!((i>=first) && (i<last))) newkc.push_back(links[i]);
  *this = newkc;
  computeVariables(); 
  dirtyHash();
  return *this;
}



void KinematicChain::computeVariables()
{
  variables.clear();
  for(Int l=0; l<size(); l++) {
    const Link& link(links[l]);

    for(Int d=0; d<link.dof(); d++)
      variables.push_back( std::make_pair<Int,Int>(l,d) );
  } 
}





bool KinematicChain::KinematicEvaluatorInitialized = false;
ref<KinematicEvaluator> KinematicChain::defaultKinematicEvaluator;


void KinematicChain::initKinematicEvaluator() 
{
  if (!KinematicEvaluatorInitialized) {
    KinematicEvaluatorInitialized = true;
    defaultKinematicEvaluator = ref<JFKengine>(NewObj JFKengine()); 
  }
  kinematicEvaluator = defaultKinematicEvaluator;
}



Matrix KinematicChain::getForwardKinematics(const Vector& q) const
{
  return kinematicEvaluator->getForwardKinematics(*this,q);
}

  
Matrix KinematicChain::getJacobian(const Vector& q, bool includeOrientation) const
{
  return kinematicEvaluator->getJacobian(*this, q, includeOrientation);
}

array<Vector> KinematicChain::getJointOrigins(const Vector& q) const
{
  return kinematicEvaluator->getJointOrigins(*this, q);
}


array<Vector> KinematicChain::getLinkOrigins(const Vector& q) const
{
  return kinematicEvaluator->getLinkOrigins(*this, q);
}




Vector KinematicChain::transform(Int fromLink, Int toLink, const Vector& v, const Vector& q) const
{
  if (fromLink == toLink) return v;
  
  Int from = fromLink;
  Int to = toLink;
  
  if (fromLink < toLink) base::swap(from, to);
  
  KinematicChain subTo_From(subChain(toLink, fromLink-toLink)); // chain from 'to' to 'from'
  KinematicChain sub0_To(subChain(0, toLink)); // chain from 0 to 'to'
  Vector subq( vectorRange(q, Range(sub0_To.dof(), subTo_From.dof())) );
  
  Matrix4 T( base::toMatrix4(subTo_From.getForwardKinematics(subq)) );
    
  if (fromLink < toLink) T.invert();
  
  return base::fromVector3(T*base::toVector3(v));
}

  
Vector KinematicChain::transform(const String& fromFrame, const String& toFrame, const Vector& v, const Vector& q) const
{
  const SInt NotFound = -2;
  const SInt InitialFrame = -1;
  
  // find link frames
  SInt from = NotFound;
  if (fromFrame == initialFrame)
    from=InitialFrame;
  else {
    for (Int fl=0; fl<size(); fl++) 
      if (links[fl].frameName == fromFrame) {
        from=fl;
        break;
      }
  }
    
  
  SInt to = NotFound;
  if (toFrame == initialFrame)
    to=InitialFrame;
  else {
    for (Int tl=0; tl<size(); tl++)
      if (links[tl].frameName == toFrame) {
        to=tl;
        break;
      }
  }
    
  if (from==NotFound)
    throw std::invalid_argument(Exception("unknown frame name:"+fromFrame));
  if (to==NotFound)
    throw std::invalid_argument(Exception("unknown frame name:"+toFrame));

  if (from == to) return v; // nothing to do
  
  
  if ((from != InitialFrame) && (to != InitialFrame))
    return transform(from, to, v, q);

  // special case when one frame is the initial frame
  Unimplemented;
  
}

  
  

void KinematicChain::externalize(base::Externalizer& e, String format, Real version)
{
  if (format == "") format = "xml";

  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" v"+base::realToString(version)+" unsupported"));

  if (e.isOutput()) {

    DOMElement* chainElem = e.createElement("kinematicchain");
    bool allDHLinks = isDHType();
    e.setElementAttribute(chainElem,"type",allDHLinks?"DH":"mixed" );

    e.appendComment(chainElem,"Parameters describing the serial Kinematic Chain:");
    e.appendComment(chainElem,"  revolute , {alpha} ,     {a} ,     {d} , {theta} ,{minlimit},{maxlimit} ");
    e.appendComment(chainElem,"  prismatic, {alpha} ,     {a} ,     {d} , {theta} ,{minlimit},{maxlimit} ");
    if (!allDHLinks) {
      e.appendComment(chainElem,"  translating,         {dir.x} , {dir.y} , {dir.z} ,{minlimit},{maxlimit} ");
      // transform here!!!
    }

    e.pushContext(chainElem);
    for(Int l=0; l<links.size(); l++) 
      links[l].externalize(e, format, version);
    e.popContext();
    
    e.appendElement(chainElem);

  } else { // input

    DOMNode* context = e.context();
    
    DOMElement* chainElem = e.getFirstElement(context, "kinematicchain");

    String units = e.getDefaultedElementAttribute(chainElem, "units", "meters");
    if ((units != "meters") && (units != "inches"))
      throw externalization_error(Exception("unknown/unsupported length unit:"+units));
    
    String type = e.getDefaultedElementAttribute(chainElem, "type", "mixed");
    if ( (type == "mixed") || (type == "DH")) {

      links.clear();
      e.pushContext(chainElem);
      
      while (e.getFirstChildElement(chainElem, "link",false) != 0) { // more joints?
	Link l;
	l.externalize(e, format, version);
	if ( (!l.isDHType()) && (type == "DH"))
	  throw externalization_error(Exception("a non D-H type joint was specified for a kinematic chain of type 'DH' (try type 'mixed')"));
        
        // convert all length measures to meters
        if (units != "meters") {
          Real scale = 1.0;
          if (units == "inches") scale = 0.0254;
          
          if (l.isDHType()) {
            l.setA( l.getA()*scale );
            l.setD( l.getD()*scale );
          }
          else if (l.type() == KinematicChain::Link::FixedTransform) {
            Logln("Warning: conversion from "+units+" to meters unsupported for fixed transform links");
          }

        }
        
	links.push_back(l);
      }
      
      e.popContext();
    
      if (links.size() < 1)
	throw externalization_error(Exception("a KinematicChain must contain at least one link (<link>...</link>)."));

    }
    else
      throw externalization_error(Exception(String("unknown KinematicChain type ")+type));
      
    computeVariables();
    dirtyHash(); 
  }
}



void KinematicChain::Link::externalize(base::Externalizer& e, String format, Real version)
{
  if (format == "") format = "xml";
  
  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" v"+base::realToString(version)+" unsupported"));
  
  if (e.isOutput()) {
    DOMElement* linkElem = e.createElement("link",false);
    char buf[1024];

    if (type() == Revolute) {
      if (  (Math::equals(jminLimit, KinematicChain::unboundedMinAngleLimit))
          ||(Math::equals(jmaxLimit, KinematicChain::unboundedMaxAngleLimit)) )
        sprintf(buf, "revolute , %8.4f, %8.4f, %8.4f, %8.4f,    -inf ,     +inf ",
	        Math::radToDeg(alpha), a, d, Math::radToDeg(theta));
      else
        sprintf(buf, "revolute , %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f ",
	        Math::radToDeg(alpha), a, d, Math::radToDeg(theta),
	        Math::radToDeg(jminLimit), Math::radToDeg(jmaxLimit));
    }
    else if (type() == Prismatic) {
      char minBuf[32], maxBuf[32];
      if (Math::equals(jminLimit, KinematicChain::unboundedMinDistLimit,999))
        sprintf(minBuf, "%8.4f", jminLimit);
      else
        sprintf(minBuf, "%s", "-inf");

      if (Math::equals(jmaxLimit, KinematicChain::unboundedMaxDistLimit,999))
        sprintf(maxBuf, "%8.4f", jmaxLimit);
      else
        sprintf(maxBuf, "%s", "+inf");

      sprintf(buf, "prismatic, %8.4f, %8.4f, %8.4f, %8.4f, %s, %s ",
              Math::radToDeg(alpha), a, d, Math::radToDeg(theta),
	      minBuf, maxBuf);
    }
    else if (type() == Translating) {
      char minBuf[32], maxBuf[32];
      if (Math::equals(jminLimit, KinematicChain::unboundedMinDistLimit,999))
        sprintf(minBuf, "%8.4f", jminLimit);
      else
        sprintf(minBuf, "%s", "-inf");

      if (Math::equals(jmaxLimit, KinematicChain::unboundedMaxDistLimit,999))
        sprintf(maxBuf, "%8.4f", jmaxLimit);
      else
        sprintf(maxBuf, "%s", "+inf");

      sprintf(buf, "translating,         %8.4f, %8.4f, %8.4f, %s, %s ",
              direction.x, direction.y, direction.z,
	      minBuf, maxBuf);
    }
    else if (type() == FixedTransform) {
      Unimplemented; ///!!!
    } 
    else
      throw externalization_error(Exception(String("unknown link type")));
      
    e.appendText(linkElem,String(buf));
    e.appendElement(linkElem);
    e.appendBreak();
  }
  else { // input
    DOMNode* context = e.context();
    
    DOMElement* linkElem = e.getFirstChildElement(context, "link");

    String linkText = e.getContainedText(linkElem);
    array<String> elts = e.splitAtDelimiter(linkText,',');
    elts[0] = e.removeChar(elts[0],' ');
    if (elts[0] == "revolute")
      linkType = Revolute;
    else if (elts[0] == "prismatic")
      linkType = Prismatic;
    else if (elts[0] == "translating")
      linkType = Translating;
    else if (elts[0] == "transform")
      linkType = FixedTransform;
    else
      throw externalization_error(Exception(String("link type should be either 'revolute', 'prismatic', 'translating' or 'transform'; not '")+elts[0]+"'."));

    Vector vals( e.stringsToReals(elts) );
    jminLimit=jmaxLimit = 0.0;
    if (isDHType()) {
      alpha = Math::degToRad( vals[1] );
      a = vals[2];
      d = vals[3];
      theta = Math::degToRad( vals[4] );
      if (type() == Revolute) {
	if (vals.size() > 6) {
	  jminLimit = Math::degToRad( vals[5] );
	  jmaxLimit = Math::degToRad( vals[6] );
	}
	if (Math::equals(jminLimit,0) && Math::equals(jmaxLimit,0)) { // unbounded
	  jminLimit = KinematicChain::unboundedMinAngleLimit;
	  jmaxLimit = KinematicChain::unboundedMaxAngleLimit;
	}
	
      }
      else {
	if (vals.size() > 6) {
	  jminLimit = vals[5];
	  jmaxLimit = vals[6];
	}
	if (Math::equals(jminLimit,0) && Math::equals(jmaxLimit,0)) { // unbounded
	  jminLimit = unboundedMinDistLimit;
	  jmaxLimit = unboundedMaxDistLimit;
	}
      }
    }
    else if (type() == Translating) {
      direction.x = vals[1];
      direction.y = vals[2];
      direction.z = vals[3];
      if (vals.size() > 5) {
	jminLimit = vals[4];
	jmaxLimit = vals[5];
      }
      if (Math::equals(jminLimit,0) && Math::equals(jmaxLimit,0)) { // unbounded
        jminLimit = unboundedMinDistLimit;
        jmaxLimit = unboundedMaxDistLimit;
      }
    }
    else if (type() == FixedTransform) {
      Unimplemented; ///!!!
    }
    else
      Assertm(false,"link type not implemented");

    e.removeElement(linkElem);
    
    dirtyHash(); 
  }

}


base::array<base::Byte> KinematicChain::hashCode() const
{
  if (!hashDirty) return hash;
  
  // create an MD5 hash over the link size, dof and each link's hashCode()
  MD5 md5;
  Int chainSize = size();
  md5.update( (unsigned char *)(&chainSize), sizeof(Int));
  Int chainDOF = dof();
  md5.update( (unsigned char *)(&chainDOF), sizeof(Int));
  for(Int l=0; l<size(); l++) {
    array<Byte> hashCode( links[l].hashCode() );
    md5.update( (unsigned char *)(&(hashCode[0])), hashCode.size()); // assumes array stores Bytes contigously
  }
  md5.finalize();
  
  hash.resize(16); // MD5 implies 16 byte hash
  unsigned char *digest = md5.raw_digest();
  for(Int b=0; b<16; b++) hash[b] = digest[b];
  
  return hash;
}



std::ostream& robot::operator<<(std::ostream& out, const robot::KinematicChain::Link& l) // Output
{

  switch(l.type()){
  case KinematicChain::Link::Revolute:  
    if (l.isActive()) 
      out << "DH Revolute :" << " alpha:" << Math::radToDeg(l.getAlpha()) << ",  a:" << l.getA() << ",  d:" << l.getD() << ",  {theta}:"  << Math::radToDeg(l.getTheta()) << std::endl; 
    else
      out << "DH Revolute :" << " alpha:" << Math::radToDeg(l.getAlpha()) << ",  a:" << l.getA() << ",  d:" << l.getD() << ",  theta:"  << Math::radToDeg(l.getTheta()) << std::endl;       
    break;
  case KinematicChain::Link::Prismatic:  
    if (l.isActive())
      out << "DH Prismatic:" << " alpha:" << Math::radToDeg(l.getAlpha()) << ",  a:" << l.getA() << ",  {d}:" << l.getD() << ",  theta:"  << Math::radToDeg(l.getTheta()) << std::endl;
    else
      out << "DH Prismatic:" << " alpha:" << Math::radToDeg(l.getAlpha()) << ",  a:" << l.getA() << ",  d:" << l.getD() << ",  theta:"  << Math::radToDeg(l.getTheta()) << std::endl;
    break;
  case KinematicChain::Link::Translating:
    if (l.isActive())
      out << "Translating : direction {t}:" << l.getT() << "*" << l.getDirection() << std::endl;
    else
      out << "Translating : direction t:" << l.getT() << "*" << l.getDirection() << std::endl;
    break;
  case KinematicChain::Link::FixedTransform:
    out << "Transform   : translation:" << l.getTransform().column(4).toVector3() << " rotation:" << base::Orient(Matrix3(l.getTransform())).getQuat4() << std::endl;
    break;
  default: out << "unknown joint type\n";
  }
  return out;
}



