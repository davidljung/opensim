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
  
  $Id: Serializer.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.8 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/Serializer>

#include <base/Serializable>

using base::Serializer;
using base::Serializable;


#ifdef DEBUG

// comment out to inhibit serialization debugging
//#define SDEBUG

#endif


Serializer::Serializer(SerializerType type) 
  : output(type==Output), follow(true), aborted(false),
      serializePointerRecursionDepth(0)
{
  ptrs.push_back(0); // the null ptr is a special at index 0 (so nulls can be serialized)
  serialized.push_back(true);
}

Serializer::Serializer(SerializerType type, ref<VFile> archive) 
  : output(type==Output), follow(true), aborted(false),
      serializePointerRecursionDepth(0) 
{
  ptrs.push_back(0); // the null ptr is a special at index 0 (so nulls can be serialized)
  serialized.push_back(true);
}

Serializer::Serializer(SerializerType type, std::ios& stream) 
  : output(type==Output), follow(true), aborted(false),
      serializePointerRecursionDepth(0)
{
  ptrs.push_back(0); // the null ptr is a special at index 0 (so nulls can be serialized)
  serialized.push_back(true);
}

Serializer::~Serializer()
{
 if (isOutput() && !aborted) {
    // check for objects that have been referenced during Output serialization
    //  but not serialized
    Int outstandingSerializationCount = 0;
    for(Int pi=0; pi<serialized.size(); pi++) 
      if (!serialized[pi]) 
	outstandingSerializationCount++;
    if (outstandingSerializationCount>0) {
      Logln("Serializer destruction with outstanding object serializations (objects referenced in the Output stream, but not serialized) - " << outstandingSerializationCount << " objects.");
    }
  }
}


bool Serializer::followReferences(bool follow)
{
  if (!follow)
    depthAtFollowDisable = serializePointerRecursionDepth;
  bool oldfollow = this->follow;
  this->follow = follow;
  return oldfollow;
}



const String Serializer::inputToConstErrorString("cannot input to a const variable");


Serializer& Serializer::serialize(Serializable& r, const String& memberName)
{
  String typeName(typeid(r).name());
  preSerializeObject(typeName, ObjectType, memberName);
  r.serialize(*this);
  postSerializeObject(memberName);
  return *this;
}



Serializer& Serializer::operator()(Vector& v, const String& memberName)
{
  comment(memberName);
  if(isOutput()) {
    operator()(v.size(),"dim");
  }
  else {
    Int dim;
    operator()(dim,"dim");
    v.resize(dim);
  }

  for(Int i=0; i<v.size(); i++)
    operator()(v[i]);

  return *this;
}


Serializer& Serializer::operator()(Matrix& m, const String& memberName)
{
  comment(memberName);
  if(isOutput()) {
    operator()(m.size1(),"rows");
    operator()(m.size2(),"cols");
  }
  else {
    Int rows, cols;
    operator()(rows,"rows");
    operator()(cols,"cols");
    m.resize(rows,cols);
  }

  for(Int r=0; r<m.size1(); r++)
    for(Int c=0; c<m.size2(); c++)
      operator()(m(r,c));

  return *this;
}



inline Int Serializer::ptrIndex(Serializable* p)
{
  for(Int i=ptrs.size()-1; i>0; i--)
    if (ptrs[i] == p) return i;
  if (ptrs[0] == p) return 0;
  return ptrs.size();
}



void Serializer::serializePointer(Serializable*& p, Serializable::SerializableInstantiator* i, const String& memberName, bool forceTypeSerialization)
{
  serializePointerRecursionDepth++;

  if (output) { // Output

    Int pi = ptrIndex(p);
    if (pi == ptrs.size()) { 
      // wasn't found (reference to object not previously encountered)

      if (follow || (!follow && (serializePointerRecursionDepth==depthAtFollowDisable+1)) ) {
#ifdef SDEBUG
	Debugln(Ser,String(serializePointerRecursionDepth,' ') << "serializing(out) obj(" << (p?String(instanceof(*p,Object)?dynamic_cast<Object*>(p)->className():"unknown"):"null") << ") [" << pi << "] first encounter");
#endif
	// serialize type, index and object
	String typeName(typeid(*p).name());
	preSerializeObject(typeName,ReferencedObjectType, memberName, forceTypeSerialization);
	serializeReferenceIndex(pi); // output index
	ptrs.at(pi) = p;
	serialized.at(pi) = true;
	p->serialize(*this); // output object
	postSerializeObject(memberName);
      }
      else {
#ifdef SDEBUG
	Debugln(Ser,String(serializePointerRecursionDepth,' ') << "serializing(out) ptr(" << (p?String(instanceof(*p,Object)?dynamic_cast<Object*>(p)->className():"unknown"):"null") << ") [" << pi << "] first encounter");
#endif
	// serialize type and index only
	String typeName(typeid(*p).name());
	preSerializeObject(typeName,ObjectReferenceType, memberName, forceTypeSerialization);
	serializeReferenceIndex(pi); // output index
	ptrs.at(pi) = p;
	serialized.at(pi) = false;
	postSerializeObject(memberName);
      }

    }
    else { // already been encountered

      if ( (follow || (!follow && (serializePointerRecursionDepth==depthAtFollowDisable+1))) 
	   && !serialized[pi]) { 
#ifdef SDEBUG
	Debugln(Ser,String(serializePointerRecursionDepth,' ') << "serializing(out) obj(" << (p?String(instanceof(*p,Object)?dynamic_cast<Object*>(p)->className():"unknown"):"null") << ") [" << pi << "]");
#endif
	// object wasn't serialized on previous encounters, must have been in
	//  non-follow mode.  Serialize it now.
	String typeName(typeid(*p).name());
	preSerializeObject(typeName,ReferencedObjectType, memberName, forceTypeSerialization);
	serializeReferenceIndex(pi); // output index
	serialized[pi] = true;
	p->serialize(*this); // output object
	postSerializeObject(memberName);
      }
      else { 
#ifdef SDEBUG
	Debugln(Ser,String(serializePointerRecursionDepth,' ') << "serializing(out) ptr(" << (p?String(instanceof(*p,Object)?dynamic_cast<Object*>(p)->className():"unknown"):"null") << ") [" << pi << "]");
#endif
	// never serialize in non-follow mode (just output index)
	//  (or if it's already been serialized)
	String typeName(typeid(*p).name());
	preSerializeObject(typeName,ObjectReferenceType, memberName, forceTypeSerialization);
	serializeReferenceIndex(pi); // output index
	postSerializeObject(memberName);
      }
    }
  }
  else { // Input

    String typeName;
    TypeModifier readTypeModifier = preSerializeObject(typeName,UnknownType, memberName, forceTypeSerialization);

    if (readTypeModifier == ReferencedObjectType) { // a referenced object

      // deserialize
      Int pi;
      serializeReferenceIndex(pi); // read index

      if (pi > ptrs.size())
	throw serialization_error(Exception("serialization input stream corrupted (object out of sequence)"));
      else {
	if (pi == ptrs.size()) { // haven't read object before, instantiate it now
	  // create an instance to hold the new object, if necessary
	  if (p==0) {
	    if (i != 0)
	      p = i->newSerializable();
	    else { // dynamically lookup an instantiator
	      try{
		const Serializable::SerializableInstantiator& i(Serializable::getSerializableInstantiator(String(), typeName));
		p = i.newSerializable();
	      } catch (std::exception& e) { abort(e.what()); }
	    }
	  }
	  Assert(p);
	  
	  if (!typeName.empty()) {
	    if (typeName != base::className(typeid(*p)))
	      throw serialization_error(Exception(String("read unexpected object type: ")+typeName+" expected "+base::className(typeid(*p))));
	  }

#ifdef SDEBUG
	  Debugln(Ser,String(serializePointerRecursionDepth,' ') << "serializing(in) obj(" << (p?String(instanceof(*p,Object)?dynamic_cast<Object*>(p)->className():"unknown"):"null") << ") [" << pi << "]" << ((pi == ptrs.size())?" instantiated":"") );
#endif
	  ptrs.at(pi) = p;
	}
	else {
	  // we've encountered the object before - which means it was instantiated
	  //  but not serialized
	  p = ptrs[pi];
	}
	
	p->serialize(*this); // input object

	postSerializeObject(memberName);
      }
    }
    else if (readTypeModifier == ObjectReferenceType) { // an object reference

      Int pi;
      serializeReferenceIndex(pi); // read index

      if (pi>ptrs.size())
	throw serialization_error(Exception("serialization input stream corrupted (reference to object not yet encountered)"));
      else {
	if (pi == ptrs.size()) { // index to unread object
	  // create an object of the referenced type using the default constructor
	  //  (we can't initialize/serialize it until it is encoundered in the stream)
	  if (p==0) {
	    if (i != 0)
	      p = i->newSerializable();
	    else { // dynamically lookup an instantiator
	      try {
		const Serializable::SerializableInstantiator& i(Serializable::getSerializableInstantiator(String(), typeName));
		p = i.newSerializable();
	      } catch (std::exception& e) { abort(e.what()); }
	    }
	  }
	  Assert(p);

	  if (!typeName.empty()) {
	    if (typeName != base::className(typeid(*p)))
	      throw serialization_error(Exception(String("read unexpected object reference type: ")+typeName+" expected "+base::className(typeid(*p))));
	  }
#ifdef SDEBUG
	  Debugln(Ser,String(serializePointerRecursionDepth,' ') << "serializing(in) ptr(" << (p?String(instanceof(*p,Object)?dynamic_cast<Object*>(p)->className():"unknown"):"null") << ") [" << pi << "] instantiated" );
#endif
	  ptrs.at(pi) = p;
	}
	else { // backward reference to previously read object
	  p = ptrs[pi];
	  Assert(p);
#ifdef SDEBUG
	  Debugln(Ser,String(serializePointerRecursionDepth,' ') << "serializing(in) ptr(" << (p?String(instanceof(*p,Object)?dynamic_cast<Object*>(p)->className():"unknown"):"null") << ") [" << pi << "]" );
#endif

	}
      }

      postSerializeObject(memberName);

    }
    else 
      throw serialization_error(Exception("serialization input stream corrupted (unknown reference type)"));
  }

  serializePointerRecursionDepth--;

}


void Serializer::abort(const String& exceptionString)
{ 
  if (!aborted) {
    aborted=true; 
    Logln(exceptionString);
    throw serialization_error(Exception(exceptionString)); 
  }
}
