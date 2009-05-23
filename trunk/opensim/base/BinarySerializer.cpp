/****************************************************************************
  Copyright (C)1996 David Jung <opensim@pobox.com>

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
  
  $Id: BinarySerializer.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.8 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/BinarySerializer>

#include <base/Serializable>

using base::BinarySerializer;
using base::Serializer;
using base::Serializable;
using base::Referenced;
using base::VFile;
using base::ref;

using std::iostream;
using std::istream;
using std::ostream;
using std::ios;
using std::streambuf;


#ifdef DEBUG

// comment out to inhibit serialization debugging
//#define SDEBUG

#endif


BinarySerializer::BinarySerializer(SerializerType type, ref<VFile> archive)
  : Serializer(type,archive),
    stream(output?archive->iostream(iostream::binary|iostream::out)
	         :archive->iostream(iostream::binary|iostream::in)),
    buf(*stream.rdbuf())
{
#ifdef SDEBUG
  Debugln(Ser,"Constructed:" << (isOutput()?String("Output"):String("Input")) );
#endif
}

BinarySerializer::BinarySerializer(SerializerType type, std::ios& stream)
    : Serializer(type,stream), 
      stream(stream),
      buf(*stream.rdbuf())
{
  Assert(output?instanceof(stream, ostream)
	       :instanceof(stream, istream));
#ifdef SDEBUG
  Debugln(Ser,"Constructed:" << (isOutput()?String("Output"):String("Input")) );
#endif
}

BinarySerializer::~BinarySerializer()
{
}


bool BinarySerializer::followReferences(bool follow)
{
#ifdef SDEBUG
  Debugln(Ser,"Reference following " << (follow?String("enabled"):String("disabled")));
#endif
  return Serializer::followReferences(follow);
}


Serializer& BinarySerializer::serialize(char& c, const String& memberName)
{
  if (output) 
    buf.sputc(c); 
  else 
    c = char(buf.sbumpc()); 
#ifdef SDEBUG
  //Debugln(Ser,"serialized char:" << c);
#endif
  return *this;
}

Serializer& BinarySerializer::serialize(Byte& b, const String& memberName)
{
  if (output) 
    buf.sputc(char(b)); 
  else 
    b = Byte(buf.sbumpc()); 
#ifdef SDEBUG
  Debugln(Ser,"serialized Byte:" << b);
#endif
  return *this;
}

Serializer& BinarySerializer::serialize(bool& b, const String& memberName)
{
  if (output) 
    buf.sputc(char(b)); 
  else 
    b = Byte(buf.sbumpc()); 
#ifdef SDEBUG
  Debugln(Ser,"serialized bool:" << b);
#endif
  return *this;
}

Serializer& BinarySerializer::serialize(SInt& i, const String& memberName)
{
  if (output) 
    buf.sputn((const char*)(&i),sizeof(SInt)); 
  else 
    buf.sgetn((char*)(&i),sizeof(SInt)); 
#ifdef SDEBUG
  Debugln(Ser,"serialized SInt:" << i);
#endif
  return *this;
}

Serializer& BinarySerializer::serialize(Int& i, const String& memberName)
{
  if (output) 
    buf.sputn((const char*)(&i),sizeof(Int)); 
  else 
    buf.sgetn((char*)(&i),sizeof(Int)); 
#ifdef SDEBUG
  Debugln(Ser,"serialized Int:" << i);
#endif
  return *this;
}

Serializer& BinarySerializer::serialize(LInt& i, const String& memberName)
{
  if (output) 
    buf.sputn((const char*)(&i),sizeof(LInt)); 
  else 
    buf.sgetn((char*)(&i),sizeof(LInt)); 
#ifdef SDEBUG
  Debugln(Ser,"serialized LInt:" << i);
#endif
  return *this;
}

Serializer& BinarySerializer::serialize(String& s, const String& memberName)
{
  if (output) {  
    Int len = s.length();
    operator()(len); // write length first
    if (len>0)
      buf.sputn((const char*)(s.c_str()),len); 
  }
  else { 
    Int len;
    operator()(len); // read length first
    if (len>0) {
      char* sbuf = new char[len+1];
      buf.sgetn(sbuf,len);
      sbuf[len]=char(0);
      s = String(sbuf);
      delete[] sbuf;
    }
  }
#ifdef SDEBUG
  Debugln(Ser,"serialized String:" << s);
#endif
  return *this;
}

Serializer& BinarySerializer::serialize(Real& r, const String& memberName)
{
  if (output) 
    buf.sputn((const char*)(&r),sizeof(Real)); 
  else 
    buf.sgetn((char*)(&r),sizeof(Real)); 
#ifdef SDEBUG
  Debugln(Ser,"serialized Real:" << r);
#endif
  return *this;
}

void BinarySerializer::flush()
{
  if (output) {
    if (instanceof(stream,std::ostream)) {
      std::ostream* out = dynamic_cast<std::ostream*>(&stream);
      (*out) << std::flush;
    }
  } 
}


/// \todo optimize type name storage so it doesn't store repeated name strings
Serializer::TypeModifier BinarySerializer::preSerializeObject(String& typeName, TypeModifier typeModifier, const String& memberName, bool forceTypeSerialization)
{
  if (output) {
    switch (typeModifier) {
    case ObjectType: break;
    case ReferencedObjectType: 
      operator()(char(ReferencedObjectType)); break;
    case ObjectReferenceType: 
      operator()(char(ObjectReferenceType)); break;
    default:
      abort(String("unknown/invalid typeModifier passed for output. Member:")+memberName);
    }
    operator()(forceTypeSerialization);

    if (forceTypeSerialization) { // binary format doesn't contain type information unless necessary
      String typeString;
      try {
	typeString = base::demangleTypeidName(typeName);
      } catch (std::exception& e) {
	abort(String("error demangling typeName:")+typeName+" for member "+memberName+" - "+e.what());
      }

      operator()(typeString);
    }
    return typeModifier;
  }
  else { // input

    TypeModifier readTypeModifier = ObjectType;
    if (typeModifier != ObjectType) {    // only the type modifier of referenced objects is serialized


      // read type indicator (object or just ref)
      char c;
      operator()(c);
      readTypeModifier = TypeModifier(Int(c));

      if ((readTypeModifier == UnknownType) || (readTypeModifier >= InvalidType)) 
	abort("unknown serialized object type read");
    
      // check we read what, if anything, was expected
      if (typeModifier != UnknownType) {
	if (readTypeModifier != typeModifier)
	  abort("unexpected serialized referenced object type read");
      }
    }

    bool typeSerialized(false);
    operator()(typeSerialized);
    if (typeSerialized) {
      String readTypeName;
      operator()(readTypeName);
      if (typeName.empty())
	typeName = readTypeName;
      else 
	if (typeName != readTypeName)
	  abort(String("read type "+readTypeName+" but expected "+typeName));
    }
    else
      typeName.clear();      

    return readTypeModifier;
  }
}
