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
  
  $Id: SimpleXMLSerializer.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.6 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/SimpleXMLSerializer>

#include <base/Serializable>

using base::SimpleXMLSerializer;
using base::Serializer;
using base::Serializable;
using base::Referenced;
using base::VFile;
using base::ref;

using std::iostream;
using std::istream;
using std::ostream;


#ifdef DEBUG

// comment out to inhibit serialization debugging
//#define SDEBUG

#endif


SimpleXMLSerializer::SimpleXMLSerializer(SerializerType type, ref<VFile> archive)
  : Serializer(type,archive),
    istream(output?0:&archive->iostream(iostream::in)),
    ostream(output?&archive->iostream(iostream::out):0),
    indent(0)
{
  serializeHeader(archive->name().str());

#ifdef SDEBUG
  Debugln(Ser,"Constructed:" << (isOutput()?String("Output"):String("Input")) );
#endif
}

SimpleXMLSerializer::SimpleXMLSerializer(SerializerType type, std::ios& stream)
  : Serializer(type,stream),
    indent(0)
{
  Assert(output?instanceof(stream, std::ostream)
	       :instanceof(stream, std::istream));
  if (output) 
    ostream = dynamic_cast<std::ostream*>(&stream);
  else 
    istream = dynamic_cast<std::istream*>(&stream);

  serializeHeader(String("stream"));

#ifdef SDEBUG
  Debugln(Ser,"Constructed:" << (isOutput()?String("Output"):String("Input")) );
#endif
}


SimpleXMLSerializer::~SimpleXMLSerializer()
{
  if (aborted) {
    if (output) flush();
    return;
  }

  if (output) {
    indent--;
    (*ostream) << "</serialization>" << std::endl;
    flush();
  }
  else {
    readEndTag("serialization");
  }
}


bool SimpleXMLSerializer::followReferences(bool follow)
{
#ifdef SDEBUG
  Debugln(Ser,"Reference following " << (follow?String("enabled"):String("disabled")));
#endif
  return Serializer::followReferences(follow);
}


Serializer& SimpleXMLSerializer::serialize(char& c, const String& memberName)
{
  String tagName(memberName.empty()?"char":memberName);
  if (output) {
    try {
      (*ostream) << String(indent,' ') << "<" << tagName << ">" << c << "</" << tagName << ">" << std::endl;
    } catch (std::exception& e) { abort(e.what()); }
  }
  else { 
    TagData tag;
    if (readStartTag(tag)) {
      if ((tag.name != tagName) && (tag.name != "char"))
	abort("expected element "+tagName+" but got "+tag.name);
      try {
	(*istream) >> c;
      } catch (std::exception& e) { abort(e.what()); }
      readEndTag(tag.name);
    }
  }

#ifdef SDEBUG
  //Debugln(Ser,"serialized char:" << c);
#endif
  return *this;
}

Serializer& SimpleXMLSerializer::serialize(Byte& b, const String& memberName)
{
  String tagName(memberName.empty()?"Byte":memberName);
  if (output) {
    try {
      (*ostream) << String(indent,' ') << "<" << tagName << ">" << Int(b) << "</" << tagName << ">" << std::endl;
    } catch (std::exception& e) { abort(e.what()); }
  }
  else {
    TagData tag;
    if (readStartTag(tag)) {
      if (tag.name != tagName)
	abort("expected element "+tagName+" but got "+tag.name);
      Int i;
      try {
	(*istream) >> i;
      } catch (std::exception& e) { abort(e.what()); }
      b = Byte(i);
      readEndTag(tag.name);
    }
  }

#ifdef SDEBUG
  Debugln(Ser,"serialized Byte:" << b);
#endif
  return *this;
}

Serializer& SimpleXMLSerializer::serialize(bool& b, const String& memberName)
{
  String tagName(memberName.empty()?"bool":memberName);
  if (output) {
    try {
      (*ostream) << std::boolalpha;
      (*ostream) << String(indent,' ') << "<" << tagName << ">" << b << "</" << tagName << ">" << std::endl;
    } catch (std::exception& e) { abort(e.what()); }
  }
  else {
    TagData tag;
    if (readStartTag(tag)) {
      if (tag.name != tagName)
	abort("expected element "+tagName+" but got "+tag.name);
      try {
	(*istream) >> std::boolalpha;
	(*istream) >> b;
      } catch (std::exception& e) { abort(e.what()); }
      readEndTag(tag.name);
    }
  }

#ifdef SDEBUG
  Debugln(Ser,"serialized bool:" << b);
#endif
  return *this;
}

Serializer& SimpleXMLSerializer::serialize(SInt& i, const String& memberName)
{
  String tagName(memberName.empty()?"SInt":memberName);
  if (output) {
    try {
      (*ostream) << String(indent,' ') << "<" << tagName << ">" << base::intToString(i) << "</" << tagName << ">" << std::endl;
    } catch (std::exception& e) { abort(e.what()); }
  }
  else {
    TagData tag;
    if (readStartTag(tag)) {
      if (tag.name != tagName)
	abort("expected element "+tagName+" but got "+tag.name);
      try {
	(*istream) >> i;
      } catch (std::exception& e) { abort(e.what()); }
      readEndTag(tag.name);
    }
  }

#ifdef SDEBUG
  Debugln(Ser,"serialized SInt:" << i);
#endif
  return *this;
}

Serializer& SimpleXMLSerializer::serialize(Int& i, const String& memberName)
{
  String tagName(memberName.empty()?"Int":memberName);
  if (output) {
    try {
      (*ostream) << String(indent,' ') << "<" << tagName << ">" << base::intToString(i) << "</" << tagName << ">" << std::endl;
    } catch (std::exception& e) { abort(e.what()); }
  }
  else {
    TagData tag;
    if (readStartTag(tag)) {
      if (tag.name != tagName)
	abort("expected element "+tagName+" but got "+tag.name);
      try {
	(*istream) >> i;
      } catch (std::exception& e) { abort(e.what()); }
      readEndTag(tag.name);
    }
  }
#ifdef SDEBUG
  Debugln(Ser,"serialized Int:" << i);
#endif
  return *this;
}

Serializer& SimpleXMLSerializer::serialize(LInt& i, const String& memberName)
{
  String tagName(memberName.empty()?"LInt":memberName);
  if (output) {
    try {
      (*ostream) << String(indent,' ') << "<" << tagName << ">" << base::intToString(i) << "</" << tagName << ">" << std::endl;
    } catch (std::exception& e) { abort(e.what()); }
  }
  else {
    TagData tag;
    if (readStartTag(tag)) {
      if (tag.name != tagName)
	abort("expected element "+tagName+" but got "+tag.name);
      try {
	(*istream) >> i;
      } catch (std::exception& e) { abort(e.what()); }
      readEndTag(tag.name);
    }
  }

#ifdef SDEBUG
  Debugln(Ser,"serialized LInt:" << i);
#endif
  return *this;
}

Serializer& SimpleXMLSerializer::serialize(String& s, const String& memberName)
{
  String tagName(memberName.empty()?"string":memberName);
  if (output) {
    try {
      (*ostream) << String(indent,' ') << "<" << tagName << ">" << s << "</" << tagName << ">" << std::endl;
    } catch (std::exception& e) { abort(e.what()); }
  }
  else {
    TagData tag;
    if (readStartTag(tag)) {
      if (tag.name != tagName)
	abort("expected element "+tagName+" but got "+tag.name);
      try {
	s.clear();
	// read chars until '<'
	char p = nextChar();
	while (p != '<') { s += p; p = nextChar(); }
	istream->putback(p);
      } catch (std::exception& e) { abort(e.what()); }
      readEndTag(tag.name);
    }
  }

#ifdef SDEBUG
  Debugln(Ser,"serialized String:" << s);
#endif
  return *this;
}

Serializer& SimpleXMLSerializer::serialize(Real& r, const String& memberName)
{
  String tagName(memberName.empty()?"real":memberName);
  if (output) {
    try {
      (*ostream) << String(indent,' ') << "<" << tagName << ">" << base::realToString(r) << "</" << tagName << ">" << std::endl;
    } catch (std::exception& e) { abort(e.what()); }
  }
  else {
    TagData tag;
    if (readStartTag(tag)) {
      if (tag.name != tagName)
	abort("expected element "+tagName+" but got "+tag.name);
      try {
	(*istream) >> r;
      } catch (std::exception& e) { abort(e.what()); }
      readEndTag(tag.name);
    }
  }

#ifdef SDEBUG
  Debugln(Ser,"serialized Real:" << r);
#endif
  return *this;
}

void SimpleXMLSerializer::flush()
{
  if (output) 
    (*ostream) << std::flush;
}




Serializer::TypeModifier SimpleXMLSerializer::preSerializeObject(String& typeName, TypeModifier typeModifier, const String& memberName, bool forceTypeSerialization)
{
  String tagName(memberName.empty()?"object":memberName);
  String typeString;
    
  if (output) {
    
    try {
      typeString = base::demangleTypeidName(typeName);
    } catch (std::exception& e) {
      abort(String("error demangling typeName:")+typeName+" for element:" +tagName+" - "+e.what());
    }
    
    String modifierString;
    if (typeModifier == ReferencedObjectType)
      modifierString = "ro";
    else if (typeModifier == ObjectReferenceType)
      modifierString = "r";

    if (!modifierString.empty())
      modifierString = String(" ref=\""+modifierString+"\"");

    (*ostream) << String(indent,' ') << "<" << tagName << " type=\"" << typeString << "\"" << modifierString << ">" << std::endl;
    indent++;

    return typeModifier;
  }
  else { // input
    TagData tag;

    if (readStartTag(tag)) {

      if (tag.name != tagName)
	abort("expected element "+tagName+" but got "+tag.name);

      if (!typeName.empty()) {
	try {
	  typeString = base::demangleTypeidName(typeName);
	} catch (std::exception& e) {
	  abort(String("error demangling typeName:")+typeName+" for element:" +tagName+" - "+e.what());
	}

	if (tag.attribute("type") != typeString) 
	  abort("expected object of type "+typeString+" but got "+tag.attribute("type"));
      }
      else
	typeName = tag.attribute("type");
      
      String modifierString(tag.attribute("ref"));
      TypeModifier readTypeModifier = ObjectType;

      if (modifierString == "r")
	readTypeModifier = ObjectReferenceType;
      else if (modifierString == "ro")
	readTypeModifier = ReferencedObjectType;

      if (typeModifier != UnknownType)
	if (typeModifier != readTypeModifier)
	  abort("got difference object reference type modifier than expected");

      return readTypeModifier;
    }
    else 
      return UnknownType;
  }
}


void SimpleXMLSerializer::postSerializeObject(const String& memberName)
{
  String tagName(memberName.empty()?"object":memberName);

  if (output) {
    indent--;
    (*ostream) << String(indent,' ') << "</" << tagName << ">" << std::endl;
  }
  else { // input 
    readEndTag(tagName);
  }
}


void SimpleXMLSerializer::serializeComment(const String& comment)
{
  if (output) {
    (*ostream) << String(indent,' ') << "<!-- " << comment << " -->" << std::endl;
  }
}


Serializer& SimpleXMLSerializer::hint(Int h)
{
  if (h == Indent) indent++;
  else if (h == Unindent) indent--;
  return *this;
}


bool SimpleXMLSerializer::serializeHeader(const String& target)
{
  
  if (output) {
    try {
      (*ostream) << "<?xml version =\"1.0\" encoding=\"UTF-8\" standalone=\"yes\" ?>" << std::endl;
      (*ostream) << "<serialization type=\"base::SimpleXMLSerializer\" version=\"1.0\" target=\"" << target << "\">" << std::endl;
      indent++;
    } catch (std::exception& e) { abort(e.what()); }
  }
  else { // input
    TagData tag;
    bool xmlTagOK = readStartTag(tag);
    if (xmlTagOK) {
      if (tag.name != "?xml") {
	xmlTagOK = false;
      } 
    }
    
    if (!xmlTagOK)
      abort("Bad XML header");
    
    bool rootElementOK = readStartTag(tag);
    if (rootElementOK) {
      if (tag.name != "serialization") {
	rootElementOK = false;
      }
      else {
	if (tag.attribute("type") != "base::SimpleXMLSerializer")
	  abort("Serialization format not supported");

	if (tag.attribute("version") != "1.0")
	  abort("Only serialization format version 1.0 is supported");
      }
    }

    if (!rootElementOK)
      abort("This XML format or version not is supported");

  }

  return true;
}


bool SimpleXMLSerializer::readStartTag(TagData& tag)
{
  Assert(!output);

  tag.name.clear();
  tag.attributeNames.clear();
  tag.attributeValues.clear();

  (*istream) >> std::noskipws;

  char p = nextNWSChar();
  if (p == '<') {
    // check for '!--' comment start
    p = nextChar();
    tag.name += p;
    if (p=='!') {
      tag.name += p;
      do { p = nextChar(); tag.name += p; } while (p==' ');
      if (p=='-') {
	tag.name += p;
	p = nextChar();
	if (p=='-') { // comment start
	  tag.name.clear();
	  // skip to end of comment
	  bool endcomment = false;
	  do {
	    p = nextChar();
	    if (p == '-') {
	      p = nextChar();
	      if (p == '-') {
		p = nextChar();
		if (p == '>') {
		  endcomment = true;
		  // eat whitespace after comment
		  p = nextNWSChar();
		  if (p!='<') {
		    istream->putback(p);
		    return false;
		  }
		}
	      }
	    }
	  } while (!endcomment);
	}
      }
    } // end check for comment

    // read tag name (ends with space or '>')
    p = nextChar();
    while ((p != '>') && (p != ' ')) {
      tag.name += p;
      p = nextChar();
    } 

    // white space is allowed after tag name, but before '>', eat it
    if (p!='>') {
      p = nextNWSChar();
      if (p!='>') 
	istream->putback(p); // still isn't tag close '>', must be start of attribute name, put it back
    }

    if (p!='>') { // tag must have some attributes, read them
      bool tagclose=false;
      do {
	String attribName;
	p = nextNWSChar();
	while ((p!=' ') && (p!='=')) { attribName += p; p = nextChar(); }
	if (p!='=') p = nextNWSChar();
	if (p!='=') abort("'=' expected after attribute name:"+attribName+" of element:"+tag.name);
	p = nextNWSChar();
	if (p!='\"') abort("attribute value must be quoted (in attribute name:"+attribName+" of element:"+tag.name+")");
	// read in attribute value
	String attribValue;
	p = nextChar();
	while (p!='\"') { attribValue += p; p = nextChar(); }
	tag.attributeNames.push_back(attribName);
	tag.attributeValues.push_back(attribValue);

	p = nextNWSChar();

	if ((tag.name[0]=='?') && (p=='?')) 
	  p = nextChar();

	if (p == '>') tagclose=true;

	if (p == '<') 
	  abort("unexpected '<' - element "+tag.name+" not terminated?");

	if (!tagclose) istream->putback(p);

      } while (!tagclose);

    }

    return true;
  }    

  istream->putback(p);
  return false;

}


bool SimpleXMLSerializer::readEndTag(const String& tagName)
{
  Assert(!output);

  (*istream) >> std::noskipws;

  String name;
  char p = nextNWSChar();
  if (p == '<') {
    // check for '!--' comment start
    p = nextChar();
    name += p;
    if (p=='!') {
      name += p;
      do { p = nextChar(); name += p; } while (p==' ');
      if (p=='-') {
	name += p;
	p = nextChar();
	if (p=='-') { // comment start
	  name.clear();
	  // skip to end of comment
	  bool endcomment = false;
	  do {
	    p = nextChar();
	    if (p == '-') {
	      p = nextChar();
	      if (p == '-') {
		p = nextChar();
		if (p == '>') {
		  endcomment = true;
		  // eat whitespace after comment
		  p = nextNWSChar();
		  if (p!='<') {
		    istream->putback(p);
		    return false;
		  }
		}
	      }
	    }
	  } while (!endcomment);
	}
      }
    } // end check for comment
    
    // read tag name (ends with space or '>')
    p = nextChar();
    while ((p != '>') && (p != ' ')) {
      name += p;
      p = nextChar();
    } 


    // white space is allowed after tag name, but before '>', eat it
    if (p!='>') 
      p = nextNWSChar();
    
    if (p!='>') // still no '>'?, improperly terminated
      abort("expected '>' to close element:<"+name);

    if (!tagName.empty())
      if (name != String("/"+tagName))
	abort("expected closing element </"+tagName+"> but read <"+name+">");
    
    return true;
  }

  return false;
}

