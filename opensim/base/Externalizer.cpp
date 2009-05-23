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

  $Id: Externalizer.cpp 1094 2004-09-13 17:39:09Z jungd $
  $Revision: 1.5 $
  $Date: 2004-09-13 13:39:09 -0400 (Mon, 13 Sep 2004) $
  $Author: jungd $

****************************************************************************/

#include <base/Externalizer>

#include <base/Application>
#include <base/Math>
#include <base/Externalizable>
#include <base/externalization_error>
#include <base/io_error>

#include <cstdio>

using base::Externalizer;
using base::Externalizable;
using base::VFile;
using base::Math;

using base::dom;

using std::iostream;
using std::istream;
using std::ostream;

bool Externalizer::xmlInitialized = false;



const String Externalizer::newline = String("\n");


Externalizer::Externalizer(Externalizable::ExternalizationType type, ref<VFile> archive)
  : xmlMode(false), impl(0), doctype(0), isoutput(type & Externalizable::Output), aborted(false),
    istream(isoutput?0:&archive->iostream(iostream::in)),
    ostream(isoutput?&archive->iostream(iostream::out):0),
    path(archive->path())
{
}

Externalizer::Externalizer(Externalizable::ExternalizationType type, std::ios& stream)
  : xmlMode(false), impl(0), doctype(0), isoutput(type & Externalizable::Output), aborted(false),
    path(String())
{
  Assert(isoutput?instanceof(stream, std::ostream)
                 :instanceof(stream, std::istream));
  if (isoutput)
    ostream = dynamic_cast<std::ostream*>(&stream);
  else
    istream = dynamic_cast<std::istream*>(&stream);
}


Externalizer::~Externalizer()
{
  if (xmlMode) {
    xmlWriteDocument(xmldoc);
    xmlReleaseDocument();
  }
}


const String Externalizer::inputToConstErrorString("cannot input to a const variable");
const String Externalizer::inputDuringOutputErrorString("cannot input from an output Externalizer");


void Externalizer::flush()
{
  if (!xmlMode)
    if (isoutput)
      (*ostream) << std::flush;
}


String Externalizer::readLine()
{
  if (isOutput()) abort(inputDuringOutputErrorString);

  if (!xmlMode) {
    char linebuf[1024];
    istream->getline(linebuf,1024);
    linebuf[1023]=char(0);
    return String(linebuf);
  }
  else {
    Unimplemented;
  }
}


String Externalizer::readString(SInt maxLength)
{
  if (isOutput()) abort(inputDuringOutputErrorString);

  if (!xmlMode) {
    char* buf = new char[maxLength+1];

    input().readsome(buf, maxLength);
    buf[maxLength] = 0;

    String str(buf);
    delete[] buf;

    return str;
  }
  else {
    Unimplemented;
  }
}


void Externalizer::writeLine(const String& line)
{
  if (!isOutput()) return;

  if (!xmlMode) {
    output() << line << std::endl;
  }
  else {
    appendText(xmlcontext.top(), line);
  }
}


void Externalizer::writeString(const String& str)
{
  if (!isOutput()) return;

  if (!xmlMode) {
    output() << str;
  }
  else {
  }
}

void Externalizer::writeComment(const String& comment)
{
  if (!isOutput()) return;

  if (!xmlMode)
    writeLine(String("# ")+comment);
  else {
    appendComment(xmlcontext.top(), comment);
  }

}



// XML stuff

void Externalizer::initXMLDoc(const String& rootElementName)
{
  if (xmlMode) return;

  xmlInitialize();

  parser = 0; xmldoc = 0;
  impl = dom::DOMImplementationRegistry::getDOMImplementation(XS("LS"));
  xmlMode = true;

  if (isInput()) {
    // parse the whole document
    xmldoc = xmlParseDocument();
    Assert(xmldoc);
    dom::DOMElement* docElem = xmldoc->getDocumentElement();
    if (!docElem)
      throw base::externalization_error(Exception("failed to obtain XML document element - invalid XML document."));

    xmlcontext.push( docElem );
   }
  else { // output
    // create a new document
    try {
      xmldoc = impl->createDocument(
                                  XS("http://www.w3.org/XML/1998/namespace"),                    // root element namespace URI
                                  XS(rootElementName),  // root element name
                                  doctype);             // document type
    } catch (dom::DOMException& e) {
      abort(String("error creating document:")+String(XS(e.msg)) );
    }
    xmlcontext.push( xmldoc->getDocumentElement() );
    xmlcontext.top()->appendChild(xmldoc->createTextNode(XS(newline)));
  }

}


dom::DOMElement* Externalizer::createElement(const String& tagname, bool indenting)
{
  dom::DOMElement* elem;

  if (!xmlMode) { // first element for document?
    String name(tagname);
    if (name == "") name = base::Application::getShortName();
    initXMLDoc(tagname);
    elem = xmldoc->getDocumentElement();
  }
  else
    elem = xmldoc->createElement(XS(tagname));

  // add a newline as the first child of the element so
  //  that the formatter knows to indent
  if (indenting)
    elem->appendChild(xmldoc->createTextNode(XS(newline)));

  return elem;
}


void Externalizer::appendElement(dom::DOMNode* n, const String& tagname, bool indenting)
{
  n->appendChild( createElement(tagname, indenting) );
}


void Externalizer::appendProcessingInstruction(dom::DOMNode* n, const String& target, const String& data)
{
  Assert(xmlMode);
  dom::DOMProcessingInstruction* pi = xmldoc->createProcessingInstruction(XS(target), XS(data));
  n->appendChild(pi);
}


void Externalizer::setDocumentType(const String& name, const String& publicid, const String& sysid)
{
  if (doctype) return; // already set

  if (!impl)
    impl = dom::DOMImplementationRegistry::getDOMImplementation(XS("LS"));

  try {
    doctype = impl->createDocumentType(XS(name),XS(publicid), XS(sysid));
  } catch (dom::DOMException& e) {
    abort(String("error setting document type:")+String(XS(e.msg)) );
  }

}



dom::DOMText* Externalizer::createText(const String& text)
{
  Assert(xmlMode);
  return xmldoc->createTextNode(XS(text));
}


void Externalizer::appendText(dom::DOMNode* n, const String& text)
{
  n->appendChild( createText(text) );
}


dom::DOMComment* Externalizer::createComment(const String& comment)
{
  Assert(xmlMode);
  return xmldoc->createComment(XS(comment));
}


void Externalizer::appendComment(dom::DOMNode* n, const String& comment)
{
  n->appendChild( createComment(comment) );
}


dom::DOMCDATASection* Externalizer::createCDATA(const String& text)
{
  Assert(xmlMode);
  return xmldoc->createCDATASection(XS(text));
}


void Externalizer::appendCDATA(dom::DOMNode* n, const String& text)
{
  Assert(xmlMode);
  n->appendChild( createCDATA(text) );
}

/// get first element with name tagname, either n or its immediate children
dom::DOMElement* Externalizer::getFirstElement(dom::DOMNode* n, const String& tagname, bool required)
{
  Assert(xmlMode);

  if (isElement(n, tagname)) // is n an element with name tagname?
    return dynamic_cast<dom::DOMElement*>(n);
  else // no, look in children
    return getFirstChildElement(n, tagname, required);
}

/// get first immediate child element of n with name tagname
dom::DOMElement* Externalizer::getFirstChildElement(dom::DOMNode* n, const String& tagname, bool required)
{
  Assert(xmlMode);
  dom::DOMElement* e = 0;

  if (n->hasChildNodes()) {
    dom::DOMNode* c = n->getFirstChild();
    while (c && (e==0) ) {
      if (isElement(c, tagname))
        e = dynamic_cast<dom::DOMElement*>(c);

      c = c->getNextSibling();
    }

  }

  if (required && (e==0)) {
    String pstr;
    if (isElement(n))
      pstr = " within element '"+elementTagName(n)+"'";
    abort(String("expected element '")+tagname+"'"+pstr);
  }

  return e;
}


dom::DOMElement* Externalizer::getNextSiblingElement(dom::DOMNode* n, const String& tagname, bool required)
{
  Assert(xmlMode);
  dom::DOMElement* e = 0;

  dom::DOMNode* c = n->getNextSibling();
  while (c && (e == 0)) {
    if (isElement(c, tagname))
      e = dynamic_cast<dom::DOMElement*>(c);

    c = c->getNextSibling();
  }

  if (required && (e==0))
    abort(String("expected ")+tagname+" element");

  return e;
}


/// remove element e from the tree (no-op if e is the current context or has no parent)
void Externalizer::removeElement(dom::DOMElement* e)
{
  Assert(xmlMode);
  if (context()->isSameNode(e)) return;

  dom::DOMNode* p = e->getParentNode();
  if (p)
    p->removeChild(e);
}


String Externalizer::getContainedText(dom::DOMNode* n, bool removeIndentation)
{
  Assert(xmlMode);
  String text;

  if (n->getNodeType() == dom::DOMNode::TEXT_NODE) {
    dom::DOMText* t = dynamic_cast<dom::DOMText*>(n);
    text += String(XS(t->getData()));
  }
  else if (n->getNodeType() == dom::DOMNode::CDATA_SECTION_NODE) {
    dom::DOMCDATASection* cd = dynamic_cast<dom::DOMCDATASection*>(n);
    text += String(XS(cd->getData()));
  }
  else if (n->hasChildNodes()) {

    dom::DOMNode* c = n->getFirstChild();

    while (c) {
      text += getContainedText(c);
      c = c->getNextSibling();
    }

  }

  if (removeIndentation) {

    // remove leading spaces from each line
    String newtext;
    SInt pos=0;
    while (pos < SInt(text.size())) {

      // skip over spaces
      while (((text[pos] == ' ') || (text[pos] == '\t')) && (pos < SInt(text.size())))
        pos++;

      if (pos < SInt(text.size())) {

        // extract text upto (& including) next new-line (or to end if no more newlines)
        Int nlpos = text.find( newline, pos );
        if (nlpos != String::npos) {
          newtext += text.substr(pos, nlpos - pos + 1);
          pos = nlpos+1; // pos after newline
        }
        else {
          newtext += text.substr(pos, text.size() - pos);
          pos = text.size(); // pos at end (to end the loop)
        }

      }

    }
    text = newtext;
  }


  return text;
}


String Externalizer::getElementAttribute(dom::DOMElement* e, const String& attrname, bool required)
{
  Assert(xmlMode);
  dom::DOMAttr* attr = e->getAttributeNode( XS(attrname) );

  if (attr == 0) {
    if (required)
      abort(String("expected '")+attrname+"' attribute of element '"+String(XS(e->getTagName()))+"'" );
    else
      return String();
  }

  if (required && !attr->getSpecified())
      abort(String("expected explicit value for '")+attrname+"' attribute of element '"+String(XS(e->getTagName()))+"'" );

  return XS( attr->getValue() );
}


String Externalizer::getDefaultedElementAttribute(dom::DOMElement* e, const String& attrname, const String& defaultValue)
{
  Assert(xmlMode);
  dom::DOMAttr* attr = e->getAttributeNode( XS(attrname) );

  if (attr == 0)
    return defaultValue;
  else
    if (!attr->getSpecified())
      return (defaultValue=="DTD")?String(XS( attr->getValue() )):defaultValue;

  return XS( attr->getValue() );
}



array<String> Externalizer::splitIntoLines(const String& text, bool removeBlankLines)
{
  array<String> lines;

  Int pos=0;
  while (pos < text.size()) {
    SInt nlpos = text.find( newline, pos );

    if (nlpos != -1) {
      if (nlpos-pos != 0) {
        String line( text.substr(pos, nlpos - pos) );
        lines.push_back( line );
      }
      else
        if (!removeBlankLines)
          lines.push_back( String() );
      pos = nlpos+1;
    }
    else {
      lines.push_back( text.substr(pos, text.size()-pos) );
      pos = text.length();
    }
  }

  return lines;
}


array<String> Externalizer::splitAtDelimiter(const String& line, char delimiter)
{
  array<String> fields;

  if (line.size() == 0) return fields;
  if (line.size() == 1) {
    fields.push_back( line );
    return fields;
  }

  Int pos=0;
  while (pos < line.size()) {
    SInt nlpos = line.find( delimiter, pos );
    if (nlpos != -1) {
      if (nlpos-pos != 0) {
        String field( line.substr(pos, nlpos - pos) );
        fields.push_back( field );
      }
      pos = nlpos+1;
    }
    else {
      fields.push_back( line.substr(pos, line.size()-pos) );
      pos = line.length();
    }
  }

  return fields;
}


base::Vector Externalizer::stringsToReals(const array<String>& strings, bool acceptUnits)
{
  Vector v( zeroVector(strings.size()) );

  for(Int i=0; i<strings.size(); i++) {
    const String& s(strings[i]);
    v[i] = base::stringToReal(s);
    if (s.length() > 2) {
      if (s.substr(s.length()-2,s.length()-1) == "in")
        v[i] *= 0.0254; // inches to meters
      else if (s.substr(s.length()-3,s.length()-1) == "deg")
        v[i] = Math::degToRad(v[i]); // degrees to radians
    }
  }

  return v;
}


base::Matrix4 Externalizer::toMatrix4(const String& text)
{
  array<String> lines = splitIntoLines(text, true);
  if (lines.size() != 4)
    throw base::externalization_error(Exception("expected 4x4 matrix"));

  Matrix4 m;
  for(Int r=0; r<4; r++) {
    array<String> rowEltStrings = splitAtDelimiter(lines[r],' ');
    Vector row( stringsToReals(rowEltStrings) );
    if (row.size() != 4)
      throw base::externalization_error(Exception("expected 4x4 matrix"));
    m(r+1,1) = row[0];
    m(r+1,2) = row[1];
    m(r+1,3) = row[2];
    m(r+1,4) = row[3];
  }
  return m;
}

String Externalizer::toString(const base::Matrix4& m)
{
  char buf[256];
  sprintf(buf,"%8.4f %8.4f %8.4f %8.4f\n%8.4f %8.4f %8.4f %8.4f\n%8.4f %8.4f %8.4f %8.4f\n%8.4f %8.4f %8.4f %8.4f\n",
          m(1,1),m(1,2),m(1,3),m(1,4),
          m(2,1),m(2,2),m(2,3),m(2,4),
          m(3,1),m(3,2),m(3,3),m(3,4),
          m(4,1),m(4,2),m(4,3),m(4,4) );
  return String(buf);
}


base::Vector3 Externalizer::toVector3(const String& text, bool acceptUnits)
{
  String s1( removeChar(text, '(') );
  String s2( removeChar(s1, ')') );
  array<String> eltStrings = splitAtDelimiter(s2,',');
  Vector v( stringsToReals(eltStrings, acceptUnits) );
  if (v.size() != 3)
    throw base::externalization_error(Exception("expected 3 dim vector"));
  return Vector3(v[0], v[1], v[2]);
}

base::Vector Externalizer::toVector(const String& text, bool commasep, bool acceptUnits)
{
  String s1( removeChar(text, '(') );
  String s2( removeChar(s1, ')') );
  String s3( commasep?removeChar(s2,' '):s2);
  array<String> eltStrings = splitAtDelimiter(s3,commasep?',':' ');
  return stringsToReals(eltStrings, acceptUnits);
}



String Externalizer::toString(const base::Vector3& v)
{
  char buf[48];
  sprintf(buf,"(%8.4f, %8.4f, %8.4f)",v.x,v.y,v.z);
  return String(buf);
}


String Externalizer::toString(const base::Vector& v, bool commasep)
{
  String str;
  char buf[20];
  for(Int i=0; i<v.size(); i++) {
    sprintf(buf,"%8.4f",v[i]);
    str += String(buf);
    if (commasep && (i !=v.size()-1) )
      str += ",";
    else
      str += " ";
  }

  if (commasep)
    str = String("(")+str+")";

  return str;
}


String Externalizer::toString(const base::Matrix& m)
{
  if ((m.size1() == 0) || (m.size2() == 0)) return String("[]");

  String line("[");
  char buf[16];

  for(Int r=0; r<m.size1(); r++) {
    String row;
    for(Int c=0; c<m.size2(); c++) {
      if (c!=m.size2())
        sprintf(buf,"%8.4f ",m(r,c));
      else
        sprintf(buf,"%8.4f",m(r,c));
      row+=String(buf);
    }
    if (r < m.size1()-1) row += "; ";
    line += row;
  }
  line += "]";

  return line;
}


base::Matrix Externalizer::toMatrix(const String& text)
{
  String s1( removeChar(text, '[') );
  String s2( removeChar(s1, ']') );
  array<String> rowStrings = splitAtDelimiter(s2,';'); // split rows
  Int rows = rowStrings.size();
  Int cols = splitAtDelimiter(rowStrings[0],' ').size(); // count cols
  Matrix m(rows,cols);

  for(Int r=0; r<rows; r++) {
    array<String> eltStrings = splitAtDelimiter(rowStrings[r],' ');
    matrixRow(m,r) = stringsToReals(eltStrings);
  }

  return m;
}


String Externalizer::toString(Real v, const String& units)
{
  if (units=="m")
    return base::realToString(v);
  else
    return base::realToString(v)+units;
}

Real Externalizer::toReal(const String& text, bool acceptUnits)
{
  Real v = base::stringToReal(text);
  if (acceptUnits && (text.length() > 2)) {
    if (text.substr(text.length()-2,text.length()-1) == "in")
      v *= 0.0254; // inches to meters
    else if (text.substr(text.length()-3, text.length()-1) == "deg")
      v = Math::degToRad(v); // degrees to radians
  }
  return v;
}




String Externalizer::removeChar(const String& text, char toRemove)
{
  String newtext;
  for(Int i=0; i<text.size(); i++)
    if (text[i] != toRemove)
      newtext += text[i];
  return newtext;
}


bool Externalizer::isNumeric(const String& text)
{
  bool numeric=true;
  std::locale loc;
  const std::ctype<char>& ct = std::use_facet<std::ctype<char> >(loc);
  Int i=0;
  while (numeric && (i<text.size())) {
    if (!ct.is(std::ctype_base::digit,text[i]))
      numeric=false;
    i++;
  }
  return numeric;
}





void Externalizer::xmlInitialize()
{
  if (!xmlInitialized) {
    try {
      dom::XMLPlatformUtils::Initialize();
      xmlInitialized = true;
    }
    catch (const dom::XMLException& toCatch) {
      throw std::runtime_error(Exception(String( dom::XMLString::transcode(toCatch.getMessage()) )));
    }
  }
}



dom::DOMDocument* Externalizer::xmlParseDocument()
{
  if (isOutput()) abort(inputDuringOutputErrorString);

  xmlInitialize();

  if (!impl)
    impl = dom::DOMImplementationRegistry::getDOMImplementation(XS("LS"));
  parser = ((dom::DOMImplementationLS*)impl)->createDOMBuilder(dom::DOMImplementationLS::MODE_SYNCHRONOUS, 0);

  // set some features on this builder
  if (parser->canSetFeature(dom::XMLUni::fgDOMValidation, true))
    parser->setFeature(dom::XMLUni::fgDOMValidation, true);
  if (parser->canSetFeature(dom::XMLUni::fgDOMNamespaces, true))
    parser->setFeature(dom::XMLUni::fgDOMNamespaces, true);
  if (parser->canSetFeature(dom::XMLUni::fgDOMDatatypeNormalization, true))
      parser->setFeature(dom::XMLUni::fgDOMDatatypeNormalization, true);
  if (parser->canSetFeature(dom::XMLUni::fgDOMComments, true))
    parser->setFeature(dom::XMLUni::fgDOMComments, false);

  // to parse the stream, read the whole thing into a memory buffer
  //  and use a MemBufInputSource (inefficient for large files though)
  array<XByte> buffer(0,4096); // initial size=0, initial capacity=4096
  XByte b;
  std::istream& is( input() );
  while (is.good()) {
    b = is.get();
    buffer.push_back(b);
  }

  dom::MemBufInputSource memInputSrc(buffer.c_array(),buffer.size(),XS("ID"));
  dom::Wrapper4InputSource domInputSrc(&memInputSrc, false);

  dom::DOMDocument *doc = 0;

  try {
    doc = parser->parse(domInputSrc);
  }
  catch (const dom::XMLException& toCatch) {
    char* message = dom::XMLString::transcode(toCatch.getMessage());
    throw base::externalization_error(Exception(String("XML Error:")+message));
  }
  catch (const dom::DOMException& toCatch) {
    char* message = dom::XMLString::transcode(toCatch.msg);
    throw base::externalization_error(Exception(String("XML DOM Error:")+message));
  }

  Assert(doc);
  return doc;
}


void Externalizer::xmlWriteDocument(dom::DOMDocument* doc)
{
  if (isInput()) return;

  xmlInitialize();

  dom::DOMNode* docElem = doc->getDocumentElement();
  xmlFormat(docElem, 1);

  dom::DOMNode* last = docElem->getLastChild();
  if (last)
    if (!isNewline(last))
      appendText(docElem, newline);

  if (!impl)
    impl = dom::DOMImplementationRegistry::getDOMImplementation(XS("LS"));
  dom::DOMWriter* writer = ((dom::DOMImplementationLS*)impl)->createDOMWriter();

  // set some features on this writer
  if (writer->canSetFeature(dom::XMLUni::fgDOMWRTDiscardDefaultContent, true))
    writer->setFeature(dom::XMLUni::fgDOMWRTDiscardDefaultContent, true);

  if (writer->canSetFeature(dom::XMLUni::fgDOMWRTFormatPrettyPrint, true))
    writer->setFeature(dom::XMLUni::fgDOMWRTFormatPrettyPrint, false);


  // output to a memory buffer, then serialize that to the stream
  dom::MemBufFormatTarget memBufTarget;

  try {
    // do the serialization through DOMWriter::writeNode();
    writer->writeNode(&memBufTarget, *doc);
  }
  catch (const dom::XMLException& toCatch) {
    char* message = dom::XMLString::transcode(toCatch.getMessage());
    throw base::externalization_error(Exception(String("XML Error:")+message));
  }
  catch (const dom::DOMException& toCatch) {
    char* message = dom::XMLString::transcode(toCatch.msg);
    throw base::externalization_error(Exception(String("XML DOM Error:")+message));
  }

  // copy to stream
  std::ostream& os( output() );
  const XByte* bytes = memBufTarget.getRawBuffer();
  XByte b = *bytes;
  while (b != 0) {
    os.put(b);
    bytes++;
    b = *bytes;
  }
  os.flush();

}


void Externalizer::xmlReleaseDocument()
{
  if (xmlMode) {

    if (parser) {
      parser->release();
      parser = 0;
      xmldoc = 0;
    }

    xmlMode=0;
  }

}


bool Externalizer::isNewline(dom::DOMNode* n)
{
  if (n->getNodeType() == dom::DOMNode::TEXT_NODE) {
    dom::DOMText* t = dynamic_cast<dom::DOMText*>(n);
    return (dom::XMLString::compareString(t->getData(),
                                          XS(newline)) == 0);
  }
  return false;
}


bool Externalizer::equals(const XCh* str1, const String& str2)
{
  return (dom::XMLString::compareString(str1, XS(str2)) == 0);
}


bool Externalizer::isElement(dom::DOMNode* n, const String& tagname)
{
  if (n->getNodeType() == dom::DOMNode::ELEMENT_NODE) {
    dom::DOMElement* e = dynamic_cast<dom::DOMElement*>(n);
    return equals(e->getTagName(), tagname);
  }
  return false;
}



// traverse the document and insert indent spaces where appropriate
void Externalizer::xmlFormat(dom::DOMNode* n, Int indent)
{
  if (!n->hasChildNodes()) return;

  dom::DOMNode* c = n->getFirstChild();
  do {
    //Debugcln(DJ,String(indent*2,' ')
    //	     << dom::XMLString::transcode(c->getNodeName()) );

    switch (c->getNodeType()) {
    case dom::DOMNode::ELEMENT_NODE: {
      dom::DOMElement* e = dynamic_cast<dom::DOMElement*>(c);
      bool indenting = false;
      dom::DOMNode* first = e->getFirstChild();
      if (first) {
        if (isNewline(first)) {
          // the first child node of this element is a newline
          //   which indicates it is indenting, not inline:
          indenting = true;

          // - add a newline (if needed) and indent space before it
          dom::DOMNode* p = e->getParentNode();
          if (p) {
            dom::DOMNode* prev = e->getPreviousSibling();
            if (prev)
              if (!isNewline(prev))
                p->insertBefore( createText(newline), e );
            p->insertBefore( createText(String(indent*indentSpaces, ' ')), e );
          }



          // - add a newline (if needed) and indent space as the last nodes to
          //   indent the close tag.  If there is only one newline child,
          //   remove it
          dom::DOMNode* last = e->getLastChild();
          if (last->isSameNode(first) && isNewline(last))
            e->removeChild(last);
          else {
            // recursive call
            xmlFormat(c,indent+1);

            last = e->getLastChild();
            if (!isNewline(last))
              appendText(e, newline);
            appendText(e, String(indent*indentSpaces, ' ') );


            // - add a newline after (so it is after the closetag)
            dom::DOMNode* p = e->getParentNode();
            if (p) {
              dom::DOMNode* next = e->getNextSibling();
              if (next)
                p->insertBefore( createText(newline), next );
              else
                appendText(p, newline );
            }
          }

        }
      }


      // if there is a newline before this element, indent it
      dom::DOMNode* prev = e->getPreviousSibling();
      if (prev)
        if (isNewline(prev)) {
          dom::DOMNode* p = e->getParentNode();
          if (p)
            p->insertBefore( createText(String(indent*indentSpaces, ' ')), e );
        }



      if (!indenting)
        xmlFormat(c,indent);

    } break;

    case dom::DOMNode::TEXT_NODE: {
      // if there is a newline before this node, indent it (if it isn't a newline itself)
      if (!isNewline(c)) {
        dom::DOMText* t = dynamic_cast<dom::DOMText*>(c);
        dom::DOMNode* prev = t->getPreviousSibling();
        if (prev) {
          if (isNewline(prev)) {
            dom::DOMNode* p = t->getParentNode();
            if (p)
              p->insertBefore( createText(String(indent*indentSpaces, ' ')), t );
          }
        }

        // if the text contains a newline, split it
        // i.e. before: t='aa\nbb\ncc' after: t='aa' t2='\n' t3='bb\ncc'
        const XCh* text = t->getData();
        SInt newlineIndex = dom::XMLString::indexOf(text,XS(newline)[0]);
        if (newlineIndex != -1) {
          if (newlineIndex != SInt(t->getLength())-1) {
            t->splitText(newlineIndex);
            dom::DOMText* newtext = dynamic_cast<dom::DOMText*>(t->getNextSibling());
            newtext->deleteData(0,1); // remove the leading newline
            // put a newline and indent spaces between them
            dom::DOMNode* p = t->getParentNode();
            if (p)
              p->insertBefore(createText(newline), newtext );
          }
          else
            t->deleteData(t->getLength()-1,1);
        }
      }

      xmlFormat(c,indent);
    } break;

    // these nodes always occur on a new line
    case dom::DOMNode::CDATA_SECTION_NODE:
    case dom::DOMNode::COMMENT_NODE: {

      // add a newline before this node (if necessary) and indent it
      dom::DOMNode* prev = c->getPreviousSibling();
      dom::DOMNode* p = c->getParentNode();
      if (prev) {
        if (!isNewline(prev)) {
          if (p) p->insertBefore( createText(newline), c);
        }

        if (p)
          p->insertBefore( createText(String(indent*indentSpaces, ' ')), c );
      }

      // add a newline after too
      dom::DOMNode* next = c->getNextSibling();
      if (next) {
        if (p) p->insertBefore( createText(newline), next );
      }
      else
        p->appendChild( createText(newline) );


    } break;

    default: ;
    }

    c = c->getNextSibling();
  } while(c);

}






void Externalizer::abort(const String& exceptionString)
{
  if (!aborted) {
    aborted=true;
    Logln(exceptionString);
    throw externalization_error(Exception(exceptionString));
  }
}
