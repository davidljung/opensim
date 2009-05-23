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
  
  $Id: Path.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.8 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/Path>

#include <sstream>

#include <base/PathRep>
#include <base/Externalizer>
#include <base/externalization_error>
#include <base/Application>
#include <base/VFile>

// concrete representations
#include <base/LineSegPathRep>
#include <base/WaypointPathRep>
#include <base/ParametricPathRep>


using base::Path;

using base::LineSegPathRep;
using base::WaypointPathRep;
using base::ParametricPathRep;

using base::externalization_error;
using base::Application;
using base::VFile;
using base::PathName;

using base::dom::DOMNode;
using base::dom::DOMElement;




Path::Path()
{
  create();
}


Path::Path(const Path& p)
{
  create(p);
}


Path::Path(const Point3& sp, const Orient& so, const Point3& ep, const Orient& eo)
{
  create(sp,so,ep,eo);
}


Path::Path(const array<Point3>& points, const array<Orient>& orients, bool deltas)
{
  create(points,orients,deltas);
}


Path::Path(const array<Vector>& points, bool deltas)
{
  create(points,deltas);
}


Path::Path(const ExpressionVector& p)
{
  create(p);
}



void Path::create()
{
  // default path (degenerate path - create as line segment with both ends the same)
  rep = ref<PathRep>(NewObj LineSegPathRep(Point3(),Orient(),Point3(),Orient()));
}

void Path::create(const Path& p)
{
  rep = ref<PathRep>( &base::clone( *p.rep ) );
}

void Path::create(const Point3& sp, const Orient& so, const Point3& ep, const Orient& eo)
{
  rep = ref<PathRep>(NewObj LineSegPathRep(sp,so,ep,eo));
}

void Path::create(const array<Point3>& points, const array<Orient>& orients, bool deltas)
{
  init(points, orients, deltas);
}

void Path::create(const array<Vector>& points, bool deltas)
{
  // convert points to arrays of Point3 & Orients
  Assert(points.size() > 0);
  Assert(points[0].size() >= 3);
  
  array<Point3> pos(points.size());
  for(Int i=0; i<points.size(); i++) {
    const Vector& v(points[i]);
    pos[i] = Point3(v[0],v[1],v[2]);
  }
  
  array<Orient> orient(points.size());
  if (points[0].size() == 6) { // assuming EulerRPY
    for(Int i=0; i<points.size(); i++) {
      const Vector& v(points[i]);
      orient[i] = Orient(v[3],v[4],v[5]);
    }
  }
  else
    if (points[0].size() == 7) { // assuming quat
      for(Int i=0; i<points.size(); i++) {
	const Vector& v(points[i]);
	orient[i] = Orient(Quat4(v[3],v[4],v[5],v[6]));
      }
    }
  
  init(pos,orient,deltas);
}


void Path::create(const ExpressionVector& p)
{
  rep = ref<PathRep>(NewObj ParametricPathRep(p));
}


void Path::init(const array<Point3>& points, const array<Orient>& orients, bool deltas)
{
  if (!deltas)
    rep = ref<PathRep>(NewObj WaypointPathRep(points, orients));
  else {
    array<Point3> apoints(points.size()+1);
    apoints[0] = Point3(0,0,0);
    for(Int i=0; i<points.size(); i++)
      apoints[i+1] = apoints[i]+points[i];

    array<Orient> aorients(orients.size()+1);
    aorients[0] = Orient();
    for(Int i=0; i<orients.size(); i++) {
      Quat4 q1(aorients[i].getQuat4());
      Quat4 q2(orients[i].getQuat4());
      aorients[i+1] = Orient(q2*q1);
    }

    rep = ref<PathRep>(NewObj WaypointPathRep(apoints, aorients));
  }
}



void Path::resample(Int samples)
{
  array<Point3> points(samples);
  array<Orient> orients(samples);

  for(Int i=0; i<samples; i++) {
    Real s = Real(i)/Real(samples-1);
    points[i] =  Path::position(s);
    orients[i] = Path::orientation(s);
  }

  init(points, orients, false);
   
}


void Path::resample(const Real dxmax)
{
  Assert(dxmax > 0);
  
  array<Point3> points;
  array<Orient> orients;
  
  // start with first distinguished value
  Real s = distinguishedValue(0);
  Point3 pos( position(s) );
  Orient orient( orientation(s) );
  points.push_back( pos );
  orients.push_back( orient );

  Real lasts(s);
  Point3 lastpos( pos );
  Int i = 1; // current distinguished value index
  while (i < numDistinguishedValues()) {

    // get next distinguished value
    s = distinguishedValue(i);
    pos = position(s);
    orient = orientation(s);
    
    // calc dx = this pos - last pos
    Vector3 dx = pos - lastpos;
    if (dx.length() > dxmax) { // dist was too big, set current pos to a distance of maxdx from last pos
      Real scale = dxmax/dx.length();
      s = lasts + scale*(s-lasts);
      pos = position(s);
      orient = orientation(s);
    }
    else
      ++i;
    
    points.push_back( pos );
    orients.push_back( orient );

    lasts = s;
    lastpos = pos;
  }
  
  init(points, orients, false);
  
}






void Path::serialize(Serializer& s)
{
  // register instantiators for all the PathRep classes for dynamic instantiation upon deserialization
  Serializable::registerSerializableInstantiator<PathRep,LineSegPathRep>(lineSegPathRepInstantiator);
  Serializable::registerSerializableInstantiator<PathRep,WaypointPathRep>(waypointPathRepInstantiator);
  s.baseRef(rep,"PathRep");
}


bool Path::formatSupported(String format, Real version, ExternalizationType type) const
{
  return    ((format=="txt") && (version==1.0))
         || ((format=="xml") && (version==1.0));
}


void Path::externalize(Externalizer& e, String format, Real version)
{
  if (format=="") format = String("xml");

  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" "+base::realToString(version)+" unsupported"));

  if (format == "txt") {

    if (e.isInput()) {
      
      array<Point3> points;
      array<Orient> orients;
      
      while (e.moreInput()) {
	
	String line(e.readLine());
	if (line.empty()) break; // empty line or eof indicates end of list
	while (line[0] == '#') line = e.readLine(); // skip comment lines
	
	// count separating spaces to determine if there are 6 or 7
	// numbers present
	Int spaces=0;
	for(Int i=0; i<line.size(); i++) 
	  if (line[i]==' ') { 
	    spaces++;
	    while ( (i<line.size()-1) && (line[i]==' ')) i++;
	  }
	if (line[0]==' ') spaces--; // don't count leading space(s)
	
	Assert(spaces >= 5);
	bool quat = (spaces>5);
	std::istringstream iss(line);
	iss.setf(std::ios_base::skipws | std::ios_base::dec);
	
	Point3 p;
	iss >> p.x >> p.y >> p.z;
	Vector v(quat?4:3);
	iss >> v[0] >> v[1] >> v[2];
	if (quat) iss >> v[3];

	points.push_back(p);
	
	orients.push_back(Orient(v, quat? Orient::Quat : Orient::EulerRPY));
      }

      init(points,orients,false);
      
    }
    else { // output
      std::ostringstream out;
      
      // comment header
      out << "# waypoint path (positions & orientations)" << std::endl;
      out << "#           x             y             z ";
      if (orientation(distinguishedValue(0)).representation() == Orient::EulerRPY)
	out << "            R             P             Y" << std::endl;
      else
	out << "           qx            qy            qz            qw" << std::endl;
      
      std::ostream::fmtflags savedFlags = out.setf(std::ios_base::dec | std::ios_base::right);
      Int savedPrec = out.precision(10);
      Int savedWidth = out.width(13);
      
      for(Int i=0; i<numDistinguishedValues(); i++) {
	Point3 p(position(distinguishedValue(i)));
	Orient o(orientation(distinguishedValue(i)));
	
// this is a hack, as under gcc the flags don't appear to stay set after an output op (!?)
#define setflags \
      out.setf(std::ios_base::dec | std::ios_base::right); \
      out.precision(10); \
      out.width(13); 

	out << p.x << " "; setflags;
        out << p.y << " "; setflags;
	out << p.z << " "; setflags;
	if ( (o.representation() != Orient::Quat) && (o.representation() != Orient::EulerRPY) )
	  o.changeRepresentation(Orient::Quat);

	Vector v(o);
	out << v[0] << " "; setflags;
	out << v[1] << " "; setflags;
	out << v[2]; setflags;
	if (v.size() > 3 ) out << " " << v[3];
	out << std::endl;

      }

      out.setf(savedFlags);
      out.precision(savedPrec);
      out.width(savedWidth);

      e.writeString(out.str());
    }

  } else if (format == "xml") {

    if (e.isOutput()) {

      DOMElement* pathElem = e.createElement("path");

      bool parametric = instanceof(*rep, ParametricPathRep);
      
      if (parametric) { Unimplemented; }
      
      Orient::Representation rep = orientation(distinguishedValue(0)).representation();
      if ((rep != Orient::Quat) && (rep != Orient::EulerRPY) )
	rep = Orient::Quat;
      
      if (rep==Orient::Quat) {
	e.setElementAttribute(pathElem,"representation", "x y z qx qy qz qw" );
	e.appendComment(pathElem, "position (x,y,z) orientation quaternion (qx,qy,qz,qw) [w is scalar component]");
      } else {
	e.setElementAttribute(pathElem,"representation", "x y z r p y" );
	e.appendComment(pathElem,"position (x,y,z) orientation Euler angles (roll, pitch yaw)");
      }
      
      e.setElementAttribute(pathElem,"pointtype", "absolute");
      if (frame!="")
	e.setElementAttribute(pathElem,"frame", frame);

      if (units!="")
	e.setElementAttribute(pathElem,"units", units);
      
      for(Int i=0; i<numDistinguishedValues(); i++) {
	Point3 p(position(distinguishedValue(i)));
	Orient o(orientation(distinguishedValue(i)));
	
	Vector vo(o);
	Vector v(3+vo.size());
	v[0] = p.x; v[1] = p.y; v[2] = p.z;
	if (vo.size() == 3) { // rpy
	  v[3] = vo[0]; v[4] = vo[1]; v[5] = vo[2];
	}
	else { // quat
	  v[3] = vo[0]; v[4] = vo[1]; v[5] = vo[2]; v[6] = vo[3];
	}
	
	e.appendText( pathElem, e.toString(v) );
	e.appendBreak( pathElem );
	
      }
      
      e.appendElement(pathElem);    

    }
    else { // input
      DOMNode* context = e.context();
      
      DOMElement* pathElem = e.getFirstElement(context, "path");

      // handle link
      String link = e.getElementAttribute(pathElem,"link",false);
      if (link != "") {
	
	ref<VFile> linkFile = Application::getInstance()->universe()->cache()->findFile(link,e.getArchivePath());
	load(linkFile,format,version);
      }
      else {
	
	String repText = e.getElementAttribute(pathElem, "representation");
	
	bool absolute = ( e.getDefaultedElementAttribute(pathElem, "pointtype", "absolute") == "absolute" );
	frame = e.getDefaultedElementAttribute(pathElem, "frame", "");
	units = e.getDefaultedElementAttribute(pathElem, "units", "");

	Orient::Representation rep;
        bool hasorient = true;
        bool parametric = false;
	bool scalarFirst = false;
        
        if (repText == "x y z") {
          hasorient = false;
        }
	else if (repText == "x y z qx qy qz qw") {
	  rep = Orient::Quat;
        }
	else if (repText == "x y z qw qx qy qz") {
	  rep = Orient::Quat;
	  scalarFirst=true;
	}
	else if (repText == "x y z r p y") {
	  rep = Orient::EulerRPY;
        }
        else if (repText == "x(s) y(s) z(s)") {
          hasorient = false;
          parametric = true;
        }
        else if (repText == "x(s) y(s) z(s) qx(s) qy(s) qz(s) qw(s)") {
          parametric = true;
	  rep = Orient::Quat;
        }
        else if (repText == "x(s) y(s) z(s) qw(s) qx(s) qy(s) qz(s)") {
          parametric = true;
	  rep = Orient::Quat;
	  scalarFirst=true;
        }
        else if (repText == "x(s) y(s) z(s) r(s) p(s) y(s)") {
          parametric = true;
	  rep = Orient::EulerRPY;
        }
	else 
	  throw externalization_error(Exception("unknown or unsupported path representation"));

        String pathText = e.getContainedText(pathElem,true);
        array<String> pathLines = e.splitIntoLines(pathText);

        if (!parametric) {	
          array<Point3> points;
          array<Orient> orients;
          
  
          if (pathLines.size() < 2)
            throw externalization_error(Exception("path must contain at least two points"));
  
          for(Int i=0; i<pathLines.size(); i++) {
            array<String> elts = e.splitAtDelimiter(pathLines[i], ' ');
            if (    (hasorient && (rep==Orient::Quat) && (elts.size() != 7))
                 || (hasorient && (rep==Orient::EulerRPY) && (elts.size() != 6)) 
                 || (!hasorient && (elts.size() != 3)) )
              throw externalization_error(Exception("path point with wrong number of elements encountered"));
            Vector v( e.stringsToReals(elts) );
            points.push_back( Point3(v[0],v[1],v[2]) );
            if (hasorient) {
              if (rep==Orient::Quat) {
                if (!scalarFirst)
                  orients.push_back( Orient(Quat4(v[3],v[4],v[5],v[6])) );
                else
                  orients.push_back( Orient(Quat4(v[6],v[3],v[4],v[5])) );
              }
              else if (rep==Orient::EulerRPY) {
                orients.push_back( Orient(v[3],v[4],v[5]) );
              }
            }
            else
              orients.push_back( Orient() );
            
          }
  
          init(points,orients,!absolute);
        }
        else { // parametric
          
          if (!absolute)
            throw externalization_error(Exception("pointtype of 'relative' doesn't make sense for parametric expression paths"));
          
          ExpressionVector v(3);

          try {
          
            if (!hasorient) {
              if (pathLines.size() != 3)
                throw externalization_error(Exception("path should have one line for each parametric expression x(s), y(s) and z(s)"));
              v[0] = Expression(pathLines[0]);
              v[1] = Expression(pathLines[1]);
              v[2] = Expression(pathLines[2]);
            }
            else { // hasorient
              if (rep == Orient::Quat) {
                if (pathLines.size() != 7)
                  throw externalization_error(Exception("path should have one line for each parametric expression x(s), y(s), z(s), qx(s), qy(s), qz(s) and qw(s)"));
                v.resize(7);
                v[0] = Expression(pathLines[0]);
                v[1] = Expression(pathLines[1]);
                v[2] = Expression(pathLines[2]);
                if (!scalarFirst) {
                  v[3] = Expression(pathLines[3]);
                  v[4] = Expression(pathLines[4]);
                  v[5] = Expression(pathLines[5]);
                  v[6] = Expression(pathLines[6]);
                }
                else {
                  v[3] = Expression(pathLines[6]);
                  v[4] = Expression(pathLines[3]);
                  v[5] = Expression(pathLines[4]);
                  v[6] = Expression(pathLines[5]);
                }
              }
              else {
                if (pathLines.size() != 6)
                  throw externalization_error(Exception("path should have one line for each parametric expression x(s), y(s), z(s), R(s), P(s), and Y(s)"));
                v.resize(6);
                v[0] = Expression(pathLines[0]);
                v[1] = Expression(pathLines[1]);
                v[2] = Expression(pathLines[2]);
                v[3] = Expression(pathLines[3]);
                v[4] = Expression(pathLines[4]);
                v[5] = Expression(pathLines[5]);
              }
            }
          } catch (std::exception& e) {
            throw externalization_error(Exception("error parsing parametric path component expression:"+String(e.what())));
          }
          
          this->rep = ref<PathRep>(NewObj ParametricPathRep(v));
          
        } // else parametric

      }

      e.removeElement(pathElem);

    }

  } // format xml


}

