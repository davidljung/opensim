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
  
  $Id: ManipulatorJointTrajectory.cpp 1039 2004-02-11 20:50:52Z jungd $
  $Revision: 1.4 $
  $Date: 2004-02-11 15:50:52 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <robot/ManipulatorJointTrajectory>

#include <sstream>

#include <base/Math>
#include <base/Externalizer>
#include <base/Serializer>
#include <base/externalization_error>


using robot::ManipulatorJointTrajectory;

using base::Vector;
using base::vectorRange;
using base::Range;
using base::Time;
using base::Serializer;
using base::Externalizer;
using base::externalization_error;
using base::XS;
using base::dom::DOMNode;
using base::dom::DOMElement;


ManipulatorJointTrajectory::ManipulatorJointTrajectory(Int numJoints, AngularUnits units)
  : angUnits(units)
{
  const Int dof = numJoints;
  qs.resize(2);
  qs[0].reset( zeroVector(dof) );
  qs[1].reset( zeroVector(dof) );
  
  times.resize(2);
  times[0] = 0;
  times[1] = 1;

  computeSis(dof);
}


ManipulatorJointTrajectory::ManipulatorJointTrajectory(const ManipulatorJointTrajectory& t)
  : qs(t.qs), times(t.times), si(t.si), angUnits(t.angUnits)
{
}


ManipulatorJointTrajectory::ManipulatorJointTrajectory(const Vector& sq, const Time& st, 
						       const Vector& eq, const Time& et,
						       AngularUnits units)
  : angUnits(units)
{
  Int dof = sq.size();
  Assert( dof > 0 );

  qs.resize(2);
  qs[0].reset(sq);
  qs[1].reset(eq);

  times.resize(2);
  times[0] = st;
  times[1] = et;

  computeSis(dof);
}


ManipulatorJointTrajectory::ManipulatorJointTrajectory(const array<Vector>& qs, 
						       const array<Time>& times, 
						       bool deltas, AngularUnits units)
  : angUnits(units)
{
  init(qs, times, deltas);
}


void ManipulatorJointTrajectory::convertComponentUnits(array<Int> components, AngularUnits units)
{
  if (getAngularUnits() == units) return; // nothing to do

  const Int dof = qs[0].size();
  for(Int i=0; i<qs.size(); i++) {
    for(Int ci=0; ci<components.size(); ci++) {
      Int c(components[ci]);
      if (c<dof) {
	qs[i][c] = (units==Radians)?Math::degToRad(qs[i][c]):Math::radToDeg(qs[i][c]);
      }
      else
	throw std::invalid_argument(Exception("component index out of range"));
    }
  }
}


void ManipulatorJointTrajectory::setNumJoints(Int numJoints, bool truncateInitial)
{
  Int currsize = qs[0].size();
  
  if (numJoints < currsize) { // discard elements from end (or start if truncateInitial is true)
    for(Int i=0; i<qs.size(); i++) {
      Vector nq( numJoints );
      if (!truncateInitial)
        nq = vectorRange(qs[i], Range(0, numJoints) );
      else
        nq = vectorRange(qs[i], Range(currsize-numJoints, currsize));
      qs[i].reset(nq);
    }
  }
  else if (numJoints > currsize) { // add zero elements on end
    const Int currentNum = qs[0].size();
    for(Int i=0; i<qs.size(); i++) {
      Vector nq( zeroVector(numJoints) );
      vectorRange(nq, Range(0, currentNum)) = qs[i];
      qs[i].reset(nq);
    }
  }

}



Vector ManipulatorJointTrajectory::q(const Time& t) const 
{
  Real s = gets(t);
  Int i = findIndex(s);
  if (Math::equals(s,si[i])) // fast path if s corresponds to a waypoint
    return qs[i];

  Vector v( qs[i+1] - qs[i] );
  Real ts( si[i+1] - si[i] ); // change in s between qs[i] to qs[i+1]
  
  Real ds = s-si[i];
  Vector dv( (ds/ts)*v );

  return qs[i] + dv;
}


Time ManipulatorJointTrajectory::time(Real s) const
{
  Math::bound<Real>(s,0,1);
  return Time( times[0].seconds() +  ( (times[times.size()-1] - times[0]).seconds() * s ) );
}


void ManipulatorJointTrajectory::shiftTime(const Time& dt)
{
  for(Int i=0; i<times.size(); i++)
    times[i] += dt;
}


void ManipulatorJointTrajectory::scaleTime(Real s)
{
  for(Int i=0; i<times.size(); i++)
    times[i] *= s;
}


Real ManipulatorJointTrajectory::gets(const Time& t) const
{
  const Time& st(times[0]);
  const Time& et(times[times.size()-1]);
  Time bt(t);
  Math::bound<Time>(bt,st,et);
  return (bt-st).seconds() / (et-st).seconds();
}

void ManipulatorJointTrajectory::serialize(Serializer& s)
{
  s(qs,"qs");
  s(times,"times");
  s(si,"si");
}


bool ManipulatorJointTrajectory::formatSupported(String format, Real version, ExternalizationType type) const
{
  return    (format == "txt") && (version==1.0) 
         || (format == "xml") && (version==1.0);
}


void ManipulatorJointTrajectory::externalize(Externalizer& e, String format, Real version)
{
  if (format=="") format = String("xml");

  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" "+base::realToString(version)+" unsupported"));

  if (format == "txt") {

    if (e.isInput()) {
      array<Vector> qs;
      array<Time> times;
      bool first = true;
      bool deltas = false;
      
      while (e.moreInput()) {
	
	String line(e.readLine());
	if (line.empty()) break; // empty line or eof indicates end of list
	while (line[0] == '#') line = e.readLine(); // skip comment lines
	
	if (first) {
	  if (!( (line.substr(0,8) == "absolute") || (line.substr(0,8) == "relative")))
	    throw std::invalid_argument(Exception(String("joint waypoint text file must indicate 'absolute' or 'relative'")));
	  
	  deltas = (line.substr(0,8) == String("relative"));
	  first = false;
	}
	else {
	  // count separating spaces to determine dof
	  Int spaces=0;
	  for(Int i=0; i<line.size(); i++) 
	    if (line[i]==' ') { 
	      spaces++;
	      while ( (i<line.size()-1) && (line[i]==' ')) i++;
	    }
	  if (line[0]==' ') spaces--; // don't count leading space(s)
	  
	  std::istringstream iss(line);
	  iss.setf(std::ios_base::skipws | std::ios_base::dec);
	  
	  const Int dof = spaces + 1 - 1; // don't count time at end as a dof
	  
	  Vector q(dof);
	  
	  for(Int i=0; i<dof; i++)
	    iss >> q[i];
	  
	  Real t;
	  iss >> t;
	  
	  qs.push_back(q);
	  
	  times.push_back(Time(t));
	}
      }
      
      if (first)
	throw std::invalid_argument(Exception(String("joint waypoint text file contains no data.")));
      
      if (qs.size() < 2)
	throw std::invalid_argument(Exception(String("joint waypoint text file must contain at least 2 'waypoint' vectors")));
      
      init(qs,times,deltas);
      
    }
    else { // output
      std::ostringstream out;
      
      // comment header
      out << "# Joint waypoint trajectory [joint positions & times (at-end)]" << std::endl;
      out << "# the following line should be 'absolute' for absolute qi's & t values or 'relative' if the vectors represent inter-waypoint deltas dqi's & dt" << std::endl;
      out << "absolute" << std::endl; // not "relative"
      out << "# q0 q1 ... qn t" << std::endl;
      std::ostream::fmtflags savedFlags = out.setf(std::ios_base::dec | std::ios_base::right);
      Int savedPrec = out.precision(10);
      Int savedWidth = out.width(13);
      const Int dof=qs[0].size();

// this is a hack, as under gcc the flags don't appear to stay set after an output op (!?)
#define setflags \
      out.setf(std::ios_base::dec | std::ios_base::right); \
      out.precision(10); \
      out.width(13); 

      setflags;
      for(Int i=0; i<qs.size(); i++) {
	Vector q(qs[i]);
	Time t(times[i]);
	
	for(Int e=0; e<dof; e++)
	  out << q[e] << " "; setflags;
	
	out << t.seconds(); setflags;
	out << std::endl;
	
      }
      
      out.setf(savedFlags);
      out.precision(savedPrec);
      out.width(savedWidth);
      
      e.writeString(out.str());
    }
    
  } else if (format == "xml") {

    if (e.isOutput()) {

      DOMElement* trajElem = e.createElement("jointtrajectory");

      e.setElementAttribute(trajElem,"pointtype","absolute");
      e.setElementAttribute(trajElem,"angunit", (angUnits==Radians)?String("rad"):String("deg"));

      e.appendComment(trajElem, "space separated joint parameter values and time (secs) at end");

      const Int dof=qs[0].size();
      
      for(Int i=0; i<qs.size(); i++) {
	Vector v(dof + 1);
	vectorRange(v, Range(0,dof)) = qs[i];
	v[dof] = times[i].seconds();

	e.appendText( trajElem, e.toString(v) );
	e.appendBreak( trajElem );
      }

      e.appendElement(trajElem);

    }
    else { // input
      
      DOMNode* context = e.context();
      
      DOMElement* trajElem = e.getFirstElement(context, "jointtrajectory");

      bool absolute = ( e.getDefaultedElementAttribute(trajElem, "pointtype", "absolute") == "absolute" );
      angUnits = ( e.getDefaultedElementAttribute(trajElem, "angunit", "rad") == "deg" )?Degrees:Radians;

      array<Vector> qs;
      array<Time> times;

      String trajText = e.getContainedText(trajElem,true);
      array<String> trajLines = e.splitIntoLines(trajText);

      if (trajLines.size() < 2)
	throw externalization_error(Exception("trajectory must contain at least two points"));

      Int dof;
      for(Int i=0; i<trajLines.size(); i++) {

	array<String> elts = e.splitAtDelimiter(trajLines[i], ' ');
	if (elts.size() < 2)
	  throw externalization_error(Exception("trajectory point with no joints encountered"));
        else {
          if (i==0)
            dof=elts.size()-1;
          else
            if (elts.size()-1 != dof)
              throw externalization_error(Exception("trajectory point of incorrect dimension encountered"));
        }

	Vector v( e.stringsToReals(elts) );

	qs.push_back( Vector( vectorRange(v, Range(0, v.size()-1)) ) );
	times.push_back( Time(v[v.size()-1]) );
      }

      e.removeElement(trajElem);

      init(qs,times,!absolute);
      
    }
    
    
  } // format xml
  
}



void ManipulatorJointTrajectory::init(const array<base::Vector>& qs, const array<base::Time>& times, bool deltas)
{
  Assert(qs.size() > 0);
  Assert(qs.size() == times.size());
  Int dof=qs[0].size();

  if (!deltas) {
    this->qs=qs;
    this->times = times;
#ifdef DEBUG
  // make sure all the Vectors are the same size
  for(Int i=0; i<qs.size(); i++) {
    Assert(qs[i].size() == dof);
  }    
#endif
  }
  else {
    // convert sequence of deltas into absolute joint-configurations and times
    array<Vector> aqs(qs.size()+1);
    aqs[0].reset( zeroVector(dof) );
    for(Int i=0; i<qs.size(); i++) {
      Assert(qs[i].size() == dof);
      aqs[i+1].reset( aqs[i]+qs[i] );
    }

    array<Time> atimes(times.size()+1);
    atimes[0] = 0;
    for(Int i=0; i<times.size(); i++)
      atimes[i+1] = atimes[i]+times[i];

    this->qs = aqs;
    this->times = atimes;
  }

  computeSis(dof);
}


void ManipulatorJointTrajectory::computeSis(Int dof)
{
  // ensure at least two points
  if (qs.size() == 0) { qs.at(0) = qs.at(1) = Vector(dof); }
  if (times.size() == 0) { times.at(0) = 0; times.at(1) = 1; }

  // s's map [0..1] to the range [start-time..end-time]
  Real T = (times[times.size()-1] - times[0]).seconds();
  Int i = 1;
  si.resize(qs.size());
  si[0] = 0;
  while (i<qs.size()) {
    if (times[i-1] > times[i])
      throw std::invalid_argument(Exception("times must be monotonically increasing"));
    si[i] = (times[i] - times[0]).seconds() / T;
    i++;
  }
}


Int ManipulatorJointTrajectory::findIndex(Real s) const
{
  Int i;
  for(i=0; i<si.size(); i++)
    if (si[i] > s) break;
  return i-1;
}
