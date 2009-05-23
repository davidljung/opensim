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
  
  $Id: Trajectory.cpp 1047 2004-02-27 19:21:35Z jungd $
  $Revision: 1.7 $
  $Date: 2004-02-27 14:21:35 -0500 (Fri, 27 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/Trajectory>

#include <sstream>

#include <base/TrajectoryTimeRep>
#include <base/Externalizer>
#include <base/externalization_error>
#include <base/Application>
#include <base/VFile>

// concrete representations
#include <base/LineSegTrajectoryRep>
#include <base/WaypointTrajectoryRep>
#include <base/ParametricTrajectoryRep>


using base::Trajectory;

using base::LineSegTrajectoryRep;
using base::WaypointTrajectoryRep;
using base::ParametricTrajectoryRep;

using base::externalization_error;
using base::Application;
using base::VFile;
using base::PathName;

using base::dom::DOMNode;
using base::dom::DOMElement;



Trajectory::Trajectory()
{
  create();
}


Trajectory::Trajectory(const Trajectory& t)
{
  create(t);
}

Trajectory::Trajectory(const Path& p, Int samples)
{
  // extract the path in a general way if possible
  if (p.numDistinguishedValues() > 2) {

    array<Point3> points(p.numDistinguishedValues());
    array<Orient> orients(p.numDistinguishedValues());
    array<Time>   times(p.numDistinguishedValues());

    for(Int i=0; i<p.numDistinguishedValues(); i++) {
      Real s = p.distinguishedValue(i);
      points[i] = p.position(s);
      orients[i] = p.orientation(s);
      times[i] = s;
    }

    create(points, orients, times, false);
  }
  else {

    array<Point3> points(samples);
    array<Orient> orients(samples);
    array<Time>   times(samples);

    for(Int i=0; i<samples; i++) {
      Real s = Real(i)/Real(samples-1);
      points[i] = p.position(s);
      orients[i] = p.orientation(s);
      times[i] = s;
    }

    create(points, orients, times, false);
    
  }
}


Trajectory::Trajectory(const Point3& sp, const Orient& so, const Time& st, 
		       const Point3& ep, const Orient& eo, const Time& et)
{
  create(sp,so,st,ep,eo,et);
}


Trajectory::Trajectory(const array<Point3>& points, const array<Orient>& orients, const array<Time>& times, bool deltas)
{
  create(points, orients, times, deltas);
}


Trajectory::Trajectory(const array<Vector>& points, bool deltas)
{
  create(points,deltas);
}


Trajectory::Trajectory(const ExpressionVector& p)
{
  create(p);
}



void Trajectory::init(const array<Point3>& points, const array<Orient>& orients, const array<Time>& times, bool deltas)
{
  ref<WaypointTrajectoryRep> wrep;

  if (!deltas)
    wrep = ref<WaypointTrajectoryRep>(NewObj WaypointTrajectoryRep(points, orients, times));
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

    array<Time> atimes(times.size()+1);
    atimes[0] = 0;
    for(Int i=0; i<times.size(); i++)
      atimes[i+1] = atimes[i]+times[i];

    wrep = ref<WaypointTrajectoryRep>(NewObj WaypointTrajectoryRep(apoints, aorients, atimes));
  }

  rep = wrep;
  trep = wrep;
}



void Trajectory::create()
{
  // default traj (degenerate traj - create as line segment with both ends the same time)
  ref<LineSegTrajectoryRep> lrep(NewObj LineSegTrajectoryRep());
  rep = lrep;
  trep = lrep;
}

void Trajectory::create(const Trajectory& t)
{
  ref<Referenced> c( &base::clone( *t.trep ) );

  rep = narrow_ref<PathRep>(c);
  trep = narrow_ref<TrajectoryTimeRep>(c);
}

void Trajectory::create(const Point3& sp, const Orient& so, const Point3& ep, const Orient& eo)
{
  ref<LineSegTrajectoryRep> lrep(NewObj LineSegTrajectoryRep(sp, so, 0, ep, eo, 1));
  rep = lrep;
  trep = lrep;
}

void Trajectory::create(const Point3& sp, const Orient& so, const Time& st,
			const Point3& ep, const Orient& eo, const Time& et)
{
  ref<LineSegTrajectoryRep> lrep(NewObj LineSegTrajectoryRep(sp, so, st, ep, eo, et));
  rep = lrep;
  trep = lrep;
}

void Trajectory::create(const array<Point3>& points, const array<Orient>& orients, bool deltas)
{
  // init default times to range over [0..1]
  array<Time> times(points.size());
  for(Int i=0; i<times.size(); i++)
    times[i] = Time( i/times.size() );

  init(points,orients,times,false);
}

void Trajectory::create(const array<Point3>& points, const array<Orient>& orients, const array<Time>& times, bool deltas)
{
  init(points,orients,times,deltas);
}

void Trajectory::create(const array<Vector>& points, bool deltas)
{
  // convert points to arrays of Point3, Orients & Times
  Assert(points.size() > 0);
  Assert(points[0].size() >= 4);

  array<Point3> pos(points.size());
  array<Time>   time(points.size());
  for(Int i=0; i<points.size(); i++) {
    const Vector& v(points[i]);
    pos[i] = Point3(v[0],v[1],v[2]);
    time[i] = Time(v[3]);
  }
  
  array<Orient> orient(points.size());
  if (points[0].size() == 7) { // assuming EulerRPY
    for(Int i=0; i<points.size(); i++) {
      const Vector& v(points[i]);
      orient[i] = Orient(v[3],v[4],v[5]);
      time[i] = Time(v[6]);
    }
  }
  else
    if (points[0].size() == 8) { // assuming quat
      for(Int i=0; i<points.size(); i++) {
	const Vector& v(points[i]);
	orient[i] = Orient(Quat4(v[3],v[4],v[5],v[6]));
	time[i] = Time(v[7]);
      }
    }

  init(pos,orient,time,deltas);
}


/// \todo implement
void Trajectory::create(const ExpressionVector& p)
{
  Unimplemented;
}



base::Path Trajectory::toPath() const
{
  return Path(*this);
}


void Trajectory::resample(Int samples)
{
  array<Point3> points(samples);
  array<Orient> orients(samples);
  array<Time>   times(samples);

  Time start = time(0);
  Time end = time(1);
  Time duration = end-start;

  for(Int i=0; i<samples; i++) {
    Real s = Real(i)/Real(samples-1);
    points[i] =  Path::position(s);
    orients[i] = Path::orientation(s);
    times[i] = start + duration*s;
  }

  init(points, orients, times, false);
   
}


void Trajectory::resample(const Real dxmax)
{
  Assert(dxmax > 0);
  
  array<Point3> points;
  array<Orient> orients;
  array<Time>   times;
  
  // start with first distinguished value
  Time t = time( distinguishedValue(0) );
  Point3 pos( position(t) );
  Orient orient( orientation(t) );
  points.push_back( pos );
  orients.push_back( orient );
  times.push_back( t );

  Time lasttime( t );
  Point3 lastpos( pos );
  Int i = 1; // current distinguished value index
  while (i < numDistinguishedValues()) {

    // get next distinguished value
    t = time(distinguishedValue(i) );
    pos = position(t);
    orient = orientation(t);
    
    // calc dx = this pos - last pos
    Vector3 dx = pos - lastpos;
    if (dx.length() > dxmax) { // dist was too big, set current pos to a distance of maxdx from last pos
      Real scale = dxmax/dx.length();
      t = lasttime + scale*(t-lasttime);
      pos = position(t);
      orient = orientation(t);
    }
    else
      ++i;
    
    points.push_back( pos );
    orients.push_back( orient );
    times.push_back( t );

    lasttime = t;
    lastpos = pos;
  }
  
  init(points, orients, times, false);
  
}

  
  

void Trajectory::serialize(Serializer& s)
{
  // register instantiators for all the TrajectoryRep classes for dynamic instantiation upon deserialization
  Serializable::registerSerializableInstantiator<TrajectoryTimeRep,LineSegTrajectoryRep>(lineSegTrajectoryRepInstantiator);
  Serializable::registerSerializableInstantiator<TrajectoryTimeRep,WaypointTrajectoryRep>(waypointTrajectoryRepInstantiator);
  
  s.baseRef(trep,"TrajectoryTimeRep");
}


bool Trajectory::formatSupported(String format, Real version, ExternalizationType type) const
{
  return    ((format == "txt") && (version==1.0))
         || ((format == "xml") && (version==1.0));
}


void Trajectory::externalize(Externalizer& e, String format, Real version)
{
  if (format=="") format = String("xml");

  if (!formatSupported(format,version,e.ioType()))
    throw std::invalid_argument(Exception(String("format ")+format+" "+base::realToString(version)+" unsupported"));

  if (format == "txt") {

    if (e.isInput()) {
      
      array<Point3> points;
      array<Orient> orients;
      array<Time> times;
      
      while (e.moreInput()) {
	
	String line(e.readLine());
	if (line.empty()) break; // empty line or eof indicates end of list
	while (line[0] == '#') line = e.readLine(); // skip comment lines
	
	// count separating spaces to determine if there are 7 or 8
	// numbers present
	Int spaces=0;
	for(Int i=0; i<line.size(); i++) 
	  if (line[i]==' ') { 
	    spaces++;
	    while ( (i<line.size()-1) && (line[i]==' ')) i++;
	  }
	if (line[0]==' ') spaces--; // don't count leading space(s)
	
	Assert(spaces >= 6);
	bool quat = (spaces>6);
	std::istringstream iss(line);
	iss.setf(std::ios_base::skipws | std::ios_base::dec);
	
	Point3 p;
	iss >> p.x >> p.y >> p.z;
	Vector v(quat?4:3);
	iss >> v[0] >> v[1] >> v[2];
	if (quat) iss >> v[3];
	Real t;
	iss >> t;
	
	points.push_back(p);
	
	orients.push_back(Orient(v, quat? Orient::Quat : Orient::EulerRPY));
	
	times.push_back(Time(t));
      }
      
      init(points,orients,times,false);
      
    }
    else { // output
      std::ostringstream out;
      
      // comment header
      out << "# waypoint trajectory (positions, orientations & times)" << std::endl;
      out << "#           x             y             z ";
      if (orientation(distinguishedValue(0)).representation() == Orient::EulerRPY)
	out << "            R             P             Y" << std::endl;
      else
	out << "           qx            qy            qz            qw            t" << std::endl;
      
      std::ostream::fmtflags savedFlags = out.setf(std::ios_base::dec | std::ios_base::right);
      Int savedPrec = out.precision(10);
      Int savedWidth = out.width(13);
      
      for(Int i=0; i<numDistinguishedValues(); i++) {
	Point3 p(position(distinguishedValue(i)));
	Orient o(orientation(distinguishedValue(i)));
	Time t(time(distinguishedValue(i)));
	
	// this is a hack, as under gcc the flags don't appear to stay set after an output op (!?)
#define setflags \
      out.setf(std::ios_base::dec | std::ios_base::right); \
      out.precision(10); \
      out.width(13); 
	
	out << p.x << " "; setflags
	out << p.y << " "; setflags
	out << p.z << " "; setflags
	if ( (o.representation() != Orient::Quat) && (o.representation() != Orient::EulerRPY) )
	  o.changeRepresentation(Orient::Quat);
	
	Vector v(o);
	out << v[0] << " "; setflags
	out << v[1] << " "; setflags
	out << v[2]; setflags
	if (v.size() > 3 ) out << " " << v[3];
	out << t.seconds(); setflags;
	out << std::endl;
	
      }

      out.setf(savedFlags);
      out.precision(savedPrec);
      out.width(savedWidth);
      
      e.writeString(out.str());
    }
  } 
  else if (format == "xml") {

    if (e.isOutput()) {

      DOMElement* trajElem = e.createElement("trajectory");

      bool parametric = instanceof(*rep, ParametricTrajectoryRep);
      
      if (parametric) { // special case for parametric representation to retain symbolic expressions
       
        // dig into the parametric representation and output the actual Expression strings
        ref<ParametricTrajectoryRep> trep(narrow_ref<ParametricTrajectoryRep>(rep));
        ExpressionVector v(trep->v);
        Expression times(trep->times);
        bool hasorient = (v.size() > 3);
        bool quatrep = false;
        if (hasorient && v.size() > 6) quatrep = true;
        
        if (!hasorient) {
          e.setElementAttribute(trajElem,"representation", "x(s) y(s) z(s) t(s)");
          e.appendComment(trajElem, "position functions x(s),y(s),z(s) and time t(s) [one per line]");
        }
        else {
          if (quatrep) {
            e.setElementAttribute(trajElem,"representation", "x(s) y(s) z(s) qx(s) qy(s) qz(s) qw(s) t(s)");
            e.appendComment(trajElem,"position functions x(s),y(s),z(s) orientation quaternion element functions (qx, qy, qz, qw) and time(s) (secs) [one per line; w is scalar]");
          }
          else {
            e.setElementAttribute(trajElem,"representation", "x(s) y(s) z(s) r(s) p(s) y(s) t(s)");
            e.appendComment(trajElem,"position functions x(s),y(s),z(s) orientation Euler angle functions (roll, pitch yaw) and time(s) (secs) [one per line]");
          }
        }
        
        
        for (Int i=0; i<v.size(); i++) {
          e.appendText( trajElem, v[i].toString() );
          e.appendBreak( trajElem);
        }
        e.appendText( trajElem, times.toString() );
        e.appendBreak( trajElem);
        
      }
      else { // output distinguished points as waypoints
        Orient::Representation rep = orientation(distinguishedValue(0)).representation();
        if ((rep != Orient::Quat) && (rep != Orient::EulerRPY) )
          rep = Orient::Quat;
  
        if (rep==Orient::Quat) {
          e.setElementAttribute(trajElem,"representation", "x y z qx qy qz qw t" );
          e.appendComment(trajElem, "position (x,y,z) orientation quaternion (qx,qy,qz,qw) [w is scalar component] time (secs)");
        } else {
          e.setElementAttribute(trajElem,"representation", "x y z r p y t" );
          e.appendComment(trajElem,"position (x,y,z) orientation Euler angles (roll, pitch yaw) time (secs)");
        }
  
        e.setElementAttribute(trajElem,"pointtype", "absolute");
        if (frame!="")
          e.setElementAttribute(trajElem,"frame", frame);
        if (units!="")
          e.setElementAttribute(trajElem,"units", units);
  
        for(Int i=0; i<numDistinguishedValues(); i++) {
          Point3 p(position(distinguishedValue(i)));
          Orient o(orientation(distinguishedValue(i)));
          Time t(time(distinguishedValue(i)));
  
          o.changeRepresentation(rep);
          Vector vo(o);
          Vector v(3+vo.size()+1);
          v[0] = p.x; v[1] = p.y; v[2] = p.z;
          if (vo.size() == 3) { // rpy
            v[3] = vo[0]; v[4] = vo[1]; v[5] = vo[2];
            v[6] = t.seconds();
          }
          else { // quat
            v[3] = vo[0]; v[4] = vo[1]; v[5] = vo[2]; v[6] = vo[3];
            v[7] = t.seconds();
          }
  
          e.appendText( trajElem, e.toString(v) );
          e.appendBreak( trajElem );
  
        }
      }
      
      e.appendElement(trajElem);    

    }
    else { // input
      DOMNode* context = e.context();
      
      // will accept either <trajectory> or <path>

      DOMElement* trajElem = e.getFirstElement(context, "trajectory", false);

      if (trajElem) { // <trajectory>

	// handle link
	String link = e.getElementAttribute(trajElem,"link",false);
	if (link != "") {
	  
	  ref<VFile> linkFile = Application::getInstance()->universe()->cache()->findFile(link,e.getArchivePath());
	  load(linkFile,format,version);
	}
	else {

	  String repText = e.getElementAttribute(trajElem, "representation");
	
	  bool absolute = ( e.getDefaultedElementAttribute(trajElem, "pointtype", "absolute") == "absolute" );
	  frame = e.getDefaultedElementAttribute(trajElem, "frame", "");
	  units = e.getDefaultedElementAttribute(trajElem, "units", "");
	
	  Orient::Representation rep;
          bool hasorient = true;
          bool parametric = false;
	  bool scalarFirst=false;
          
          if (repText == "x y z t") {
            hasorient=false;
          }
	  else if (repText == "x y z qx qy qz qw t") {
	    rep = Orient::Quat;
          }
	  else if (repText == "x y z qw qx qy qz t") {
	    rep = Orient::Quat;
	    scalarFirst=true;
	  }
	  else if (repText == "x y z r p y t") {
	    rep = Orient::EulerRPY;
          }
          else if (repText == "x(s) y(s) z(s) t(s)") {
            hasorient = false;
            parametric = true;
          }
          else if (repText == "x(s) y(s) z(s) qx(s) qy(s) qz(s) qw(s) t(s)") {
            parametric = true;
            rep = Orient::Quat;
          }
          else if (repText == "x(s) y(s) z(s) qw(s) qx(s) qy(s) qz(s) t(s)") {
            parametric = true;
            rep = Orient::Quat;
            scalarFirst=true;
          }
          else if (repText == "x(s) y(s) z(s) r(s) p(s) y(s) t(s)") {
            parametric = true;
            rep = Orient::EulerRPY;
          }
          else            
	    throw externalization_error(Exception("unknown or unsupported trajectory representation"));
	  
	  String trajText = e.getContainedText(trajElem,true);
	  array<String> trajLines = e.splitIntoLines(trajText);

          if (!parametric) {
            array<Point3> points;
            array<Orient> orients;
            array<Time> times;
            
            if (trajLines.size() < 2)
              throw externalization_error(Exception("trajectory must contain at least two points"));
          
            for(Int i=0; i<trajLines.size(); i++) {
              array<String> elts = e.splitAtDelimiter(trajLines[i], ' ');
              if (    ((rep==Orient::Quat) && (elts.size() != 8))
                   || ((rep==Orient::EulerRPY) && (elts.size() != 7)) )
                throw externalization_error(Exception("trajectory point with wrong number of elements encountered"));
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
              
              times.push_back( Time(v[v.size()-1]) );
            }
  
            init(points,orients,times,!absolute);
          }
          else { // parametric
            
            if (!absolute)
              throw externalization_error(Exception("pointtype of 'relative' doesn't make sense for parametric expression trajectories"));
            
            ExpressionVector v(4);
  
            try {
            
              if (!hasorient) {
                if (trajLines.size() != 4)
                  throw externalization_error(Exception("trajectory should have one line for each parametric expression x(s), y(s), z(s) and t(s)"));
                v[0] = Expression(trajLines[0]);
                v[1] = Expression(trajLines[1]);
                v[2] = Expression(trajLines[2]);
                v[3] = Expression(trajLines[3]);
              }
              else { // hasorient
                if (rep == Orient::Quat) {
                  if (trajLines.size() != 8)
                    throw externalization_error(Exception("trajectory should have one line for each parametric expression x(s), y(s), z(s), qx(s), qy(s), qz(s),qw(s) and t(s)"));
                  v.resize(8);
                  v[0] = Expression(trajLines[0]);
                  v[1] = Expression(trajLines[1]);
                  v[2] = Expression(trajLines[2]);
                  if (!scalarFirst) {
                    v[3] = Expression(trajLines[3]);
                    v[4] = Expression(trajLines[4]);
                    v[5] = Expression(trajLines[5]);
                    v[6] = Expression(trajLines[6]);
                  }
                  else {
                    v[3] = Expression(trajLines[6]);
                    v[4] = Expression(trajLines[3]);
                    v[5] = Expression(trajLines[4]);
                    v[6] = Expression(trajLines[5]);
                  }
                  v[7] = Expression(trajLines[7]);
                }
                else {
                  if (trajLines.size() != 7)
                    throw externalization_error(Exception("trajectory should have one line for each parametric expression x(s), y(s), z(s), R(s), P(s), Y(s) and t(s)"));
                  v.resize(7);
                  v[0] = Expression(trajLines[0]);
                  v[1] = Expression(trajLines[1]);
                  v[2] = Expression(trajLines[2]);
                  v[3] = Expression(trajLines[3]);
                  v[4] = Expression(trajLines[4]);
                  v[5] = Expression(trajLines[5]);
                  v[6] = Expression(trajLines[6]);
                }
              }
            } catch (std::exception& e) {
              throw externalization_error(Exception("error parsing parametric trajectory component expression:"+String(e.what())));
            }

            ref<ParametricTrajectoryRep> prep;
            try {
              prep = ref<ParametricTrajectoryRep>(NewObj ParametricTrajectoryRep(v));
            } catch (std::exception& e) {
              throw externalization_error(Exception("error creating parametric trajectory:"+String(e.what())));
            }
            this->rep = prep;
            this->trep = prep;

          } // else parametric
          
        }

	e.removeElement(trajElem);

      }
      else { // !trajElem 

	DOMElement* pathElem = e.getFirstElement(context, "path", false);

	if (pathElem) {

          // In general, to use a Path as a Trajectory without knowing its representation,
          //  we just sample the path and use the sample points while adding a default
          //  time interval of [0..1]
          // However, we handle some special cases of known representations explicitly
          
          // First, load the Path
	  Path path;
	  path.externalize(e, format, version);

	  frame = path.getCoordFrame();
	  units = path.getUnits();

          // check for parametric representation
          if (instanceof(*path.rep, ParametricPathRep)) { // use the PathRep directly
            
            ref<ParametricPathRep> ppathrep( narrow_ref<ParametricPathRep>(path.rep) );
            
            ExpressionVector v( ppathrep->v.size()+1 ); // make room for times Expression and copy other elements over
            for(Int i=0; i<ppathrep->v.size(); i++) v[i] = ppathrep->v[i];
            v[v.size()-1] = Expression::p[0]; // i.e. t = s

            ref<ParametricTrajectoryRep> prep;
            try {
              prep = ref<ParametricTrajectoryRep>(NewObj ParametricTrajectoryRep(v));
            } catch (std::exception& e) {
              throw externalization_error(Exception("error creating parametric trajectory from path:"+String(e.what())));
            }

            this->rep = prep;
            this->trep = prep;

          }
          else { // general case, Resample and use Path's distinguished points.

            // Resample and use it's distinguished points.  Times will default to [0..1]
            array<Point3> points;
            array<Orient> orients;
            array<Time> times;
  
            for(Int i=0; i<path.numDistinguishedValues(); i++) {
              Real s = path.distinguishedValue(i);
              points.push_back(path.position(s));
              orients.push_back(path.orientation(s));
              times.push_back( Time( Real(i)/Real(path.numDistinguishedValues()-1) ) );
            }
  
            init(points,orients,times,false);
          }
          
	}
	else
	  throw externalization_error(Exception("either <trajectory> or <path> expected"));

      }

    }

  } // format xml
  
}


