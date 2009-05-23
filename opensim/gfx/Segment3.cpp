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
  
  $Id: Segment3.cpp 1030 2004-02-11 20:46:17Z jungd $
  $Revision: 1.1 $
  $Date: 2004-02-11 15:46:17 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <math.h>

#include <gfx/Segment3>
#include <gfx/Line3>

using base::Point3;
using gfx::Segment3;
using gfx::Line3;


bool Segment3::contains(const Point3& p) const
{
  // if p isn't on the line through s-e then it isn't on the segment either
  if (!Line3(*this).contains(p)) return false;

  // it is on the line, so just check the components are within range
  return    (p.x >= Math::minimum(s.x,e.x)-consts::epsilon) && (p.x <= Math::maximum(s.x,e.x)+consts::epsilon)
         && (p.y >= Math::minimum(s.y,e.y)-consts::epsilon) && (p.y <= Math::maximum(s.y,e.y)+consts::epsilon)
         && (p.z >= Math::minimum(s.z,e.z)-consts::epsilon) && (p.z <= Math::maximum(s.z,e.z)+consts::epsilon);
}


Real Segment3::distanceTo(const Point3& p) const
{
  return (p-pointClosestTo(p)).length();
}
  

Point3 Segment3::pointClosestTo(const Point3& p) const
{
  if ( s.equals(e) || s.equals(p) ) return s;

  Vector3 d(e-s);
  Real u = ( dot(d,p-s) / dot(d,d) );
  if (u <= 0) return s;
  if (u >= 1) return e;
  return s+u*d;
}


// this implementation and its explanation was basically ripped from the XEngine code by Martin Ecker
//  (email: martin.ecker@liwest.at - see http://xengine.sourceforge.net )
// it looks a lot like the Magic Software algorithm ( http://www.magic-software.com )
/*
	Given two segments X1(s) = P1 + s * D1 and X2(t) = P2 + t * D2 with s, t in
	[0, 1] we can determine the minimal distance using the squared-distance 
	function for any two points on the segments:
		Q(s, t) = |X1(s) - X2(t)|^2
	The function Q(s, t) is quadratic in s and t and can be represented as
		Q(s, t) = as^2 + 2bst + ct^2 +2ds + 2et + f
	with
		a = D1 * D1
		b = -D1 * D2
		c = D2 * D2
		d = D1 * (P1 - P2)
		e = -D2 * (P1 - P2)
		f = (P1 - P2) * (P1 - P2)
	See the line-to-line distance algorithm for a more detailed description of this. 
	
	The basic problem is to find the minimum of the gradient dQ of Q where dQ = (0, 0).
	However, since we're dealing with segments, the resulting s and t must be in
	[0, 1]. The minimum can occur anywhere on Q and s and t could be outside the
	interval [0, 1]. So we have to distinguish between interior points and end
	points. In particular, we have to distinguish between 8 regions where Q(s, t)
	can live. These are: 
		1: s > 1, t in [0, 1]
		2: s > 1, t > 1
		3: s in [0, 1], t > 1
		4: s < 0, t > 1
		5: s < 0, t in [0, 1]
		6: s < 0, t < 0
		7: s in [0, 1], t < 0
		8: s > 1, t < 0
	
	The following graph shows the possible regions. Region 0 is where s and t
	are in [0, 1] and that is the unit square where we have to find the minimum
	distance of the two segments.

	       t

	   4   | 3 | 2 
	---------------
	   5   | 0 | 1
	--------------- s
	       |   |
	   6   | 7 | 8
	       |   |

	Suppose we're in region 1, therefore the minimum of Q occurs somewhere where
	s > 1 but t is in [0, 1]. We now need to find a value Q(s, t) with s = 1
	so that we still get the minimum for all t in [0, 1]. This can be done by
	minimizing the function F(t) = Q(1, t) which is a one-dimensional problem.
	So we just determine the t where F'(t) = 0 where t in [0, 1]. Of course the
	minimum of F(t) can occur outside the interval [0, 1]. In that case we
	either choose t = 0 (if the minimum t < 0) or t = 1 (if the minimum t > 1).
	Regions 3, 5 and 7 are handled similarly. For region 3 we find the minimum
	of Q(s, 1). For region 5 we find the minimum of Q(0, t) and for region 7
	we find the minimum of Q(s, 0).

	Regions 2, 4, 6 and 8 require slightly different handling. Suppose we're in
	region 2 and thus the minimum occurs at s > 1 and t > 1. In this case
	the minimum for s and t in [0, 1] could occur at either s = 1 or t = 1.
	Since the global minimum occurs in region 2 where s > 1 and t > 1,the
	gradient dQ(1, 1) at the corner (1, 1) cannot point point inside region 0
	where s and t are in [0, 1]. If dQ = (Qs, Qt) where Qs and Qt are the partial
	derivatives of Q, then the partial derivatives cannot both be negative.
	The signs of Qs(1, 1) and Qt(1, 1) determine whether we have to use s = 1
	or the edge t = 1. If Qs(1, 1) > 0 then the minimum occurs on the edge t = 1
	since Q(s, 1) < Q(1, 1) for s < 1. If Qt(1, 1) > 0 then the minimum occurs
	on the edge s = 1. Now we can use the approach we've already used for
	regions 1, 3, 5 and 7 to determine the minimum on that edge (and whether
	it occurs on an end point or interior point of [0, 1]).

	So far we have only looked at the case where the determinant of Q(s, t)
	ac - b^2 > 0 (for segments that are not parallel). For line segments that
	are parallel, where ac - b^2 = 0, we can simplify the calculations by
	projecting the second segment onto the first line.
	We do this by projecting the start-point P2 and the end-point P2 + D2
	of the second line segment on the first line segment like so:
		Bstart = P1 + tstart * D1 = P1 + (D1 * (P2 - P1)) / (D1 * D1) * D1 = P1 - d / a * D1
		Bend = P1 + tend * D1 = P1 + (D1 * (P2 + D2 - P1)) / (D1 * D1) * D1 = P1 - (b + d) / a * D1
	where a, b and d are coefficients of Q(s, t) as shown before. Now the problem
	is just to find the relative position of the segment (P1, P1 + D1) to (Bstart, Bend).
	If they overlap there is an infinite number of pairs with minimum distance
	and so we can just choose one point on an overlapping part of a segment and 
	and calculate the distance to the orthogonal interior point of the other segment.
	To further simplify, it is enough to compare the interval [tstart, tend] with
	[0, 1]. To do that we need to first find out the order of [tstart, tend].
	Or in other words, we need to find out if it's really [tstart, tend] or rather
	[tend, tstart] because the direction vector of the second line segment
	points in the other direction of the first line segment. Once that is done
	it is a simple matter of comparing the intervals to find out the minimum
	values for s and t. Note that tstart = -d / a corresponds to t = 0 and 
	tend = -(b + d) / a corresponds to t = 1.
*/

Segment3 Segment3::shortestSegmentBetween(const Segment3& s2, Real& ds) const
{
  const Segment3& s1(*this);

  Real s, t;
  Vector3 d1(s1.e-s1.s);
  Vector3 d2(s2.e-s2.s);
  Vector3 diff = s1.s - s2.s;
  Real a = dot(d1,d1);
  Real b = -dot(d1,d2);
  Real c = dot(d2,d2);
  Real d = dot(d1,diff);
  Real f = dot(diff,diff);
  Real det = Math::abs(a * c - Math::sqr(b));
  
  if (det >= consts::epsilon)
  {
    // segments are not parallel
    Real e = -dot(d2,diff);
    t = b * d - a * e;
    s = b * e - c * d;
    
    if (t >= 0) {
      if (t <= det) {			// t <= 1 ?
        if (s >= 0) {
          if (s <= det) {		// s <= 1?
            // region 0, s and t in [0, 1]
            det = 1 / det;						
            s *= det;
            t *= det;
            ds = s * (a * s + 2 * b * t + 2 * d) + t * (c * t + 2 * e) + f;
          }
          else {			// s > 1
            // region 1, s > 1 and t in [0, 1]
            // we'll set s = 1 and thus have to minimize 
            // Q(1, t) = F(t) = a + 2bt + ct^2 + 2d + 2et + f
            // F'(t) = 2b + 2ct + 2e = 0 ==> t = -(b + e) / c
            s = 1;
            Real temp = b + e;
            if (temp >= 0) {		// t < 0?
              t = 0;
              ds = a + 2 * d + f;
            }
            else if (-temp > c)	{ // t > 1?
              t = 1;
              ds = a + c + f + 2 * (temp + d);
            }
            else {
              t = -temp / c;
              ds = a + temp * t + 2 * d + f;
            }
          }
        }
        else {				// s < 0
          // region 5, s < 0, t in [0, 1]
          // we'll set s = 0 and have to minimize
          // Q(0, t) = F(t) = ct^2 + 2et + f
          // F'(t) = 2ct + 2e = 0 ==> t = -e / c
          s = 0;
          if (e >= 0) {		// t < 0?
            t = 0;
            ds = f;
          }
          else if (-e > c) { // t > 1?
            t = 1;
            ds = c + 2 * e + f;
          }
          else {
            t = -e / c;
            ds = e * t + f;
          }
        }
      }
      else {	// t > 1
        if (s >= 0) {
          if (s <= det) {	// s <= 1?
            // region 3, s in [0, 1], t > 1
            // in this case t = 1 and we have to minimize
            // Q(s, 1) = F(s) = as^2 + 2bs + c + 2ds + 2e + f
            // F'(s) = 2as + 2b + 2d = 0 ==> s = -(b + d) / a
            t = 1;
            Real temp = b + d;
            if (temp >= 0) {			// s < 0?
              s = 0;
              ds = c + 2 * e + f;
            }
            else if (-temp > a) {		// s > 1?
              s = 1;
              ds = a + c + f + 2 * (temp + e);
            }
            else {
              s = -temp / a;
              ds = temp * s + c + 2 * e + f;
            }
          }
          else {				// s > 1
            // region 2, s > 1, t > 1
            if (a + b + d > 0) {	// Qs(1, 1) > 0?
              // t = 1, so we determine the minimum of Q(s, 1) just like in region 3
              // since Q(s, 1) < Q(1, 1) for s < 1 we can omit the second test for s > 1
              // because the minimum definitely won't be there
              t = 1;
              Real temp = b + d;
              if (temp >= 0) {			// s < 0?
                s = 0;
                ds = c + 2 * e + f;
              }
              else {
                s = -temp / a;
                ds = temp * s + c + 2 * e + f;
              }
            }
            else {
              // s = 1, so we determine the minimum of Q(1, t) just like in region 1
              s = 1;
              Real temp = b + e;
              if (temp >= 0) {		// t < 0?
                t = 0;
                ds = a + 2 * d + f;
              }
              else if (-temp > c) {	// t > 1?
                t = 1;
                ds = a + c + f + 2 * (temp + d);
              }
              else {
                t = -temp / c;
                ds = a + temp * t + 2 * d + f;
              }
            }
          }
        }
        else {					// s < 0
          // region 4, s < 0, t > 1
          if (b + d < 0) {					// Qs(0, 1) < 0?
            // t = 1, so we determine the minimum of Q(s, 1) just like in region 3
            // since Q(s, 1) < Q(0, 1) for s > 0 we can omit the test for s < 0
            // because the minimum definitely won't be there
            t = 1;
            Real temp = b + d;
            if (-temp > a) {		// s > 1?
              s = 1;
              ds = a + c + f + 2 * (temp + e);
            }
            else {
              s = -temp / a;
              ds = temp * s + c + 2 * e + f;
            }
          }
          else {
            // s = 0, so we determine the minimum of Q(0, t) just like in region 5
            s = 0;
            if (e >= 0) {			// t < 0?
              t = 0;
              ds = f;
            }
            else if (-e > c) {	// t > 1?
              t = 1;
              ds = c + 2 * e + f;
            }
            else {
              t = -e / c;
              ds = e * t + f;
            }
          }
        }
      }
    }
    else {		// t < 0
      if (s >= 0) {
        if (s <= det) {			// s <= 1?
          // region 7, s in [0, 1], t < 0
          // we'll use t = 0 and have to minimize
          // Q(s, 0) = F(s) = as^2 + 2ds + f
          // F'(s) = 2as + 2d = 0 ==> s = -d / a
          t = 0;
          if (d >= 0) {			// s < 0?
            s = 0;
            ds = f;
          }
          else if (-d > a) {	// s > 1?
            s = 1;
            ds = a + 2 * d + f;
          }
          else {
            s = -d / a;
            ds = d * s + f;
          }
        }
        else {					// s > 1
          // region 8, s > 1, t < 0
          if (a + d > 0) {	// Qs(1, 0) > 0
            // t = 0, so we determine the minimum of Q(s, 0) just like in region 7
            // since Q(s, 0) < Q(1, 0) for s < 1 we can omit the test for s > 1
            // because the minimum definitely won't be there
            t = 0;
            if (d >= 0) {			// s < 0?
              s = 0;
              ds = f;
            }
            else {
              s = -d / a;
              ds = d * s + f;
            }
          }
          else {
            // s = 1, so we determine the minimum of Q(1, t) just like in region 1
            s = 1;
            Real temp = b + e;
            if (temp >= 0) {		// t < 0?
              t = 0;
              ds = a + 2 * d + f;
            }
            else if (-temp > c)	{ // t > 1?
              t = 1;
              ds = a + c + f + 2 * (temp + d);
            }
            else {
              t = -temp / c;
              ds = a + temp * t + 2 * d + f;
            }
          }
        }
      }
      else {	// s < 0
        // region 6, s < 0, t < 0
        if (d < 0) {		// Qs(0, 0) < 0
          // t = 0, so we determine the minimum of Q(s, 0) just like in region 7
          // since Q(s, 0) < Q(0, 0) for s > 0 we can omit the test for s < 0
          // because the minimum definitely won't be there
          t = 0;
          if (-d > a) {		// s > 1?
            s = 1;
            ds = a + 2 * d + f;
          }
          else {
            s = -d / a;
            ds = d * s + f;
          }
        }
        else {
          // s = 0, so we determine the minimum of Q(0, t) just like in region 5
          s = 0;
          if (e >= 0) {			// t < 0?
            t = 0;
            ds = f;
          }
          else if (-e > c) {	// t > 1?
            t = 1;
            ds = c + 2 * e + f;
          }
          else {
            t = -e / c;
            ds = e * t + f;
          }
        }
      }
    }
  }
  else {
    // segments are parallel
    
    // check for ordering of [tstart, tend] interval or in other words, 
    // check whether -d / a > -(b + d) / a which can be simplified to 0 < b
    if (b > 0) {
      // compare [-(b + d) / a, -d / a] with [0, 1]
      if (d > 0) {					// if -d / a <= 0 then the minimum is at s = 0, t = 0
        s = t = 0;
        ds = f;
      }
      else if (-d <= a) {			// if 0 < -d / a <= 1 then minimum is at s = -d / a, t = 0
        s = -d / a;
        t = 0;
        ds = d * s + f;
      }
      else {
        Real e = -dot(d2,diff);
        Real temp = b + d;
        s = 1;
        if (-temp >= a) {		// if -(b + d) / a >= 1 then the minimum is at s = 1, t = 1
          t = 1;
          ds = a + c + f + 2 * (temp + e);
        }
        else {						// else the minimum is at s = 1, t = -(a + d) / b
          t = -(a + d) / b;
          ds = -a + f + t * (c * -t + 2 * e);
        }
      }
    }
    else {
      // compare [-d / a, -(b + d) / a] with [0, 1]
      if (-d >= a) {				// if -d / a >= 1 then minimum is at s = 1, t = 0
        s = 1;
        t = 0;
        ds = a + 2 * d + f;
      }
      else if (d <= 0) {		// if 0 <= -d / a < 1 then minimum is at s = -d / a, t = 0
        s = -d / a;
        t = 0;
        ds = d * s + f;
      }
      else {
        Real e = -dot(d2,diff);
        s = 0;
        if (b > -d) {				// if -(b + d) / a <= 0 then minimum is at s = 0, t = 1
          t = 1;
          ds = c + 2 * e + f;
        }
        else {						// else minimum is at s = 0, t = -d / b
          t = -d / b;
          ds = t * (c * t + 2 * e) + f;
        }
      }
    }
  }
  
  ds = Math::abs(ds);

  return Segment3( s1.s + s*d1, s2.s + t*d2 );
}

