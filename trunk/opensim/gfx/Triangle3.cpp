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
  
  $Id: Triangle3.cpp 1030 2004-02-11 20:46:17Z jungd $
  $Revision: 1.3 $
  $Date: 2004-02-11 15:46:17 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <math.h>

#include <gfx/Triangle3>
#include <gfx/Plane>

using gfx::Triangle3;
using gfx::Point3;
using gfx::Segment3;
using gfx::Plane;


Real Triangle3::distanceTo(const Point3& p) const
{
  return Vector3(p-pointClosestTo(p)).length();
}

  
// this implementation and its explanation was basically ripped from the XEngine code by Martin Ecker
//  (email: martin.ecker@liwest.at - see http://xengine.sourceforge.net )
// it looks a lot like the Magic Software algorithm ( http://www.magic-software.com )
/*
	Given a point P and a triangle T(s, t) = B + s * E0 + t * E1 with s and t 
	in [0, 1] and s + t <= 1 we want to find the closest distance of P to T.
	We want to find the values (s, t) that gives us the closest point on
	the triangle to P. To do so we have to minimze the squared-distance function
		Q(s, t) = |T - P|^2
	This function is quadratic in s and t and is of the form
		Q(s, t) = as^2 + 2bst + ct^2 + 2ds + 2et + f
	with
		a = E0 * E0
		b = E0 * E1
		c = E1 * E1
		d = E0 * (B - P)
		e = -E1 * (B - P)
		f = (B - P) * (B - P)
	Quadratics are classified by the sign of the determinant 
		ac - b^2 = (E0 * E0) * (E1 * E1) - (E0 * E1)^2 = |E0 x E1|^2 > 0
	The determinant is always > 0 since E0 and E1 are linearly independent
	and thus the cross product of the two vectors always yields a nonzero
	vector.
	The minimum of Q(s, t) either occurs at an interior point or at a
	boundary of the domain of T(s, t). So we minimize Q(s, t) by determining
	the gradient dQ(s, t) and calculating s and t where dQ = (0, 0)
		dQ(s, t) = (2as + 2bt + 2d, 2bs + 2ct + 2e) = (0, 0)
	which is at
		s = (be - cd) / (ac - b^2)
		t = (bd - ae) / (ac - b^2)
	If (s, t) is an interior point we have found the minimum. Otherwise
	we must determine the correct boundary of the triangle where the
	minimum occurs. Consider the following figure:

	   t
	   
	\ 2|
	 \ |
	  \|
	   |\
	   | \  1
	   |  \
	 3 | 0 \
	   |    \
	--------------- s
	 4 | 5    \ 6
	   |       \
	   |        \

	If (s, t) is in region 0 we have found the minimum. If (s, t) is
	in region 1 we intersect the graph with the plane s + t = 1 and
	get the parabola F(s) = Q(s, 1 - s) for s in [0, 1] as curve of 
	intersection that we can minimize. Either the minimum occurs at
	F'(s) = 0 or at an end point where s = 0 or s = 1. Regions 3 and
	5 are handled similarly.
	If (s, t) is in region 2 the minimum could be on the edge s + t = 1
	or s = 0. Because the global minimum occurs in region 2 the gradient
	at the corner (0, 1) cannot point inside the triangle.
	If dQ = (Qs, Qt) with Qs and Qt being the respective partial derivatives
	of Q, it must be that (0, -1) * dQ(0, 1) and (1, -1) * dQ(0, 1)
	cannot both be negative and the signs of these values can be used
	to determine the correct edge. (0, -1) and (1, -1) are direction vectors
	for the edges s = 0 and s + t = 1. Similar arguments apply to
	regions 4 and 6.
*/
Point3 Triangle3::pointClosestTo(const Point3& p) const
{
  Real s, t;
  Real ds=0; // squared distance
  // express triangle as T(s, t) = B + s * E0 + t * E1 with s and t in [0, 1] and s + t <= 1.
  Point3  tp = p2(); // triangle 'point' - B
  Vector3 e0 = p1() - tp; // triangle E0
  Vector3 e1 = p3() - tp; // triangle E1

  Vector3 diff = tp - p;
  Real a = dot(e0,e0);
  Real b = dot(e0,e1);
  Real c = dot(e1,e1);
  Real d = dot(e0,diff);
  Real e = dot(e1,diff);
  Real f = dot(diff,diff);
  Real det = Math::abs(a * c - Math::sqr(b));
  s = b * e - c * d;
  t = b * d - a * e;
  
  if (s + t <= det) {		// s + t <= 1?
    if (s < 0) {
      if (t < 0) {
        // region 4
        // minimum could be on edge s = 0 or t = 0 depending on the signs of
        // Qs and Qt for dQ(0, 0) = (2d, 2e)
        // s = 0: (0, 1) * dQ(0, 0) = (0, 1) * (2d, 2e) = 2e	if < 0 then minimum on s = 0
        // t = 0: (1, 0) * dQ(0, 0) = (1, 0) * (2d, 2e) = 2d	if < 0 then minimum on t = 0
        if (d < 0) {		// Qs(0, 0) < 0 ?
          // find minimum on edge t = 0, so minimize
          // F(s) = Q(s, 0) = as^2 + 2ds + f
          // F'(s) = 2as + 2d = 0 ==> s = -d / a
          // since Q(s, 0) < Q(0, 0) for s > 0 we can omit the test for s > 0
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
          // find minimum on edge s = 0, so minimize
          // F(t) = Q(0, t) = ct^2 + 2et + f
          // F'(t) = 2ct + 2e = 0 ==> t = -e / c
          s = 0;
          if (e >= 0) {			// t < 0 ?
            t = 0;
            ds = f;
          }
          else if (-e > c) {	// t > 1 ?
            t = 1;
            ds = c + 2 * e + f;
          }
          else {
            t = -e / c;
            ds = c * t + f;
          }
        }
      }
      else {
        // region 3
        // we must find the minimum on the edge s = 0, so we minimize
        // F(t) = Q(0, t) = ct^2 + 2et + f
        // F'(t) = 2ct + 2e = 0 ==> t = -e / c
        s = 0;
        if (e >= 0) {			// t < 0 ?
          t = 0;
          ds = f;
        }
        else if (-e > c) {	// t > 1 ?
          t = 1;
          ds = c + 2 * e + f;
        }
        else {
          t = -e / c;
          ds = c * t + f;
        }
      }
    }
    else if (t < 0) {
      // region 5
      // find minimum on the edge t = 0, so we minimize
      // F(s) = Q(s, 0) = as^2 + 2ds + f
      // F'(s) = 2as + 2d = 0 ==> s = -d / a
      t = 0;
      if (d >= 0) {		// s < 0?
        s = 0;
        ds = f;
      }
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
      // region 0
      // we have an interior point, so just use s and t in 
      // Q(s, t) = as^2 + 2bst + ct^2 + 2ds + 2et + f
      det = 1 / det;
      s *= det;
      t *= det;
      ds = f + s * (a * s + 2 * (b * t + 2 * d)) + t * (c * t + 2 * e);
    }
  }
  else {	// s + t > 1
    if (s < 0) {
      // region 2
      // minimum could be on edge s = 0 or s + t = 1, depending on the sign of
      // s = 0: (0, -1) * dQ(0, 1) = (0, -1) * (2b + 2d, 2c + 2e) = -2 * (c + e)		if < 0 then minimum on s = 0
      // s + t = 1: (1, -1) * dQ(0, 1) = (1, -1) * (2b + 2d, 2c + 2e) = 2 * ((b + d) - (c + e))		if < 0 then minimum is on s + t = 1
      
      Real bd = b + d;
      Real ce = c + e;
      if (bd < ce) {
        // minimum on s + t = 1
        // F(s) = Q(s, 1 - s) = (a - 2b + c)s^2 + 2(b - c + d - e)s + c + 2e + f
        // F'(s) = 2(a - 2b + c)s + 2(b - c + d - e) = 0 ==> s = (c + e - b - d) / (a - 2b + c)
        Real temp = ce - bd;
        Real temp2 = a - 2 * b + c;
        if (temp > temp2) {		// s > 1?
          s = 1;
          t = 0;
          ds = a + 2 * d + f;
        }
        else {
          s = temp / temp2;
          t = 1 - s;
          ds = f + s * (a + 2 * (b * t + d)) + t * (c * t + 2 * e);
        }
      }
      else {
        // minimum on s = 0, so we minimize
        // F(t) = Q(0, t) = ct^2 + 2et + f
        // F'(t) = 2ct + 2e = 0 ==> t = -e / c
        s = 0;
        if (e >= 0) {			// t < 0 ?
          t = 0;
          ds = f;
        }
        else if (-e > c) {	// t > 1 ?
          t = 1;
          ds = c + 2 * e + f;
        }
        else {
          t = -e / c;
          ds = c * t + f;
        }
      }
    }
    else if (t < 0) {
      // region 6
      // minimum could be on edge s + t = 1 or t = 0
      // t = 0: (-1, 0) * dQ(1, 0) = (-1, 0) * (2a + 2d, 2b + 2e) = -2 * (a + d)		if < 0 then minimum on t = 0
      // s + t = 1: (-1, 1) * dQ(1, 0) = (-1, 1) * (2a + 2d, 2b + 2e) = -2 * (a + d) + 2 * (b + e)		if < 0 then minimum on s + t = 1
      Real ad = a + d;
      Real be = b + e;
      if (be < ad) {
        // minimum is on s + t = 1
        // F(t) = Q(1 - t, t) = (a - 2b + c)t^2 + 2(b - a - d + e)t + a + 2d + f
        // F'(t) = 2(a - 2b + c)t + 2(b - a - d + e) = 0 ==> t = (a + d - b - e) / (a - 2b + c)
        Real temp = ad - be;
        Real temp2 = a - 2 * b + c;
        if (temp > temp2) {		// t > 1?
          t = 1;
          s = 0;
          ds = c + 2 * e + f;
        }
        else {
          t = temp / temp2;
          s = 1 - t;
          ds = f + s * (a + 2 * (b * t + d)) + t * (c * t + 2 * e);
        }
      }
      else {
        // minimum is on t = 0, so we minimize
        // F(s) = Q(s, 0) = as^2 + 2ds + f
        // F'(s) = 2as + 2d = 0 ==> s = -d / a
        t = 0;
        if (d >= 0) {		// s < 0?
          s = 0;
          ds = f;
        }
        if (-d > a) {		// s > 1?
          s = 1;
          ds = a + 2 * d + f;
        }
        else {
          s = -d / a;
          ds = d * s + f;
        }
      }
    }
    else {
      // region 1
      // minimum on edge s + t = 1 so we minimize
      // F(s) = Q(s, 1 - s) = (a - 2b + c)s^2 + 2(b - c + d - e)s + c + 2e + f
      // F'(s) = 2(a - 2b + c)s + 2(b - c + d - e) = 0 ==> s = (c + e - b - d) / (a - 2b + c)
      Real temp = c + e - b - d;
      if (temp < 0) {		// s < 0?
        s = 0;
        t = 1;
        ds = c + 2 * e + f;
      }
      else {
        Real temp2 = a - 2 * b + c;
        if (temp > temp2)		// s > 1?
        {
          s = 1;
          t = 0;
          ds = a + 2 * d + f;
        }
        else {
          s = temp / temp2;
          t = 1 - s;
          ds = f + s * (a + 2 * (b * t + d)) + t * (c * t + 2 * e);
        }
      }
    }
  }
  
  
  ds = Math::abs(ds);
  return tp + s * e0 + t * e1;
}





// this implementation and its explanation was basically ripped from the XEngine code by Martin Ecker
//  (email: martin.ecker@liwest.at - see http://xengine.sourceforge.net )
// it looks a lot like the Magic Software algorithm ( http://www.magic-software.com )
/* 
	The problem of computing the minimal squared distance between a segment and
	a triangle is very similar to the one of calculating the distance between
	a line and a triangle.
	However the domain for the parameter r of the segment this time is r in [0, 1].
	Therefore we get more regions when partitioning the space spanned by (s, t, r)
	into regions. 
	Consider the following two figures:

	   t                           t
	   
	\ 2|                           |       |
	 \ |                      3n-2n|  3-2  |3p-2p
	  \|                           |       |
	   |\                  --------|-----------------
	   | \                         |       |
	   |  \                        |       |
	 3 | 0 \  1            3n-0n-1n| 3-0-1 |3p-0p-1p
	   |    \                      |       |
	---r----------- s      --------s----------------- r
	   |      \                    |       |
	 4 | 5     \ 6            4n-5n|  4-5  |4p-5p
	   |        \                  |       |

	The first figure shows a cut with the s-t plane and shows the regions
	we got with line-to-triangle distance calculation. The second figure
	shows a cut with the t-r plane and shows the new regions we get since
	the prism that represents region 0 is no longer an infinite prism but
	a finite prism along the positive r-axis where r in [0, 1]. 
	The 7 regions where r < 0 have the same names as the regions where r
	is in [0, 1] except that they have an additional letter n for negative.
	The 7 regions where r > 1 have the same names as the regions where r
	is in [0, 1] except that they have an additional letter p for positive.
*/
Segment3 Triangle3::shortestSegmentBetween(const Segment3& seg, Real& ds) const
{
  Real s, t, r;
  ds=0; // distance squared

  // express triangle as T(s, t) = B + s * E0 + t * E1 with s and t in [0, 1] and s + t <= 1.
  Point3  tp = _p2; // triangle 'point' - B
  Vector3 e0 = _p1 - _p2; // triangle edge 0
  Vector3 e1 = _p3 - _p2; // triangle edge 1
  Vector3 diff = _p2 - seg.s;
  Vector3 segdir = seg.e-seg.s; // seg direction
  Point3 p1,p2; // between segment end-points
  Real a00 = dot(e0,e0);
  Real a11 = dot(e1,e1);
  Real a22 = dot(segdir,segdir);
  Real a01 = dot(e0,e1);
  Real a02 = -dot(e0,segdir);
  Real a12 = -dot(e1,segdir);
  Real b0 = dot(e0,diff);
  Real b1 = dot(e1,diff);
  Real b2 = -dot(segdir,diff);
  
  // cofactors for calculating the determinant and later on to invert A
  Real cf00 = a11 * a22 - a12 * a12;
  Real cf01 = a02 * a12 - a22 * a01;
  Real cf02 = a01 * a12 - a02 * a11;
  Real det = a00 * cf00 + a01 * cf01 + a02 * cf02;
  
  if (Math::abs(det) >= consts::epsilon)		// det != 0?
  {
    // to be able to invert A we need some more cofactors
    Real cf11 = a00 * a22 - a02 * a02;
    Real cf12 = a02 * a01 - a00 * a12;
    Real cf22 = a00 * a11 - a01 * a01;
    
    det = 1 / det;
    s = (-b0 * cf00 - b1 * cf01 - b2 * cf02) * det;
    t = (-b0 * cf01 - b1 * cf11 - b2 * cf12) * det;
    r = (-b0 * cf02 - b1 * cf12 - b2 * cf22) * det;
    
    if (s + t <= 1) {
      if (s < 0) {
        if (t < 0) {
          // region 4 or 4n or 4p
          // the minimum is either on face s = 0 or t = 0 or r = 0/1
          Segment3 segment1(_p2, _p1);
          Segment3 segment2(_p2, _p3);
          Segment3 sb1(segment1.shortestSegmentBetween(seg));
          Segment3 sb2(segment2.shortestSegmentBetween(seg));
          Real ds1 = sb1.norm();
          Real ds2 = sb2.norm();
          if (ds1 < ds2) {
            ds = ds1;
            p1 = sb1.s; p2 = sb1.e;
          }
          else {
            ds = ds2;
            p1 = sb2.s; p2 = sb2.e;
          }
        }
        else {
          // region 3 or 3n or 3p
          // the minimum is on the face s = 0 or r = 0/1
          Segment3 sb = seg.shortestSegmentBetween(Segment3(_p2, _p3), ds);
          p1 = sb.e; p2 = sb.s;
        }
      }
      else if (t < 0) {
        // region 5 or 5n or 5p
        // the minimum is on the face t = 0 or r = 0/1
        Segment3 sb = seg.shortestSegmentBetween(Segment3(_p2, _p1), ds);
        p1 = sb.e; p2 = sb.s;
      }
      else if (r >= 0 && r <= 1) {
        // region 0
        Real c = dot(diff,diff);
        ds = s * (a00 * s + 2 * (a01 * t + a02 * r + b0)) + 
        t * (a11 * t + 2 * (a12 * r + b1)) +
        r * (a22 * r + 2 * b2) + c;
        
        p1 = seg.s + r * segdir;
        p2 = tp + s * e0 + t * e1;
      }			
    }
    else {
      if (s < 0) {
        // region 1 or 2n or 2p
        // the minimum is either on the face s = 0 or s + t = 1 or r = 0/1
        Segment3 segment1(_p2, _p3);
        Segment3 segment2(_p1, _p3);
        Segment3 sb1(segment1.shortestSegmentBetween(seg));
        Segment3 sb2(segment1.shortestSegmentBetween(seg));
        Real ds1 = sb1.norm();
        Real ds2 = sb2.norm();
        if (ds1 < ds2) {
          ds = ds1;
          p1 = sb1.s; p2 = sb1.e;
        }
        else {
          ds = ds2;
          p1 = sb2.s; p2 = sb2.e;
        }
      }
      else if (t < 0) {
        // region 6n
        // the minimum is either on the face t = 0 or s + t = 1 or r = 0/1
        Segment3 segment1(_p2, _p1);
        Segment3 segment2(_p1, _p3);
        Segment3 sb1(segment1.shortestSegmentBetween(seg));
        Segment3 sb2(segment1.shortestSegmentBetween(seg));
        Real ds1 = sb1.norm();
        Real ds2 = sb2.norm();
        if (ds1 < ds2) {
          ds = ds1;
          p1 = sb1.s; p2 = sb1.e;
        }
        else {
          ds = ds2;
          p1 = sb2.s; p2 = sb2.e;
        }
      }
      else {
        // region 1 or 1n or 1p
        // minimum is on face s + t = 1 or r = 0/1
        Segment3 segment1(_p1, _p3);
        Segment3 sb = segment1.shortestSegmentBetween(seg, ds);
        p1 = sb.s; p2 = sb.e;
      }
    }
    
    // if r outside [0, 1] the minimum might be on r = 0 or r = 1
    if (r < 0 || r > 1) {
      // determine squared distance from start-point or end-point of the segment to the triangle
      // this is for face r = 0 or r = 1 depending on the value of r
      Point3 p1temp, p2temp;
      Real distTemp;
      if (r < 0) {
        Segment3 sb(pointClosestTo(seg.s), seg.s);
        distTemp = sb.norm();
        p1temp = sb.s; p2temp = sb.e;
      }
      else {
        Segment3 sb(pointClosestTo(seg.e), seg.e);
        distTemp = sb.norm();
        p1temp = sb.s; p2temp = sb.e;
      }
      if (distTemp < ds || (s >= 0 && t >= 0)) {	// the part after || is for region 0n or 0p where the minimum is on r = 0 or r = 1
        ds = distTemp;
        p1 = p1temp;
        p2 = p2temp;
      }
    }
  }
  else {
    // segment and triangle are parallel, so we'll just compute the minimum 
    // distance of the segment to each of the three triangle edges and of the segment end-points to the triangle
    Segment3 segment1(_p2, _p1);
    Segment3 segment2(_p2, _p3);
    Segment3 segment3(_p1, _p3);
    Segment3 sb1(segment1.shortestSegmentBetween(seg));
    Segment3 sb2(segment2.shortestSegmentBetween(seg));
    Segment3 sb3(segment3.shortestSegmentBetween(seg));
    Real ds1 = sb1.norm();
    Real ds2 = sb2.norm();
    Real ds3 = sb3.norm();
    if (ds1 < ds2) {
      if (ds1 < ds3) {
        ds = ds1;
        p1 = sb1.s; p2 = sb1.e;
      }
      else {
        ds = ds3;
        p1 = sb3.s; p2 = sb3.e;
      }
    }
    else { // ds2 < ds1
      if (ds2 < ds3) {
        ds = ds2;
        p1 = sb2.s; p2 = sb2.e;
      }
      else {
        ds = ds3;
        p1 = sb3.s; p2 = sb3.e;
      }
    }
    
    Point3 p1temp, p2temp;
    Segment3 sbp(pointClosestTo(seg.s), seg.s);
    Real distTemp = sbp.norm();
    p1temp = sbp.s; p2temp = sbp.e;
    if (distTemp < ds) {
      ds = distTemp;
      p1 = p1temp;
      p2 = p2temp;
    }
    Segment3 sbp2(pointClosestTo(seg.e), seg.e);
    distTemp = sbp2.norm();
    p1temp = sbp2.s; p2temp = sbp2.e;
    if (distTemp < ds) {
      ds = distTemp;
      p1 = p1temp;
      p2 = p2temp;
    }
  }
  
  
  ds = Math::abs(ds);

  //Assert(this->contains(p1)); // can fail is interpenetration
  //Assert(seg.contains(p2));
  
  return Segment3(p1,p2);
}



Segment3 Triangle3::shortestSegmentBetween(const Triangle3& t) const
{
  // compare the shortest distance between each edge of this with t
  //  and each edge of t with this
  Segment3 t11(p1(), p2());
  Segment3 t12(p2(), p3());
  Segment3 t13(p3(), p1());
  
  Segment3 t21(t.p1(), t.p2());
  Segment3 t22(t.p2(), t.p3());
  Segment3 t23(t.p3(), t.p1());

  Segment3 seg = shortestSegmentBetween(t21);
  Segment3 shortest = seg;
  
  seg = shortestSegmentBetween(t22);
  if (seg.norm() < shortest.norm()) shortest = seg;
  
  seg = shortestSegmentBetween(t23);
  if (seg.norm() < shortest.norm()) shortest = seg;
  
  seg = t.shortestSegmentBetween(t11);
  if (seg.norm() < shortest.norm()) shortest = seg;

  seg = t.shortestSegmentBetween(t12);
  if (seg.norm() < shortest.norm()) shortest = seg;

  seg = t.shortestSegmentBetween(t13);
  if (seg.norm() < shortest.norm()) shortest = seg;

  return shortest;
}
  
  
Real Triangle3::distanceTo(const Triangle3& t) const
{
  return shortestSegmentBetween(t).length();
}








// helper class for computing triangle intersections
namespace gfx {

class TriangleDesc : public Triangle3
{
public:
  TriangleDesc(const Triangle3& t, const Plane& p)
    : Triangle3(t) 
  {
    const Vector3& n=p.normal;
    Vector3 a(base::abs(n.x),base::abs(n.y),base::abs(n.z));
    if (a.x>a.y) {
      if (a.x>a.z) { i1=2; i2=3; }
      else         { i1=1; i2=2; }
    }
    else {
      if (a.y>a.z) { i1=1; i2=3; }
      else         { i1=1; i2=2; }
    }
  }
  
  bool pointInTri(const Vector3& P) const
  {
    const Point3& v1( (*this)[1] );
    const Point3& v2( (*this)[2] );
    const Point3& v3( (*this)[3] );
    Vector3 u(P[i1]-v1[i1],
	      v2[i1]-v1[i1],
	      v3[i1]-v1[i1]);
    Vector3 v(P[i2]-v1[i2],
	      v2[i2]-v1[i2],
	      v3[i2]-v1[i2]);
    Real a,b;
    if (u.y==0.0) {
      b=u.x/u.z;
      if (b>=0.0 && b<=1.0) a=(v.x-b*v.z)/v.y;
      else return false;
    }
    else {
      b=(v.x*u.y-u.x*v.y)/(v.z*u.y-u.z*v.y);
      if (b>=0.0 && b<=1.0) a=(u.x-b*u.z)/u.y;
      else return false;
    }
    return (a>=0 && (a+b)<=1);
  }
  
  // A more forgiving version
  const Point3& operator[] (Int i) const throw()
  {
    SInt zi = i-1;
    if (zi<0) zi += 3;
    zi = zi%3;
    
    switch (zi) {
    case 0: return at(1);
    case 1: return at(2);
    case 2: return at(3);
    default: return at(1); // never
    }
  }
	
  Int i1,i2;
};
  
}// gfx

using gfx::TriangleDesc;


// 
// Some 'C' triangle intersection code
//
// The implementation of Triangle follows at the bottom




/* Triangle/triangle intersection test routine,
 * by Tomas Moller, 1997.
 * See article "A Fast Triangle-Triangle Intersection Test",
 * Journal of Graphics Tools, 2(2), 1997
 * updated: 2001-06-20 (added line of intersection)
 *
 * int tri_tri_intersect(Real V0[3],Real V1[3],Real V2[3],
 *                       Real U0[3],Real U1[3],Real U2[3])
 *
 * parameters: vertices of triangle 1: V0,V1,V2
 *             vertices of triangle 2: U0,U1,U2
 * result    : returns 1 if the triangles intersect, otherwise 0
 *
 * Here is a version withouts divisions (a little faster)
 * int NoDivTriTriIsect(Real V0[3],Real V1[3],Real V2[3],
 *                      Real U0[3],Real U1[3],Real U2[3]);
 * 
 * This version computes the line of intersection as well (if they are not coplanar):
 * int tri_tri_intersect_with_isectline(Real V0[3],Real V1[3],Real V2[3], 
 *				        Real U0[3],Real U1[3],Real U2[3],int *coplanar,
 *				        Real isectpt1[3],Real isectpt2[3]);
 * coplanar returns whether the tris are coplanar
 * isectpt1, isectpt2 are the endpoints of the line of intersection
 */


#define FABS(x) (base::abs(x))        /* implement as is fastest on your machine */

/* if USE_EPSILON_TEST is true then we do a check: 
         if |dv|<EPSILON then dv=0.0;
   else no check is done (which is less robust)
*/
#define USE_EPSILON_TEST TRUE  
#define EPSILON 0.000001


/* some macros */
#define CROSS(dest,v1,v2)                      \
              dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
              dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
              dest[2]=v1[0]*v2[1]-v1[1]*v2[0];

#define DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])

#define SUB(dest,v1,v2) dest[0]=v1[0]-v2[0]; dest[1]=v1[1]-v2[1]; dest[2]=v1[2]-v2[2]; 

#define ADD(dest,v1,v2) dest[0]=v1[0]+v2[0]; dest[1]=v1[1]+v2[1]; dest[2]=v1[2]+v2[2]; 

#define MULT(dest,v,factor) dest[0]=factor*v[0]; dest[1]=factor*v[1]; dest[2]=factor*v[2];

#define SET(dest,src) dest[0]=src[0]; dest[1]=src[1]; dest[2]=src[2]; 

/* sort so that a<=b */
#define SORT(a,b)       \
             if(a>b)    \
             {          \
               Real c; \
               c=a;     \
               a=b;     \
               b=c;     \
             }

#define ISECT(VV0,VV1,VV2,D0,D1,D2,isect0,isect1) \
              isect0=VV0+(VV1-VV0)*D0/(D0-D1);    \
              isect1=VV0+(VV2-VV0)*D0/(D0-D2);


#define COMPUTE_INTERVALS(VV0,VV1,VV2,D0,D1,D2,D0D1,D0D2,isect0,isect1) \
  if(D0D1>0.0f)                                         \
  {                                                     \
    /* here we know that D0D2<=0.0 */                   \
    /* that is D0, D1 are on the same side, D2 on the other or on the plane */ \
    ISECT(VV2,VV0,VV1,D2,D0,D1,isect0,isect1);          \
  }                                                     \
  else if(D0D2>0.0f)                                    \
  {                                                     \
    /* here we know that d0d1<=0.0 */                   \
    ISECT(VV1,VV0,VV2,D1,D0,D2,isect0,isect1);          \
  }                                                     \
  else if(D1*D2>0.0f || D0!=0.0f)                       \
  {                                                     \
    /* here we know that d0d1<=0.0 or that D0!=0.0 */   \
    ISECT(VV0,VV1,VV2,D0,D1,D2,isect0,isect1);          \
  }                                                     \
  else if(D1!=0.0f)                                     \
  {                                                     \
    ISECT(VV1,VV0,VV2,D1,D0,D2,isect0,isect1);          \
  }                                                     \
  else if(D2!=0.0f)                                     \
  {                                                     \
    ISECT(VV2,VV0,VV1,D2,D0,D1,isect0,isect1);          \
  }                                                     \
  else                                                  \
  {                                                     \
    /* triangles are coplanar */                        \
    return coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2);      \
  }



/* this edge to edge test is based on Franlin Antonio's gem:
   "Faster Line Segment Intersection", in Graphics Gems III,
   pp. 199-202 */ 
#define EDGE_EDGE_TEST(V0,U0,U1)                      \
  Bx=U0[i0]-U1[i0];                                   \
  By=U0[i1]-U1[i1];                                   \
  Cx=V0[i0]-U0[i0];                                   \
  Cy=V0[i1]-U0[i1];                                   \
  f=Ay*Bx-Ax*By;                                      \
  d=By*Cx-Bx*Cy;                                      \
  if((f>0 && d>=0 && d<=f) || (f<0 && d<=0 && d>=f))  \
  {                                                   \
    e=Ax*Cy-Ay*Cx;                                    \
    if(f>0)                                           \
    {                                                 \
      if(e>=0 && e<=f) return 1;                      \
    }                                                 \
    else                                              \
    {                                                 \
      if(e<=0 && e>=f) return 1;                      \
    }                                                 \
  }                                

#define EDGE_AGAINST_TRI_EDGES(V0,V1,U0,U1,U2) \
{                                              \
  Real Ax,Ay,Bx,By,Cx,Cy,e,d,f;               \
  Ax=V1[i0]-V0[i0];                            \
  Ay=V1[i1]-V0[i1];                            \
  /* test edge U0,U1 against V0,V1 */          \
  EDGE_EDGE_TEST(V0,U0,U1);                    \
  /* test edge U1,U2 against V0,V1 */          \
  EDGE_EDGE_TEST(V0,U1,U2);                    \
  /* test edge U2,U1 against V0,V1 */          \
  EDGE_EDGE_TEST(V0,U2,U0);                    \
}

#define POINT_IN_TRI(V0,U0,U1,U2)           \
{                                           \
  Real a,b,c,d0,d1,d2;                     \
  /* is T1 completly inside T2? */          \
  /* check if V0 is inside tri(U0,U1,U2) */ \
  a=U1[i1]-U0[i1];                          \
  b=-(U1[i0]-U0[i0]);                       \
  c=-a*U0[i0]-b*U0[i1];                     \
  d0=a*V0[i0]+b*V0[i1]+c;                   \
                                            \
  a=U2[i1]-U1[i1];                          \
  b=-(U2[i0]-U1[i0]);                       \
  c=-a*U1[i0]-b*U1[i1];                     \
  d1=a*V0[i0]+b*V0[i1]+c;                   \
                                            \
  a=U0[i1]-U2[i1];                          \
  b=-(U0[i0]-U2[i0]);                       \
  c=-a*U2[i0]-b*U2[i1];                     \
  d2=a*V0[i0]+b*V0[i1]+c;                   \
  if(d0*d1>0.0)                             \
  {                                         \
    if(d0*d2>0.0) return 1;                 \
  }                                         \
}

int coplanar_tri_tri(Real N[3],Real V0[3],Real V1[3],Real V2[3],
                     Real U0[3],Real U1[3],Real U2[3])
{
   Real A[3];
   short i0,i1;
   /* first project onto an axis-aligned plane, that maximizes the area */
   /* of the triangles, compute indices: i0,i1. */
   A[0]=fabs(N[0]);
   A[1]=fabs(N[1]);
   A[2]=fabs(N[2]);
   if(A[0]>A[1])
   {
      if(A[0]>A[2])  
      {
          i0=1;      /* A[0] is greatest */
          i1=2;
      }
      else
      {
          i0=0;      /* A[2] is greatest */
          i1=1;
      }
   }
   else   /* A[0]<=A[1] */
   {
      if(A[2]>A[1])
      {
          i0=0;      /* A[2] is greatest */
          i1=1;                                           
      }
      else
      {
          i0=0;      /* A[1] is greatest */
          i1=2;
      }
    }               
                
    /* test all edges of triangle 1 against the edges of triangle 2 */
    EDGE_AGAINST_TRI_EDGES(V0,V1,U0,U1,U2);
    EDGE_AGAINST_TRI_EDGES(V1,V2,U0,U1,U2);
    EDGE_AGAINST_TRI_EDGES(V2,V0,U0,U1,U2);
                
    /* finally, test if tri1 is totally contained in tri2 or vice versa */
    POINT_IN_TRI(V0,U0,U1,U2);
    POINT_IN_TRI(U0,V0,V1,V2);

    return 0;
}


int tri_tri_intersect(Real V0[3],Real V1[3],Real V2[3],
                      Real U0[3],Real U1[3],Real U2[3])
{
  Real E1[3],E2[3];
  Real N1[3],N2[3],d1,d2;
  Real du0,du1,du2,dv0,dv1,dv2;
  Real D[3];
  Real isect1[2], isect2[2];
  Real du0du1,du0du2,dv0dv1,dv0dv2;
  short index;
  Real vp0,vp1,vp2;
  Real up0,up1,up2;
  Real b,c,max;

  /* compute plane equation of triangle(V0,V1,V2) */
  SUB(E1,V1,V0);
  SUB(E2,V2,V0);
  CROSS(N1,E1,E2);
  d1=-DOT(N1,V0);
  /* plane equation 1: N1.X+d1=0 */

  /* put U0,U1,U2 into plane equation 1 to compute signed distances to the plane*/
  du0=DOT(N1,U0)+d1;
  du1=DOT(N1,U1)+d1;
  du2=DOT(N1,U2)+d1;

  /* coplanarity robustness check */
#if USE_EPSILON_TEST==TRUE
  if(fabs(du0)<EPSILON) du0=0.0;
  if(fabs(du1)<EPSILON) du1=0.0;
  if(fabs(du2)<EPSILON) du2=0.0;
#endif
  du0du1=du0*du1;
  du0du2=du0*du2;

  if(du0du1>0.0f && du0du2>0.0f) /* same sign on all of them + not equal 0 ? */
    return 0;                    /* no intersection occurs */

  /* compute plane of triangle (U0,U1,U2) */
  SUB(E1,U1,U0);
  SUB(E2,U2,U0);
  CROSS(N2,E1,E2);
  d2=-DOT(N2,U0);
  /* plane equation 2: N2.X+d2=0 */

  /* put V0,V1,V2 into plane equation 2 */
  dv0=DOT(N2,V0)+d2;
  dv1=DOT(N2,V1)+d2;
  dv2=DOT(N2,V2)+d2;

#if USE_EPSILON_TEST==TRUE
  if(fabs(dv0)<EPSILON) dv0=0.0;
  if(fabs(dv1)<EPSILON) dv1=0.0;
  if(fabs(dv2)<EPSILON) dv2=0.0;
#endif

  dv0dv1=dv0*dv1;
  dv0dv2=dv0*dv2;
        
  if(dv0dv1>0.0f && dv0dv2>0.0f) /* same sign on all of them + not equal 0 ? */
    return 0;                    /* no intersection occurs */

  /* compute direction of intersection line */
  CROSS(D,N1,N2);

  /* compute and index to the largest component of D */
  max=fabs(D[0]);
  index=0;
  b=fabs(D[1]);
  c=fabs(D[2]);
  if(b>max) max=b,index=1;
  if(c>max) max=c,index=2;

  /* this is the simplified projection onto L*/
  vp0=V0[index];
  vp1=V1[index];
  vp2=V2[index];
  
  up0=U0[index];
  up1=U1[index];
  up2=U2[index];

  /* compute interval for triangle 1 */
  COMPUTE_INTERVALS(vp0,vp1,vp2,dv0,dv1,dv2,dv0dv1,dv0dv2,isect1[0],isect1[1]);

  /* compute interval for triangle 2 */
  COMPUTE_INTERVALS(up0,up1,up2,du0,du1,du2,du0du1,du0du2,isect2[0],isect2[1]);

  SORT(isect1[0],isect1[1]);
  SORT(isect2[0],isect2[1]);

  if(isect1[1]<isect2[0] || isect2[1]<isect1[0]) return 0;
  return 1;
}


#define NEWCOMPUTE_INTERVALS(VV0,VV1,VV2,D0,D1,D2,D0D1,D0D2,A,B,C,X0,X1) \
{ \
        if(D0D1>0.0f) \
        { \
                /* here we know that D0D2<=0.0 */ \
            /* that is D0, D1 are on the same side, D2 on the other or on the plane */ \
                A=VV2; B=(VV0-VV2)*D2; C=(VV1-VV2)*D2; X0=D2-D0; X1=D2-D1; \
        } \
        else if(D0D2>0.0f)\
        { \
                /* here we know that d0d1<=0.0 */ \
            A=VV1; B=(VV0-VV1)*D1; C=(VV2-VV1)*D1; X0=D1-D0; X1=D1-D2; \
        } \
        else if(D1*D2>0.0f || D0!=0.0f) \
        { \
                /* here we know that d0d1<=0.0 or that D0!=0.0 */ \
                A=VV0; B=(VV1-VV0)*D0; C=(VV2-VV0)*D0; X0=D0-D1; X1=D0-D2; \
        } \
        else if(D1!=0.0f) \
        { \
                A=VV1; B=(VV0-VV1)*D1; C=(VV2-VV1)*D1; X0=D1-D0; X1=D1-D2; \
        } \
        else if(D2!=0.0f) \
        { \
                A=VV2; B=(VV0-VV2)*D2; C=(VV1-VV2)*D2; X0=D2-D0; X1=D2-D1; \
        } \
        else \
        { \
                /* triangles are coplanar */ \
                return coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2); \
        } \
}



int NoDivTriTriIsect(Real V0[3],Real V1[3],Real V2[3],
                     Real U0[3],Real U1[3],Real U2[3])
{
  Real E1[3],E2[3];
  Real N1[3],N2[3],d1,d2;
  Real du0,du1,du2,dv0,dv1,dv2;
  Real D[3];
  Real isect1[2], isect2[2];
  Real du0du1,du0du2,dv0dv1,dv0dv2;
  short index;
  Real vp0,vp1,vp2;
  Real up0,up1,up2;
  Real bb,cc,max;
  Real a,b,c,x0,x1;
  Real d,e,f,y0,y1;
  Real xx,yy,xxyy,tmp;

  /* compute plane equation of triangle(V0,V1,V2) */
  SUB(E1,V1,V0);
  SUB(E2,V2,V0);
  CROSS(N1,E1,E2);
  d1=-DOT(N1,V0);
  /* plane equation 1: N1.X+d1=0 */

  /* put U0,U1,U2 into plane equation 1 to compute signed distances to the plane*/
  du0=DOT(N1,U0)+d1;
  du1=DOT(N1,U1)+d1;
  du2=DOT(N1,U2)+d1;

  /* coplanarity robustness check */
#if USE_EPSILON_TEST==TRUE
  if(FABS(du0)<EPSILON) du0=0.0;
  if(FABS(du1)<EPSILON) du1=0.0;
  if(FABS(du2)<EPSILON) du2=0.0;
#endif
  du0du1=du0*du1;
  du0du2=du0*du2;

  if(du0du1>0.0f && du0du2>0.0f) /* same sign on all of them + not equal 0 ? */
    return 0;                    /* no intersection occurs */

  /* compute plane of triangle (U0,U1,U2) */
  SUB(E1,U1,U0);
  SUB(E2,U2,U0);
  CROSS(N2,E1,E2);
  d2=-DOT(N2,U0);
  /* plane equation 2: N2.X+d2=0 */

  /* put V0,V1,V2 into plane equation 2 */
  dv0=DOT(N2,V0)+d2;
  dv1=DOT(N2,V1)+d2;
  dv2=DOT(N2,V2)+d2;

#if USE_EPSILON_TEST==TRUE
  if(FABS(dv0)<EPSILON) dv0=0.0;
  if(FABS(dv1)<EPSILON) dv1=0.0;
  if(FABS(dv2)<EPSILON) dv2=0.0;
#endif

  dv0dv1=dv0*dv1;
  dv0dv2=dv0*dv2;

  if(dv0dv1>0.0f && dv0dv2>0.0f) /* same sign on all of them + not equal 0 ? */
    return 0;                    /* no intersection occurs */

  /* compute direction of intersection line */
  CROSS(D,N1,N2);

  /* compute and index to the largest component of D */
  max=(Real)FABS(D[0]);
  index=0;
  bb=(Real)FABS(D[1]);
  cc=(Real)FABS(D[2]);
  if(bb>max) max=bb,index=1;
  if(cc>max) max=cc,index=2;

  /* this is the simplified projection onto L*/
  vp0=V0[index];
  vp1=V1[index];
  vp2=V2[index];

  up0=U0[index];
  up1=U1[index];
  up2=U2[index];

  /* compute interval for triangle 1 */
  NEWCOMPUTE_INTERVALS(vp0,vp1,vp2,dv0,dv1,dv2,dv0dv1,dv0dv2,a,b,c,x0,x1);

  /* compute interval for triangle 2 */
  NEWCOMPUTE_INTERVALS(up0,up1,up2,du0,du1,du2,du0du1,du0du2,d,e,f,y0,y1);

  xx=x0*x1;
  yy=y0*y1;
  xxyy=xx*yy;

  tmp=a*xxyy;
  isect1[0]=tmp+b*x1*yy;
  isect1[1]=tmp+c*x0*yy;

  tmp=d*xxyy;
  isect2[0]=tmp+e*xx*y1;
  isect2[1]=tmp+f*xx*y0;

  SORT(isect1[0],isect1[1]);
  SORT(isect2[0],isect2[1]);

  if(isect1[1]<isect2[0] || isect2[1]<isect1[0]) return 0;
  return 1;
}

/* sort so that a<=b */
#define SORT2(a,b,smallest)       \
             if(a>b)       \
             {             \
               Real c;    \
               c=a;        \
               a=b;        \
               b=c;        \
               smallest=1; \
             }             \
             else smallest=0;


inline void isect2(Real VTX0[3],Real VTX1[3],Real VTX2[3],Real VV0,Real VV1,Real VV2,
	    Real D0,Real D1,Real D2,Real *isect0,Real *isect1,Real isectpoint0[3],Real isectpoint1[3]) 
{
  Real tmp=D0/(D0-D1);          
  Real diff[3];
  *isect0=VV0+(VV1-VV0)*tmp;         
  SUB(diff,VTX1,VTX0);              
  MULT(diff,diff,tmp);               
  ADD(isectpoint0,diff,VTX0);        
  tmp=D0/(D0-D2);                    
  *isect1=VV0+(VV2-VV0)*tmp;          
  SUB(diff,VTX2,VTX0);                   
  MULT(diff,diff,tmp);                 
  ADD(isectpoint1,VTX0,diff);          
}



inline int compute_intervals_isectline(Real VERT0[3],Real VERT1[3],Real VERT2[3],
				       Real VV0,Real VV1,Real VV2,Real D0,Real D1,Real D2,
				       Real D0D1,Real D0D2,Real *isect0,Real *isect1,
				       Real isectpoint0[3],Real isectpoint1[3])
{
  if(D0D1>0.0f)                                        
  {                                                    
    /* here we know that D0D2<=0.0 */                  
    /* that is D0, D1 are on the same side, D2 on the other or on the plane */
    isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,isect0,isect1,isectpoint0,isectpoint1);
  } 
  else if(D0D2>0.0f)                                   
    {                                                   
    /* here we know that d0d1<=0.0 */             
    isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,isect0,isect1,isectpoint0,isectpoint1);
  }                                                  
  else if(D1*D2>0.0f || D0!=0.0f)   
  {                                   
    /* here we know that d0d1<=0.0 or that D0!=0.0 */
    isect2(VERT0,VERT1,VERT2,VV0,VV1,VV2,D0,D1,D2,isect0,isect1,isectpoint0,isectpoint1);   
  }                                                  
  else if(D1!=0.0f)                                  
  {                                               
    isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,isect0,isect1,isectpoint0,isectpoint1); 
  }                                         
  else if(D2!=0.0f)                                  
  {                                                   
    isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,isect0,isect1,isectpoint0,isectpoint1);     
  }                                                 
  else                                               
  {                                                   
    /* triangles are coplanar */    
    return 1;
  }
  return 0;
}

#define COMPUTE_INTERVALS_ISECTLINE(VERT0,VERT1,VERT2,VV0,VV1,VV2,D0,D1,D2,D0D1,D0D2,isect0,isect1,isectpoint0,isectpoint1) \
  if(D0D1>0.0f)                                         \
  {                                                     \
    /* here we know that D0D2<=0.0 */                   \
    /* that is D0, D1 are on the same side, D2 on the other or on the plane */ \
    isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,&isect0,&isect1,isectpoint0,isectpoint1);          \
  }                                                     
#if 0
  else if(D0D2>0.0f)                                    \
  {                                                     \
    /* here we know that d0d1<=0.0 */                   \
    isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,&isect0,&isect1,isectpoint0,isectpoint1);          \
  }                                                     \
  else if(D1*D2>0.0f || D0!=0.0f)                       \
  {                                                     \
    /* here we know that d0d1<=0.0 or that D0!=0.0 */   \
    isect2(VERT0,VERT1,VERT2,VV0,VV1,VV2,D0,D1,D2,&isect0,&isect1,isectpoint0,isectpoint1);          \
  }                                                     \
  else if(D1!=0.0f)                                     \
  {                                                     \
    isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,&isect0,&isect1,isectpoint0,isectpoint1);          \
  }                                                     \
  else if(D2!=0.0f)                                     \
  {                                                     \
    isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,&isect0,&isect1,isectpoint0,isectpoint1);          \
  }                                                     \
  else                                                  \
  {                                                     \
    /* triangles are coplanar */                        \
    coplanar=1;                                         \
    return coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2);      \
  }
#endif

int tri_tri_intersect_with_isectline(Real V0[3],Real V1[3],Real V2[3],
				     Real U0[3],Real U1[3],Real U2[3],int *coplanar,
				     Real isectpt1[3],Real isectpt2[3])
{
  Real E1[3],E2[3];
  Real N1[3],N2[3],d1,d2;
  Real du0,du1,du2,dv0,dv1,dv2;
  Real D[3];
  Real isect1[2], isect2[2];
  Real isectpointA1[3],isectpointA2[3];
  Real isectpointB1[3],isectpointB2[3];
  Real du0du1,du0du2,dv0dv1,dv0dv2;
  short index;
  Real vp0,vp1,vp2;
  Real up0,up1,up2;
  Real b,c,max;
  //  Real tmp,diff[3];
  int smallest1,smallest2;
  
  /* compute plane equation of triangle(V0,V1,V2) */
  SUB(E1,V1,V0);
  SUB(E2,V2,V0);
  CROSS(N1,E1,E2);
  d1=-DOT(N1,V0);
  /* plane equation 1: N1.X+d1=0 */

  /* put U0,U1,U2 into plane equation 1 to compute signed distances to the plane*/
  du0=DOT(N1,U0)+d1;
  du1=DOT(N1,U1)+d1;
  du2=DOT(N1,U2)+d1;

  /* coplanarity robustness check */
#if USE_EPSILON_TEST==TRUE
  if(fabs(du0)<EPSILON) du0=0.0;
  if(fabs(du1)<EPSILON) du1=0.0;
  if(fabs(du2)<EPSILON) du2=0.0;
#endif
  du0du1=du0*du1;
  du0du2=du0*du2;

  if(du0du1>0.0f && du0du2>0.0f) /* same sign on all of them + not equal 0 ? */
    return 0;                    /* no intersection occurs */

  /* compute plane of triangle (U0,U1,U2) */
  SUB(E1,U1,U0);
  SUB(E2,U2,U0);
  CROSS(N2,E1,E2);
  d2=-DOT(N2,U0);
  /* plane equation 2: N2.X+d2=0 */

  /* put V0,V1,V2 into plane equation 2 */
  dv0=DOT(N2,V0)+d2;
  dv1=DOT(N2,V1)+d2;
  dv2=DOT(N2,V2)+d2;

#if USE_EPSILON_TEST==TRUE
  if(fabs(dv0)<EPSILON) dv0=0.0;
  if(fabs(dv1)<EPSILON) dv1=0.0;
  if(fabs(dv2)<EPSILON) dv2=0.0;
#endif

  dv0dv1=dv0*dv1;
  dv0dv2=dv0*dv2;
        
  if(dv0dv1>0.0f && dv0dv2>0.0f) /* same sign on all of them + not equal 0 ? */
    return 0;                    /* no intersection occurs */

  /* compute direction of intersection line */
  CROSS(D,N1,N2);

  /* compute and index to the largest component of D */
  max=fabs(D[0]);
  index=0;
  b=fabs(D[1]);
  c=fabs(D[2]);
  if(b>max) max=b,index=1;
  if(c>max) max=c,index=2;

  /* this is the simplified projection onto L*/
  vp0=V0[index];
  vp1=V1[index];
  vp2=V2[index];
  
  up0=U0[index];
  up1=U1[index];
  up2=U2[index];

  /* compute interval for triangle 1 */
  *coplanar=compute_intervals_isectline(V0,V1,V2,vp0,vp1,vp2,dv0,dv1,dv2,
				       dv0dv1,dv0dv2,&isect1[0],&isect1[1],isectpointA1,isectpointA2);
  if(*coplanar) return coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2);     


  /* compute interval for triangle 2 */
  compute_intervals_isectline(U0,U1,U2,up0,up1,up2,du0,du1,du2,
			      du0du1,du0du2,&isect2[0],&isect2[1],isectpointB1,isectpointB2);

  SORT2(isect1[0],isect1[1],smallest1);
  SORT2(isect2[0],isect2[1],smallest2);

  if(isect1[1]<isect2[0] || isect2[1]<isect1[0]) return 0;

  /* at this point, we know that the triangles intersect */

  if(isect2[0]<isect1[0])
  {
    if(smallest1==0) { SET(isectpt1,isectpointA1); }
    else { SET(isectpt1,isectpointA2); }

    if(isect2[1]<isect1[1])
    {
      if(smallest2==0) { SET(isectpt2,isectpointB2); }
      else { SET(isectpt2,isectpointB1); }
    }
    else
    {
      if(smallest1==0) { SET(isectpt2,isectpointA2); }
      else { SET(isectpt2,isectpointA1); }
    }
  }
  else
  {
    if(smallest2==0) { SET(isectpt1,isectpointB1); }
    else { SET(isectpt1,isectpointB2); }

    if(isect2[1]>isect1[1])
    {
      if(smallest1==0) { SET(isectpt2,isectpointA2); }
      else { SET(isectpt2,isectpointA1); }      
    }
    else
    {
      if(smallest2==0) { SET(isectpt2,isectpointB2); }
      else { SET(isectpt2,isectpointB1); } 
    }
  }
  return 1;
}




//
// class Triangle3 implementation
//


bool Triangle3::intersect(const Triangle3& t) const
{
  const Triangle3& t2(*this);

  // C doesn't know about const
  Real* t1p1 = const_cast<Real*>(t.p1().c_array());
  Real* t1p2 = const_cast<Real*>(t.p2().c_array());
  Real* t1p3 = const_cast<Real*>(t.p3().c_array());
  Real* t2p1 = const_cast<Real*>(t2.p1().c_array());
  Real* t2p2 = const_cast<Real*>(t2.p2().c_array());
  Real* t2p3 = const_cast<Real*>(t2.p3().c_array());

  return (bool)NoDivTriTriIsect(t1p1,t1p2,t1p3,
				t2p1,t2p2,t2p3);

  /* Old code to compute intersection point
  const Triangle& t1(*this);
  const Triangle& t2(t);
  Plane p1(t1);
  Int other_side=0;
  {
    Real f1=p1.classify(t2[1]);
    Real f2=p1.classify(t2[2]);
    Real f3=p1.classify(t2[3]);
    Real f12=f1*f2;
    Real f23=f2*f3;
    if (f12>0.0 && f23>0.0) return Vector3::zero;
    other_side=(f12<0.0?(f23<0.0?1:0):2);
  }
  Plane p2(t2);
  Vector3 n12(p1.normal+p2.normal);
  TriangleDesc td2(t2,p2);
  const Vector3& a2=td2[other_side+2];
  const Vector3& b2=td2[other_side+1];
  const Vector3& c2=td2[other_side+3];
  Real t21=-(p1.d+p2.d+a2*n12)/((b2-a2)*n12);
  TriangleDesc td1(t1,p1);
  Vector3 P21(a2+t21*(b2-a2));
  if (td1.pointInTri(P21)) return P21;
  Real t22=-(p1.d+p2.d+c2*n12)/((b2-c2)*n12);
  Vector3 P22(c2+t22*(b2-c2));
  if (td1.pointInTri(P22)) return P22;
  
  {
    Real f1=p2.classify(t1[1]);
    Real f2=p2.classify(t1[2]);
    Real f3=p2.classify(t1[3]);
    Real f12=f1*f2;
    Real f23=f2*f3;
    if (f12>0.0 && f23>0.0) return Vector3::zero;
    other_side=(f12<0.0?(f23<0.0?1:0):2);
  }
  const Vector3& a1=td1[other_side+2];
  const Vector3& b1=td1[other_side+1];
  const Vector3& c1=td1[other_side+3];
  Real t11=-(p1.d+p2.d+a1*n12)/((b1-a1)*n12);
  Vector3 P11(a1+t11*(b1-a1));
  if (td2.pointInTri(P11)) return P11;
  Real t12=-(p1.d+p2.d+c1*n12)/((b1-c1)*n12);
  Vector3 P12(c1+t12*(b1-c1));
  if (td2.pointInTri(P12)) return P12;
  return Point3::zero;
  */
}


bool Triangle3::contact(const Triangle3& t, Contact& contact) const
{
  const Triangle3& t2(*this);

  // C doesn't know about const
  Real* t1p1 = const_cast<Real*>(t.p1().c_array());
  Real* t1p2 = const_cast<Real*>(t.p2().c_array());
  Real* t1p3 = const_cast<Real*>(t.p3().c_array());
  Real* t2p1 = const_cast<Real*>(t2.p1().c_array());
  Real* t2p2 = const_cast<Real*>(t2.p2().c_array());
  Real* t2p3 = const_cast<Real*>(t2.p3().c_array());

  int coplanar = 0;
  Point3 i1, i2;
  int intersected = tri_tri_intersect_with_isectline(t1p1,t1p2,t1p3,
						     t2p1,t2p2,t2p3,
						     &coplanar, 
						     i1.c_array(), i2.c_array());
  if (!intersected) {
    contact.type = Contact::None;
    return false;
  }
  
    if (!coplanar) {
    if (i1.equals(i2)) {
      contact.type = Contact::Point;
      contact.point = i1;
      contact.depth = 0;
  }
    else {
      contact.type = Contact::Segment;
      contact.segment = Segment3(i1,i2);
      //!!! calculate depth
      contact.depth = 0;
    }
  }
  else { // co-planar triangles
    
    // First check the common case, where one triangle in completely
    //  inside the other

    


  }

  return true;
}
