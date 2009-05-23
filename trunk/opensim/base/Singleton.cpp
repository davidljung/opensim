/****************************************************************************
  Copyright (C)2001 by Andrei Alexandrescu

  This file is a derivative of a file from the Loki Library written by
  Andrei Alexandrescu.  It was distributed by him under the terms
  listed below (titled Loki Original Distribution Terms).
  In accordance with the terms, this distribution contains the copyright
  notice here and the copyright notice and permission notice
  in supporting documentation.  The terms do *not* require
  redistribution under those same terms.  This code is distributed
  to you under the terms of the GNU General Public License (GPL) 
  below.  The GPL does not require you to maintain the terms of
  the Loki Original Distribution Terms, but you are encouraged to do so.

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
  
 ****************************************************************************
  Loki Original Distribution Terms:

  The Loki Library
  Copyright (c) 2001 by Andrei Alexandrescu
  This code accompanies the book:
  Alexandrescu, Andrei. "Modern C++ Design: Generic Programming and Design 
      Patterns Applied". Copyright (c) 2001. Addison-Wesley.
  Permission to use, copy, modify, distribute and sell this software for any 
      purpose is hereby granted without fee, provided that the above copyright 
      notice appear in all copies and that both that copyright notice and this 
      permission notice appear in supporting documentation.
  The author or Addison-Welsey Longman make no representations about the 
      suitability of this software for any purpose. It is provided "as is" 
      without express or implied warranty.
 ****************************************************************************

  $Id: Singleton.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.1 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

// Last update: June 20, 2001

#include <base/Singleton>

using namespace base::Private;

base::Private::TrackerArray base::Private::pTrackerArray = 0;
unsigned int base::Private::elements = 0;

////////////////////////////////////////////////////////////////////////////////
// function AtExitFn
// Ensures proper destruction of objects with longevity
////////////////////////////////////////////////////////////////////////////////

void base::Private::AtExitFn()
{
    assert(elements > 0 && pTrackerArray != 0);
    // Pick the element at the top of the stack
    LifetimeTracker* pTop = pTrackerArray[elements - 1];
    // Remove that object off the stack
    // Don't check errors - realloc with less memory 
    //     can't fail
    pTrackerArray = static_cast<TrackerArray>(std::realloc(
        pTrackerArray, --elements));
    // Destroy the element
    delete pTop;
}

////////////////////////////////////////////////////////////////////////////////
// Change log:
// June 20, 2001: ported by Nick Thurn to gcc 2.95.3. Kudos, Nick!!!
// Feb 25, 2002: David Jung: integrated into larger project - changed to 
//                namespace base and changed header comments to reflect GPL
//                distribution
////////////////////////////////////////////////////////////////////////////////
