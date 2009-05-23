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
  
  $Id: Time.cpp 1046 2004-02-27 19:20:16Z jungd $
  $Revision: 1.4 $
  $Date: 2004-02-27 14:20:16 -0500 (Fri, 27 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/Time>

#include <base/Serializer>

#include <iostream>

extern "C" {
#include <sys/time.h>
#include <sys/uio.h>
#include <unistd.h>
#include <signal.h>
}

using base::Time;


Time* Time::time=0;
long Time::baseSecs;
long Time::baseMicros;
Time Time::lastNow;
long Time::resolution;


Time::Time()
{
  const SInt minResolution = 20000; // 1/50th sec
  
  if (time==0) {
    // initialise statics
    time = this;
    
    /*
    // We use itimers to monitor passing time - but don't make use of signals for notification
    //  set the itimer SIGALRM to be ignored
    struct sigaction act;
    act.sa_handler = SIG_IGN;
    sigemptyset(&act.sa_mask);
    act.sa_flags=0;
    sigaction(SIGALRM, &act, 0);
    */

    // Get current system time to use as base (so the real-time starts at 0)
    struct timeval now;
    gettimeofday(&now,0);
    baseSecs = now.tv_sec;
    baseMicros = now.tv_usec;

    usleep(1);
    
    gettimeofday(&now,0);
    resolution = (now.tv_usec - baseMicros) + microsPerSec*(now.tv_sec - baseSecs);

    if (resolution > minResolution) {
      
      // it is possible the timer wrapped, so just do the calculation again, just in case
      gettimeofday(&now,0);
      baseSecs = now.tv_sec;
      baseMicros = now.tv_usec;
      usleep(1);
      
      gettimeofday(&now,0);
      resolution = (now.tv_usec - baseMicros) + microsPerSec*(now.tv_sec - baseSecs);
      
      if (resolution > minResolution) {   
        Logln("WARNING: time resolution is only " << resolution << "us.  It may be too coarse.");
      }
      
    }
    //Logln("Time resolution: " << resolution << "us ; " << (Real(resolution)/Real(1000000)) << "s" << " ; 1/" << 1.0/(Real(resolution)/Real(1000000)) << "th of a sec" );
  }

  secs=micros=0;
}



Time::Time(Real secs)
{
  this->secs = long(secs);
  micros = long((secs-Real(this->secs))*Real(microsPerSec));
}


Time::~Time()
{
  if (this==time) {
    // cleanup statics

    time=0;
  }
}


const Time& Time::now()
{
  struct timeval now;
  gettimeofday(&now,0);
  lastNow.secs = now.tv_sec - baseSecs;
  lastNow.micros = now.tv_usec - baseMicros;
  if (lastNow.micros < 0) {
    lastNow.micros += microsPerSec;
    lastNow.secs--;
  }
  return lastNow;
}


bool Time::less(const Time& t1, const Time& t2)
{
  if (t1.secs < t2.secs)
    return true;
  else
    if (t1.secs == t2.secs)
      return (t1.micros < t2.micros);

  return false;
}


bool Time::greater(const Time& t1, const Time& t2)
{
  if (t1.secs > t2.secs)
    return true;
  else
    if (t1.secs == t2.secs)
      return (t1.micros > t2.micros);

  return false;
}


void Time::sleep(const Time& t)
{
  if (t.secs > 0) ::sleep(t.secs);
  if (t.micros > 0) usleep(t.micros);
}


void Time::sleep(long secs, long micros)
{
  sleep(Time(secs,micros));
}


void Time::normalize()
{
  if (secs > 0) {
    while (micros < 0) {
      micros+= microsPerSec;
      secs--;
    }
    while (micros >= microsPerSec) {
      micros-= microsPerSec;
      secs++;
    }
  }
  
  if (secs == 0) {
    while (micros >= microsPerSec) {
      micros-= microsPerSec;
      secs++;
    }
    while (micros <= -microsPerSec) {
      micros+= microsPerSec;
      secs--;
    }
  }

  if (secs < 0) {
    while (micros > 0) {
      micros-= microsPerSec;
      secs++;
    }
    while (micros <= -microsPerSec) {
      micros+= microsPerSec;
      secs--;
    }
  }

  if (secs!=0) {
    if ((secs>0) && (micros<0)) {
      micros+=microsPerSec;
      secs--;
    }
    if ((secs<0) && (micros>0)) {
      micros-=microsPerSec;
      secs++;
    }
  }

}


std::ostream& base::operator<<(std::ostream& out, const Time& t)
{
  return out << t.secs << "s:" << t.micros << "us";
}


Time& Time::operator-=(const Time& t)
{
  long ds = secs - t.secs;
  long dm = micros - t.micros;

  Time r(ds,dm);
  r.normalize();
  *this = r;
  return *this;
}


Time& Time::operator+=(const Time& t)
{
  long ss = secs + t.secs;
  long sm = micros + t.micros;

  Time r(ss,sm);
  r.normalize();
  *this = r;
  return *this;
}


Time& Time::operator*=(Real s)
{
  Real rs = Real(secs)*s;
  long ss = long(rs);
  Real ds = rs - ss;
  
  long sm = long( Real(micros)*s );
  sm += long( ds*microsPerSec );

  Time r(ss,sm);
  r.normalize();
  *this = r;
  return *this;
}


void Time::serialize(Serializer& s)
{ 
  s(LInt(secs),"secs"); s(LInt(micros),"micros"); 
}


