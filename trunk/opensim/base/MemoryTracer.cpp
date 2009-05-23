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
  
  $Id: MemoryTracer.cpp 1048 2004-02-27 19:22:02Z jungd $
  $Revision: 1.4 $
  $Date: 2004-02-27 14:22:02 -0500 (Fri, 27 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/MemoryTracer>

#include <string>
#include <iostream>

using std::string;

using base::MemoryTracer;



// if defined, will cause a menu to be displayed when a memory problem is
//  detected (stopping the program)
#define _INTERACTIVE_ 

// if defined and not _INTERACTIVE_, then a memory problem will throw a
//  std::bad_alloc exception (possibly from within new or delete)
#define _THROWEXCEPTION_

// if defined, calls to allocate, validate, release & invalidate will report arguments (on every call)
//  (even more verbose that using _TRACEOUTPUT_ )
//#define _MEMORYTRACEDEBUG_



base::Byte MemoryTracer::prefill[MemoryTracer::Filling]; // Filler for integrity check
base::Byte MemoryTracer::initialized = 0;
bool MemoryTracer::cleanedUp = false;
Int MemoryTracer::entry = 0;
Int MemoryTracer::fullCheckCounter = 0;
base::LInt MemoryTracer::allocCount = 0;
base::LInt MemoryTracer::deallocCount = 0;
base::LInt MemoryTracer::checkCount = 0;
MemoryTracer::AllocList MemoryTracer::mlist; // the allocated memory list
MemoryTracer::AllocList MemoryTracer::dlist; // list of memory allocated and then deallocated (so we can catch double delete's)
base::Byte MemoryTracer::postfill[Filling]; // Filler for integrity check

const int MemoryTracer::IsArray = 0xabcd;
const int MemoryTracer::IsNotArray = 0xfcad;

inline void MemoryTracer::Logerr(char* errstr) 
{
  // can't use base's Logln because that uses iostreams (which need new/delete)
  char str[256];
  sprintf(str,"MemoryTracer Log: %s\n",errstr);
  fprintf(stderr,str);
}


// this is not meant to be an atomic lock, just something to help flag multiple thread entry
inline void MemoryTracer::entrylock() 
{
  if (entry != 0) {
    Logerr("Multiple thread entry detected!  MemoryTracer is not threadsafe - please restrict use to a single thread at a time.");
    handleError();
  }
  else
    entry++;
}

inline void MemoryTracer::exitunlock()
{
  entry--;
}

//
// Home grown doubly linked list
//  (Unfortunately, MemoryTracer can't used STL because it must not call the global
//   new or delete.  Using STL containers with malloc_alloc as their
//   allocator should work - but, alas, it doesn't).


void MemoryTracer::AllocList::push_front(AllocEntry* e)
{
  if (head == 0) {
    head = tail = e;
    e->next = e->prev = 0;
  }
  else {
    AllocEntry* oldhead = head;
    head = e;
    head->next = oldhead;
    head->prev = 0;
    oldhead->prev = head;
  }
  len++;
}


void MemoryTracer::AllocList::push_back(AllocEntry* e)
{
  if (head == 0) {
    head = tail = e;
    e->next = e->prev = 0;
  }
  else {
    AllocEntry* oldtail = tail;
    tail = e;
    tail->next = 0;
    tail->prev = oldtail;
    oldtail->next = tail;
  }
  len++;
}

MemoryTracer::AllocEntry* MemoryTracer::AllocList::pop_front()
{
  AllocEntry* oldhead = head;
  if (head != 0) {
    head = head->next;
    if (head)
      head->prev = 0;
    else
      tail = 0;
    len--;
  }
  return oldhead;
}


MemoryTracer::AllocEntry* MemoryTracer::AllocList::pop_back()
{
  AllocEntry* oldtail = tail;
  if (tail != 0) {
    tail = tail->prev;
    if (tail)
      tail->next = 0;
    else
      head = 0;
    len--;
  }
  return oldtail;
}


void MemoryTracer::AllocList::remove(AllocEntry* e)
{
  if (e == tail) {
    pop_back();
  }
  else if (e == head) {
    pop_front();
  }
  else {
    e->next->prev = e->prev;
    e->prev->next = e->next;
    len--;
  }
}


MemoryTracer::AllocEntry* MemoryTracer::AllocList::find(void* address, AllocEntry* from) const
{ 
  Byte* addr = (Byte*)address;
  AllocEntry* m = (from==0)?head:from;
  while (m) {
    if (!m->isExternal()) {
      if (addr == m->address+Filling)
	return m;
    }
    else
      if (addr == m->address)
	return m;
    
    m = m->next;
  }
  return 0;
}



  void MemoryTracer::menu() throw(std::bad_alloc)
  {
    char ans[32];
    bool quit=false;
    while (!quit) {
      printf("\n");
      printf("Choose: (a)llocated memory list        (n)amed allocated memory\n");
      printf("        (r)eciently deallocated memory (d)ivide by 0 crash (SIGFPE)\n");
      printf("         e(x)it   (t)hrow bad_alloc    (c)continue? ");
      scanf("%s",ans);
      switch (tolower(ans[0])) {
      case 'a': dump(); break;
      case 'n': dumpNamed(); break;
      case 'r': dumpDeallocated(); break;
      case 'd': causefault(); break;
      case 'x': exit(-1); break;
      case 't': throw std::bad_alloc(); break;
      case 'c': quit=true; break;
      default: ;
      }
    }
  }


  void MemoryTracer::handleError() throw(std::bad_alloc)
  {
#ifdef _INTERACTIVE_
    menu();
#else
#ifdef _THROWEXCEPTION_
    throw std::bad_alloc();
#endif
#endif
  }


  // Does a dividie by zero - Debugger can catch this to provide a stack trace
  int MemoryTracer::causefault() 
  {
    int i=5, j=5;
    // try not to let compiler optimise this out of existance 
    j = i/(j-i); // Divide by 0 to get debugger's attention
    return j;
  }
  
  
  // Checks all currently allocated memory for overwrites
  bool MemoryTracer::checkAllMemory() throw(std::bad_alloc)
  {
    bool ok = true;
#ifdef _TRACEMEMORY_
    checkCount++;
    AllocEntry* m = mlist.front();
    int size, i,j;
    char errstr[256];
    
    while (m) {
      if (!m->isOK()) {
	Logerr("Memory allocation record has been corrupted (AllocEntry).");
	bool prefillOK = true;
	for(Int i=0; i<Filling; i++) 
	  if (m->prefill[i] != Filler) prefillOK = false;
	bool postfillOK = true;
	for(Int i=0; i<Filling; i++) 
	  if (m->postfill[i] != Filler) postfillOK = false;
	sprintf(errstr,"Entry: prefill:%s postfill:%s size:%6d isMagic:%s",
		prefillOK?"OK":"BAD", postfillOK?"OK":"BAD",m->size,m->isMagic()?"OK":"BAD");
	Logerr(errstr);
	m=0;
	ok=false;
      } 
      else {
	
	if (!m->isExternal()) {
	  Byte* mem = (Byte*)m->address;
	  size = m->size;
	  for(i=0; (i<Filling) && ok; i++) {
	    if ( *(mem+i) != Filler ) {
	      sprintf(errstr,"Memory before allocation %x (%s) has been overwritten",(Int)mem+Filling,m->name);
	      for(j=0; j<Filling; j++) {
		sprintf(errstr,"addr(m-%d) = %d (should be %d)",Filling-j,(unsigned int)*(mem+i),Filler);
		Logerr(errstr);
	      }
	      ok=false;
	    }
	    if ( *(mem+Filling+size+i) != Filler ) {
	      sprintf(errstr,"Memory after allocation %x (%s) has been overwritten",(Int)mem+Filling,m->name);
	      Logerr(errstr);
	      for(j=0; j<Filling; j++) {
		sprintf(errstr,"addr(m+size+%d) = %d (should be %d)",j,(Int)*(mem+size+Filling+i),Filler);
		Logerr(errstr);
	      }
	      ok=false;
	    }
	    
	  }
	} 
	
	m = m->next;
      }
      
    }
    
    if (!ok) handleError();
#endif    
    return ok;
  }
  

  // allocate memory (returns 0 on error)
  void* MemoryTracer::allocate(int size, const string& name, bool isArray) throw(std::bad_alloc)
  {
    entrylock();
    if (!isOK()) {
      Logerr("memory allocation records have been corrupted (MemoryTracer).");
      exitunlock();
      return 0;
    }

    int i;
    AllocEntry* ne;
    if (size <= 0) {
      Logerr("passed size <= 0.");
      handleError();
      return 0;
    }
    void* mem = malloc(size + Filling*2);
    if (mem == 0) return 0;

#if (_FULLCHECKFREQUENTY_ > 0)
    if ((++fullCheckCounter % _FULLCHECKFREQUENTY_) == 0) {
      fullCheckCounter=0;
      checkAllMemory();
    }
#endif

    ne = (AllocEntry*)malloc(sizeof(AllocEntry));
    if (ne == 0) {
      free(mem);
      Logerr("no memory available to allocate (for entry).");
      return 0;
    }
    ne->fill();

    // fill 'Filling' bytes before and after returned memory area with 'Filler' byte 
    for(i=0; i< Filling; i++) {
      *(((Byte*)mem)+i) = Filler;
      *(((Byte*)mem)+size+Filling+i) = Filler;
    }

    ne->size = size;
    ne->name = 0;
    ne->setIsArray(isArray);
    
    if (name.size() > 0) {
      ne->name = (char*)malloc(name.size()+1);
      if (ne->name == 0) {
	Logerr("no memory available to allocate name.");
      }
      else
	strcpy(ne->name, name.c_str());
    }

    ne->setMagic();
    ne->address = (Byte*)mem;

    mlist.push_front(ne);
    
    allocCount++;
    exitunlock();
    void* addr = ((Byte*)mem)+Filling;
#ifdef _MEMORYTRACEDEBUG_
    char logstr[80];
    sprintf(logstr, "allocated(size:%d, name:%s, isArray:%s) = %x", size, (name.size()>0)?name.c_str():"<unnamed>",isArray?"TRUE":"FALSE",Int(addr));
    Logerr(logstr);
#endif
    return addr;
  }
  


  // registers an externally allocated region as valid (for Checks).
  bool MemoryTracer::validate(void* address, int size, const string& name) throw(std::bad_alloc)
  {
    entrylock();
    if (!isOK()) {
      Logerr("memory allocation records have been corrupted.");
      exitunlock();
      return false;
    }

    AllocEntry* ne;
    char errstr[256];

    if (size < 0) {
      Logerr("passed size < 0.");
      exitunlock();
      handleError();
      return false;
    }
    
#if (_FULLCHECKFREQUENTY_ > 0)
    if ((++fullCheckCounter % _FULLCHECKFREQUENTY_) == 0) {
      fullCheckCounter=0;
      checkAllMemory();
    }
#endif
    
    // first look to see if this address is already registered as valid,
    // if so, return - nothing more to do 
    AllocEntry* m = mlist.find(address);
    if (m) {
      if (!m->isExternal()) {
	if (address == m->address + Filling) {
	  sprintf(errstr,"memory at address %x (%s) was allocated with allocate(), so cannot be validated with validate()\n",
		  Int(m->address+Filling),(m->name==0)?"<unnamed>":m->name);
	  Logerr(errstr);
	  exitunlock();
	  return false;
	}
      }
      else
	if (address == m->address) {
	  exitunlock();
	  return true;
	}
    }
    
    ne = (AllocEntry*)malloc(sizeof(AllocEntry));
    if (ne == 0) {
      Logerr("no memory available to allocate (for entry).");
      exitunlock();
      return false;
    }
    
    ne->size = size;
    ne->name = 0;

    if (name.size() > 0) {
      ne->name = (char*)malloc(name.size()+1);
      if (ne->name == 0) {
	Logerr("no memory available to allocate (for name).");
      }
      else
	strcpy(ne->name, name.c_str());
      
    }
    ne->setMagic();
    ne->setExternal();
    ne->setIsArray(false);
    ne->address = (Byte*)address;
    
    mlist.push_front(ne);

    exitunlock();

#ifdef _MEMORYTRACEDEBUG_
    char logstr[80];
    sprintf(logstr, "validated(address:%x, size:%d, name:%s)", Int(address), size, (name.size()>0)?name.c_str():"<unnamed>");
    Logerr(logstr);
#endif

    return true;
  }
  
  

  // free allocated memory
  bool MemoryTracer::release(void* address, bool isArray, int size) throw(std::bad_alloc)
  {
    entrylock();
    if (!isOK()) {
      Logerr("memory allocation records have been corrupted.");
      exitunlock();
      return false;
    }

    if (cleanedUp) return true;

    Byte* addr = (Byte*)address;
    char errstr[256];

#if (_FULLCHECKFREQUENTY_ > 0)
    if ((++fullCheckCounter % _FULLCHECKFREQUENTY_) == 0) {
      fullCheckCounter=0;
      checkAllMemory();
    }
#endif
    
    AllocEntry* m = mlist.find(address);
    bool found = (m!=0);
    char name[128];
    strcpy(name, "<unknown>");

    if (found) {
      
      if (addr == m->address+Filling) {
	if ((size!=-1) && (size!=m->size)) {
	  sprintf(errstr,"size given (%d) for region at %x doesn't match that allocated (%d)",
		  size,Int(address),m->size);
	  Logerr(errstr);
	}
	if (m->isArray() != isArray) {
	  if (m->isArray()) {
	    sprintf(errstr,"region at %x was allocated as an array but not free'd as one",
		    Int(address));
	    Logerr(errstr);
	    handleError();
	  }
	  else {
	    sprintf(errstr,"region at %x was free'd as an array but not allocated as one",
		    Int(address));
	    Logerr(errstr);
	    handleError();
	  }
	}
	
#ifdef _MEMORYTRACEDEBUG_
	if (m->name)
	  strncpy(name, m->name, 127);
#endif
	
	// remove from list 
	mlist.remove(m);

	free(m->address); // free memory

	dlist.push_front(m); // add to deallocated list
	if (dlist.size() > DList_Max) {
	  AllocEntry* dm = dlist.pop_back();
	  if (dm->name!=0) 
	    free(dm->name);
	  free(dm);
	}
      }
      else {
	if ((addr == m->address) && (m->isExternal())) {
	  sprintf(errstr,"memory at address %x was not allocated with allocate(), but was validated with validate(). (will invalidate without freeing).",(Int)address);
	  Logerr(errstr);
	  exitunlock();
	  invalidate(address,size);
	  entrylock();
	}
      }
    }
    else { // not found
      sprintf(errstr,"no memory allocated at address %x.",(Int)address);
      Logerr(errstr);

      // check to see if the memory was previously deallocated
      AllocEntry* from = dlist.front();
      AllocEntry* dm;
      do {
	dm = dlist.find(address,from);
	if (dm) {
	  Byte* addr = (dm->isExternal()?dm->address:dm->address+Filling);
	  sprintf(errstr,"memory was previously allocated at address %x as %s (size %d) - but has since been deallocated.",
		  Int(addr),((dm->name==0)?"<unnamed>":dm->name),dm->size);
	  Logerr(errstr);
	  from = dm->next;
	  if (from==0) from = dlist.back();
	}
      } while (dm);

      //dump();
      checkAllMemory();
      exitunlock();
      handleError();
      return found;
    }

    deallocCount++;
    exitunlock();

#ifdef _MEMORYTRACEDEBUG_
    char logstr[80];
    sprintf(logstr, "released(address:%x, isArray:%s, size:%6d, name:%s)", Int(address), isArray?"TRUE":"FALSE", size, name);
    Logerr(logstr);
#endif

    return found;
  }
  


  // invalidate externally allocated region, previously registered
  //   with validate()
  bool MemoryTracer::invalidate(void* address, int size) throw(std::bad_alloc)
  {
    entrylock();
    if (!isOK()) {
      Logerr("memory allocation records have been corrupted.");
      exitunlock();
      return false;
    }

    if (cleanedUp) return true;

    Byte* addr = (Byte*)address;
    char errstr[256];
    
#if (_FULLCHECKFREQUENTY_ > 0)
    if ((++fullCheckCounter % _FULLCHECKFREQUENTY_) == 0) {
      fullCheckCounter=0;
      checkAllMemory();
    }
#endif

    AllocEntry* m = mlist.find(address);
    bool found = (m!=0);
    char name[128];
    strcpy(name, "<unknown>");

    if (found) {
      
      if (addr == m->address) {
	if ((size!=-1) && (size!=m->size)) {
	  sprintf(errstr,"size given (%d) doesn't match that validated (%d).",size,m->size);
	  Logerr(errstr);
	}

	// remove from list 
#ifdef _MEMORYTRACEDEBUG_
	if (m->name)
	  strncpy(name, m->name, 127);
#endif
	mlist.remove(m);
	if (m->name != 0) free(m->name);
	free(m);
      }
      else {
	if ((addr == m->address+Filling) && (!m->isExternal())) {
	  sprintf(errstr,"memory at address %x was allocated with allocate(), not validated with validate().",(Int)address);
	  Logerr(errstr);
	}
      }
      
    }
    else { // !found
      sprintf(errstr,"no memory validated at address %x.",(Int)address);
      Logerr(errstr);
      exitunlock();
      handleError();
      return found;
    }

    exitunlock();

#ifdef _MEMORYTRACEDEBUG_
    char logstr[80];
    sprintf(logstr, "invalidated(address:%x, size:%6d, name:%s)", Int(address), size, name);
    Logerr(logstr);
#endif

    return found;
  }
  
  
  // Check that addr & addr+typesize-1 are within an 
  //    allocated memory area
  void* MemoryTracer::checkAddrRange(void* address, long typesize) throw(std::bad_alloc)
  {
    checkAddr(address, address, true);
    if (typesize>0) checkAddr( ((char*)address)+typesize-1, address, false);
    return address;
  }


  // Check that addr is within an allocated memory area
  bool MemoryTracer::checkAddr(void* address, void* reportAddr, bool checkAllMemory) throw(std::bad_alloc)
  {
    bool found=false;
    Byte *addr = (Byte*)address;
    char errstr[256];
    AllocEntry* m = mlist.front();
    
    if (checkAllMemory) MemoryTracer::checkAllMemory();

    if (reportAddr == 0) reportAddr = address;
    
    while (m && !found) {
      if (!m->isExternal()) {
	
	if ((m->address <= addr) && (addr < m->address+Filling*2+m->size)) { // it's within greater area 
	  
	  if ((m->address+Filling <= addr) && (addr < m->address+Filling+m->size)) // within valid area? 
	    found=true;
	  else { // must be in one of filling areas 
	    if ((m->address <= addr) && (addr < m->address+Filling)) {
	      if (address == reportAddr) {
		sprintf(errstr,"reference address %x is before memory area at %x (size %d) (%s).",
			Int(addr), Int(m->address+Filling), m->size, m->name);
		Logerr(errstr);
	      }
	      else {
		sprintf(errstr,"reference to address %x may allow to access address %x which is before memory area at %x (size %d) (%s).",
			Int(reportAddr), Int(addr), Int(m->address+Filling), m->size, m->name);
		Logerr(errstr);
	      }
	    }
	    else { // must be after then 
	      if (address == reportAddr) {
		sprintf(errstr,"reference address %x is after memory area at %x (size %d) (%s).",
			Int(addr), Int(m->address+Filling), m->size, m->name);
		Logerr(errstr);
	      }
	      else {
		sprintf(errstr,"reference to address %x may allow to access address %x which is after memory area at %x (size %d) (%s)\n",
			Int(reportAddr), Int(addr), Int(m->address+Filling), m->size, m->name);
		Logerr(errstr);
	      }
	    }
	    
	  }
	}
      } else { // external 
	if ((m->address <= addr) && (addr < m->address+m->size))
	  found = true;
      }
      
      m = m->next;
    }
    if (!found) {
      if (address == reportAddr) {
	sprintf(errstr,"reference to invalid memory address %x detected.",Int(addr));
	Logerr(errstr);
      }
      else {
	sprintf(errstr,"detected reference to address %x that may allow access to invalid address %x.",Int(reportAddr), Int(addr));
	Logerr(errstr);
      }
      handleError();
      return false;
  }
    
    return true;
  }
  

  char* MemoryTracer::getName(void* addr) 
  {
    AllocEntry* m = mlist.front();
    bool found=false;
    AllocEntry* entry = 0;
    while (m && !found) {
      if (!m->isExternal()) {
	
	if ((m->address <= addr) && (addr < m->address+Filling*2+m->size)) { // it's within greater area 
	  
	  if ((m->address+Filling <= addr) && (addr < m->address+Filling+m->size)) { // within valid area? 
	    found=true;
	    entry = m;
	  }
	}
      } else { // external 
	if ((m->address <= addr) && (addr < m->address+m->size)) {
	  found = true;
	  entry=m;
	}
      }
      
      m = m->next;
    }
    
    if (found)
      return entry->name;
    else
      return 0;
  }


  long MemoryTracer::getOffset(void* addr) 
  {
    AllocEntry* m = mlist.front();
    bool found=false;

    long offset=-1;
    while (m && !found) {
      if (!m->isExternal()) {
	
	if ((m->address <= addr) && (addr < m->address+Filling*2+m->size)) { // it's within greater area 
	  
	  if ((m->address+Filling <= addr) && (addr < m->address+Filling+m->size)) { // within valid area? 
	    found=true;
	    offset = (Byte*)addr - (m->address+Filling);
	  }
	}
      } else { // external 
	if ((m->address <= addr) && (addr < m->address+m->size)) {
	  found = true;
	  offset = (Byte*)addr - m->address;
	}
      }
      
      m = m->next;
    }
    
    return offset;
  }


  // free all allocated memory
  bool MemoryTracer::cleanup()
  {
#ifdef _TRACEMEMORY_
    printf("(Cleanup: %ld allocations, %ld deallocations, %ld integrity checks performed)\n",allocCount,deallocCount,checkCount);

    char errstr[256];
    if (mlist.size() > 0) {
      dump();
      sprintf(errstr,"Freeing %d leaked memory allocations",mlist.size());
      Logerr(errstr);
      while (mlist.size() > 0) {
	AllocEntry* m = mlist.pop_front();
	if (m->name != 0) free(m->name);
	if (!m->isExternal()) free(m->address);
	free(m);
      }
    }

    while (dlist.size() > 0) {
      AllocEntry* m = dlist.pop_front();
      if (m->name != 0) free(m->name);
      free(m);
    }
#endif    
    cleanedUp = true;
    return true;
  }

  // display allocated memory areas
  void MemoryTracer::dump() 
  {
#ifdef _TRACEMEMORY_
    AllocEntry* m = mlist.front();
    
    checkAllMemory();
    
    printf("\nValid memory regions (%d):\n",mlist.size());
    printf("(%ld allocations, %ld deallocations, %ld integrity checks performed)\n",allocCount,deallocCount,checkCount);
    printf("----------------------------------------------------------------------------------\n");
    
    while(m) {
      if (m->isOK()) {
	printf("%52s: %10x (%9d) %s %s\n", (m->name==0)?"<unnamed>":m->name, Int(m->address+Filling), 
	       m->size,(m->isArray()?"[]":"  "),m->isExternal()?"EXT":"");
	m = m->next;
      }
      else {
	Logerr("Memory allocation structure corrupted!");
	m=0;
      }
    }
    printf("----------------------------------------------------------------------------------\n\n");
#endif
  }


  // display allocated memory areas reciently deallocated
  void MemoryTracer::dumpDeallocated() 
  {
#ifdef _TRACEMEMORY_
    AllocEntry* m = dlist.front();
    
    printf("\nReciently deallocated memory regions (%d):\n",dlist.size());
    printf("(%ld allocations, %ld deallocations, %ld integrity checks performed)\n",allocCount,deallocCount,checkCount);
    printf("----------------------------------------------------------------------------------\n");
    
    while(m) {
      if (m->isOK()) {
	printf("%52s: %10x (%9d) %s %s\n", (m->name==0)?"<unnamed>":m->name, Int(m->address+Filling), 
	       m->size,(m->isArray()?"[]":"  "),m->isExternal()?"EXT":"");
	m = m->next;
      }
      else {
	Logerr("Memory allocation structure corrupted!");
	m=0;
      }
    }
    printf("----------------------------------------------------------------------------------\n\n");
#endif
  }


  // display named allocated memory areas only
  void MemoryTracer::dumpNamed() 
  {
#ifdef _TRACEMEMORY_
    AllocEntry* m = mlist.front();
    
    checkAllMemory();
    
    printf("\nValid named memory regions (from %d total):\n",mlist.size());
    printf("(%ld allocations, %ld deallocations, %ld integrity checks performed)\n",allocCount,deallocCount,checkCount);
    printf("----------------------------------------------------------------------------------\n");
    
    int e=0;
    while(m) {
      if (m->isOK()) {
	if (m->name!=0)
	  printf("%4d %52s: %10x (%9d) %s %s\n",e,m->name, (Int)(m->address+Filling),
		 m->size,(m->isArray()?"[]":"  "),m->isExternal()?"EXT":"");
        ++e; 
	m = m->next;
      }
      else {
	Logerr("Memory allocation structure corrupted!");
	m=0;
      }
    }
    printf("----------------------------------------------------------------------------------\n\n");
#endif    
  }







#include <list>

namespace {

using base::Byte;
using base::Int;

std::list<Byte*> delayedDeletes;
Int count = 0;
Int maxDepth = 0;
static const Int maxCount = 5000;

void stressTestAllocatorRecurse(Int depth)
{
  Int size = (rand() & 0xff)+1;
  Byte* mem = new Byte[size];
  ++count;

  if (depth > maxDepth) 
    maxDepth = depth;

  Int r = (rand() & 0xc0) >> 6; // 0-2
  if (r && (count < maxCount))
    stressTestAllocatorRecurse(depth+1);

  Int d = (rand() & 0x80) >> 7; // 0 or 1
  if (d)
    delete[] mem;
  else
    delayedDeletes.push_back(mem);
}

} // namespace

using std::list;

void base::stressTestAllocator()
{
  Logfln("Performing memory allocator test...");
  while (count < maxCount) {
    stressTestAllocatorRecurse(1);
    MemoryTracer::checkAllMemory();

    // delete half of the delayed delete's
    base::Int c = delayedDeletes.size()/2;
    std::list<Byte*>::iterator s = delayedDeletes.begin();
    std::list<Byte*>::iterator end = delayedDeletes.end();
    while (s != end) {
      if (--c > 0) 
	if (*s) delete[] (*s);
      (*s) = 0;
      ++s;
    }
    MemoryTracer::checkAllMemory();
  }
  
  // delete remaining delayed delete's
  std::list<Byte*>::const_iterator s = delayedDeletes.begin();
  std::list<Byte*>::const_iterator end = delayedDeletes.end();
  while (s != end) {
    if (*s) delete[] (*s);
    ++s;
  }
  delayedDeletes.clear();
  MemoryTracer::checkAllMemory();

  Logfln("Done. " << count << " allocations (" << maxDepth << " recursion depth)")
}





// global


#ifdef _TRACEMEMORY_

void* operator new(size_t size) throw (std::bad_alloc)
{
  void* p = base::MemoryTracer::allocate(size,"");
#ifdef _TRACEOUTPUT_
#ifndef _NAMEDOUTPUTONLY_
  fprintf(stderr, "new(%d) = %x\n",size,Int(p));
#endif
#endif
  return p;
}

void* operator new(size_t size, const string& name) throw (std::bad_alloc)
{
  void* p = base::MemoryTracer::allocate(size,name);
#ifdef _TRACEOUTPUT_
  fprintf(stderr, "new(%d,%s) = %x\n",size,name.c_str(),Int(p));
#endif
  return p;
}

void operator delete(void* p) throw()
{
#ifdef _TRACEOUTPUT_
  char* name = base::MemoryTracer::getName(p);
  if (name)
    fprintf(stderr, "delete[](%x,%s)\n", Int(p),name);
#ifndef _NAMEDOUTPUTONLY_
  else
    fprintf(stderr, "delete[](%x)\n", Int(p));
#endif
#endif
  base::MemoryTracer::release(p);
}


void* operator new[](size_t size) throw (std::bad_alloc)
{
  void* p = base::MemoryTracer::allocate(size,"",true);
#ifdef _TRACEOUTPUT_
#ifndef _NAMEDOUTPUTONLY_
  fprintf(stderr, "new[](%d) = %x\n",size,Int(p));
#endif
#endif
  return p;
}

void* operator new[](size_t size, const string& name) throw (std::bad_alloc)
{
  void* p = base::MemoryTracer::allocate(size,name,true);
#ifdef _TRACEOUTPUT_
  fprintf(stderr, "new[](%d,%s) = %x\n",size,name.c_str(),Int(p));
#endif
  return p;
}

void operator delete[](void* p) throw()
{
#ifdef _TRACEOUTPUT_
  char* name = base::MemoryTracer::getName(p);
  if (name)
    fprintf(stderr, "delete[](%x,%s)\n", Int(p),name);
#ifndef _NAMEDOUTPUTONLY_
  else
    fprintf(stderr, "delete[](%x)\n", Int(p));
#endif
#endif
  base::MemoryTracer::release(p,true);
}

#endif
