#ifndef _GFX_CUTILS_H_
#define _GFX_CUTILS_H_

#include <X11/X.h>
#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/Xutil.h>
#include <GL/gl.h>

// function to return visual->class, because class is a c++
//  keyword, we can access the field!
int getVisualInfoClass(XVisualInfo* visual); 



// Until nVidia includes these in libGL.so
// (an SGI GLX extension)

int glXBindChannelToWindowSGIX( Display *display,
        int screen,
        int channel,
        Window window );

int glXChannelRectSGIX(  Display *display,
        int screen,
        int channel,
        int x,
        int y,
        int w,
        int h );

int glXQueryChannelRectSGIX( Display *display,
        int screen,
        int channel,
        int *dx,
        int *dy,
        int *dw,
        int *dh );

int glXQueryChannelDeltasSGIX( Display *display,
        int screen,
        int channel,
        int *x,
        int *y,
        int *w,
        int *h );


int glXChannelRectSyncSGIX( Display *display,
        int screen,
        int channel,
        GLenum synctype);




#endif
