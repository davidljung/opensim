#include <gfx/cutils.h>

#include <stdio.h>
#include <GL/gl.h>


int getVisualInfoClass(XVisualInfo* visual) 
{ 
  return visual->class; 
}



// Until nVidia includes these in libGL.so

int glXBindChannelToWindowSGIX( Display *display,
                     int screen,
                     int channel,
                     Window window )
{
  return 0;
}


int glXChannelRectSGIX(  Display *display,
                     int screen,
                     int channel,
                     int x,
                     int y,
                     int w,
                     int h )
{
  return 0;
}


int glXQueryChannelRectSGIX( Display *display,
                     int screen,
                     int channel,
                     int *dx,
                     int *dy,
                     int *dw,
                     int *dh )
{
  return 0;
}


int glXQueryChannelDeltasSGIX( Display *display,
                     int screen,
                     int channel,
                     int *x,
                     int *y,
                     int *w,
                     int *h )
{
  return 0;
}


int glXChannelRectSyncSGIX( Display *display,
                     int screen,
                     int channel,
                     GLenum synctype)
{
  return 0;
}





