#ifndef DUTILS_H
#define DUTILS_H

#ifdef MACOS
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h> 
#else
#include <GL/glut.h>
#include <GL/gl.h> 
#include <GL/glu.h>
#endif

void DisplayStr(char* str,
		void *font, GLclampf r, GLclampf g, GLclampf b, 
		GLfloat x, GLfloat y)
{
  /* font: font to use, e.g., GLUT_BITMAP_HELVETICA_10
     r, g, b: text colour
     x, y: text position in window: range [0,0] (bottom left of window)
     to [1,1] (top right of window). */
  
  char *ch; 
  GLint matrixMode;
  GLboolean lightingOn;
  
  lightingOn= glIsEnabled(GL_LIGHTING);        /* lighting on? */
  if (lightingOn) glDisable(GL_LIGHTING);

  glGetIntegerv(GL_MATRIX_MODE, &matrixMode);  /* matrix mode? */

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0.0, 1.0, 0.0, 1.0);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glPushAttrib(GL_COLOR_BUFFER_BIT);       /* save current colour */
  glColor3f(r, g, b);
  glRasterPos3f(x, y, 0.0);
  for(ch= str; *ch; ch++) {
    glutBitmapCharacter(font, (int)*ch);
  }
  glPopAttrib();
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(matrixMode);
  if (lightingOn) glEnable(GL_LIGHTING);
}

#endif
