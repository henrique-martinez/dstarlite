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
                GLfloat x, GLfloat y);
#endif
