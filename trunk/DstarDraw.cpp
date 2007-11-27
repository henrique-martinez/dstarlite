/* DstarDraw.cpp
 * James Neufeld (neufeld@cs.ualberta.ca)
 */

// Header stuff
#ifdef MACOS
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h> 
#else
#include <GL/glut.h>
#include <GL/gl.h> 
#include <GL/glu.h>
#endif

#include <unistd.h>
#include "Dstar.h"

int hh, ww;


int window; 
Dstar *dstar;

int scale = 6;
int mbutton = 0;
int mstate = 0;


bool b_autoreplan = true, b_examine_mode=false;

// end header


void InitGL(int Width, int Height)
{
  hh = Height;
  ww = Width;

  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClearDepth(1.0);	

  glViewport(0,0,Width,Height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0,Width,0,Height,-100,100);
  glMatrixMode(GL_MODELVIEW);

}

void ReSizeGLScene(int Width, int Height)
{
  hh = Height;
  ww = Width;

  glViewport(0,0,Width,Height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0,Width,0,Height,-100,100);
  glMatrixMode(GL_MODELVIEW);

}

void DrawGLScene()
{

  usleep(100);

  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
  glLoadIdentity();
  glPushMatrix();

  glScaled(scale,scale,1);

  if (b_autoreplan) dstar->replan();

  dstar->draw();
  
  glPopMatrix();
  glutSwapBuffers();

}

void examine_cell(int x, int y) {
  
  dstar->queryCell(x,y);

}

void keyPressed(unsigned char key, int x, int y) 
{
  usleep(100);

  switch(key) {
  case 'q':
  case 'Q':
    glutDestroyWindow(window); 
    exit(0);
    break;
  case 'r':
  case 'R':
    dstar->replan();
    break;
  case 'a':
  case 'A':
    b_autoreplan = !b_autoreplan;
    break;
  case 'c':
  case 'C':
    dstar->init(40,50,140, 90);
    break;
  case 'p':
  case 'P': {
    ds_path path = dstar->getPath();
    printf("Path cost = %g\n", path.cost);
    break;
  }
  case 'e':
  case 'E':
    b_examine_mode = !b_examine_mode;
    break;
  }
}

void clickCell(int x, int y, double cost) {

  
  dstar->updateCell(x, y, cost);  
  x++;
  dstar->updateCell(x, y, cost);  
  y++;
  dstar->updateCell(x, y, cost); 
  x--;
  dstar->updateCell(x, y, cost);  
  x--;
  dstar->updateCell(x, y, cost);  
  y--;
  dstar->updateCell(x, y, cost);  
  y--;
  dstar->updateCell(x, y, cost); 
  x++;
  dstar->updateCell(x, y, cost);  
  x++;
  dstar->updateCell(x, y, cost);
  

}
void mouseFunc(int button, int state, int x, int y) {
  
  y = hh -y+scale/2;
  x += scale/2;

  y /= scale;
  x /= scale;
  mbutton = button;

  if ((mstate = state) == GLUT_DOWN) {
    
    if (b_examine_mode) {
      examine_cell(x,y);
    } else if (button == GLUT_LEFT_BUTTON) {
      clickCell(x, y, -1);
    } else if (button == GLUT_RIGHT_BUTTON) {
      dstar->updateStart(x, y);
    } else if (button == GLUT_MIDDLE_BUTTON) {
      dstar->updateGoal(x, y);
    }
  }
}

void mouseMotionFunc(int x, int y)  {

  y = hh -y+scale/2;
  x += scale/2;
  
  y /= scale;
  x /= scale;
  
  if (mstate == GLUT_DOWN) {
    if (b_examine_mode) {
      examine_cell(x,y);
    } else if (mbutton == GLUT_LEFT_BUTTON) {
      clickCell(x, y, -1);
    } else if (mbutton == GLUT_RIGHT_BUTTON) {
      dstar->updateStart(x, y);
    } else if (mbutton == GLUT_MIDDLE_BUTTON) {
      dstar->updateGoal(x, y);
    }
  }

}

int main(int argc, char **argv) {

  glutInit(&argc, argv);  
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);  
  glutInitWindowSize(1000, 800);  
  glutInitWindowPosition(50, 20);  
    
  window = glutCreateWindow("Dstar Visualizer");  
  
  glutDisplayFunc(&DrawGLScene);  
  glutIdleFunc(&DrawGLScene);
  glutReshapeFunc(&ReSizeGLScene);
  glutKeyboardFunc(&keyPressed);
  glutMouseFunc(&mouseFunc);
  glutMotionFunc(&mouseMotionFunc);

  InitGL(800, 600);

  dstar = new Dstar();
  dstar->init(40,50,140, 90);
  
  printf("----------------------------------\n");
  printf("Dstar Visualizer\n");
  printf("Commands:\n");
  printf("[q/Q] - Quit\n");
  printf("[r/R] - Replan\n");
  printf("[a/A] - Toggle Auto Replan\n");
  printf("[e/E] - Toggle Examine Cell Mode\n");
  printf("[c/C] - Clear (restart)\n");
  printf("left mouse click - make cell untraversable (cost -1)\n");
  printf("middle mouse click - move goal to cell\n");
  printf("right mouse click - move start to cell\n");
  printf("----------------------------------\n");

  glutMainLoop();  

  return 1;
}
