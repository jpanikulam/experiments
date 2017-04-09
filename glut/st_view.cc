#include <iostream>

#include <GL/freeglut.h>

struct State {
  // float2 prev_mouse_pos = make_float2(0.0, 0.0);

  // Image render dimensions
  uint width = 512;
  uint height = 512;

  float scale = 1.0;
  float theta = 0.0;

} gstate;

void initGL(int *argc, char **argv) {
  // initialize GLUT callback functions
  glutInit(argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
  glutInitWindowSize(gstate.width, gstate.height);
  glutCreateWindow("GL ST viewer");

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0, (float)gstate.width / (float)gstate.height, 0.1, 100.0);
}

// display results using OpenGL (called by GLUT)
void display() {
  // Display results
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // use OpenGL to build view matrix
  GLfloat modelView[16];
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  std::cout << gstate.theta << std::endl;
  glRotatef(gstate.theta, 0.0, 1.0, 0.0);

  glGetFloatv(GL_MODELVIEW_MATRIX, modelView);

  // for (int k = 0; k < 16; ++k)
  // {
  //   std::cout << modelView[k] << ", ";
  // }
  // std::cout << std::endl;

  // glPushMatrix();
  // glPopMatrix();

  //
  // Draw the texture on the simplest quad
  //
  // Draw textured quad
  // glEnable(GL_TEXTURE_2D);

  glBegin(GL_QUADS);

  // This is just a quad that occupies the whole screen

  /*
  glTexCoord2f(0, 0);
  glVertex2f(-1, -1);

  glTexCoord2f(1, 0);
  glVertex2f(1, -1);

  glTexCoord2f(1, 1);
  glVertex2f(1, 1);

  glTexCoord2f(0, 1);
  glVertex2f(-1, 1);
  */

  const float scale = 0.2f;
  const float first_depth = 0.3f;
  const float second_depth = 0.5f;

  glTexCoord2f(0, 0);
  glVertex3f(-scale, -scale, first_depth);

  glTexCoord2f(scale, 0);
  glVertex3f(scale, -scale, first_depth);

  glTexCoord2f(scale, scale);
  glVertex3f(scale, scale, -second_depth);

  glTexCoord2f(0, scale);
  glVertex3f(-scale, scale, -second_depth);

  glEnd();

  glDisable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, 0);

  //
  // And it's over
  //

  glutSwapBuffers();
  glutReportErrors();
  gstate.theta += 0.1;
  if (gstate.theta >= 90) {
    gstate.theta = 0.0;
  }
}

void keyboard(unsigned char key, int x, int y) {
  switch (key) {
    case 27:
      glutDestroyWindow(glutGetWindow());
      return;

    case 'f':
      break;

    case 'q':
      std::cout << "Attempting to exit" << std::endl;
      glutDestroyWindow(glutGetWindow());
      return;

    default:
      break;
  }

  glutPostRedisplay();
}

void idle() { glutPostRedisplay(); }

int main(int argc, char **argv) {
#if defined(__linux__)
  setenv("DISPLAY", ":0", 0);
#endif
  std::cout << "stviewer starting\n" << std::endl;

  //
  // Initialize openGL
  //
  initGL(&argc, argv);

  //
  // Initialize glut
  //
  // glutMouseFunc(mouse);
  // glutMotionFunc(motion);

  glutDisplayFunc(display);
  glutKeyboardFunc(keyboard);
  glutIdleFunc(idle);

  glutMainLoop();
}