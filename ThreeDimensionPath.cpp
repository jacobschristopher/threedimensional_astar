#include "Vector.h"

#include <GL/glut.h>
#include <iostream>
#include <vector>
#include <set>
#include <math.h>

void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);

    glLoadIdentity();

    gluPerspective(60.0, 4.0/3.0, 0.01, 100.0);

    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();

    gluLookAt(40, 0, 40, 0, 0, 0, 0, 1, 0);

    glPushMatrix();
    glRotatef(0.0, 1.0, 0.0, 0.0);
    glRotatef(0.0, 0.0, 1.0, 0.0);
    glRotatef(0.0, 0.0, 0.0, 1.0);

    glPopMatrix();

    glutSwapBuffers();
}

void handleKey(unsigned char key, int x, int y)
{

}

void specialInput(int key, int x, int y) {
    
}

int main(int argc, char* argv[]) 
{
    // Start up GLUT
    glutInit(&argc, argv);

    // Create the graphics window, giving width, height and title text
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowPosition(80,80);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Three Dimensional Pathfinding using A*");

    // Specify clear background color
    glEnable(GL_DEPTH_TEST);

    // Set up callback routines
    glutDisplayFunc(display);
    glutKeyboardFunc(handleKey);
    glutSpecialFunc(specialInput);

    glClearColor(0,0,0,0);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    std::cout << "Starting program..." << '\n';
    std::cout << "Welcome." << '\n';

    glutMainLoop();
    return 0;
}