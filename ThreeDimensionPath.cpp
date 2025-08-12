#include "Vector.h"

#include <GL/glut.h>
#include <iostream>
#include <vector>
#include <set>
#include <math.h>

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