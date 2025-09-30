#include "Render.h"
#include <GL.glut.h>
#include <cmath>

Render::Render() {}

void Render::setupCamera(double rotate_x, double rotate_y, double rotate_z) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, 4.0/3.0, 0.01, 100.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(40, 0, 40, 0, 0, 0, 0, 1, 0);

    glPushMatrix();
    glRotatef(rotate_x, 1.0, 0.0, 0.0);
    glRotatef(rotate_y, 0.0, 1.0, 0.0);
    glRotatef(rotate_z, 0.0, 0.0, 1.0);
}

void Render::clearScreen() {
    glPopMatrix()
}

void Render::swapBuffers() {
    glutSwapBuffers();
}

void Render::drawVoxels(const std::vector<Voxels>& cubes) {
  for(size_t i = 0; i < cubes.size(); i++) {
    Vector URC = cubes[i].upperRightCorner;
    Vector BLC = cubes[i].bottomLeftCorner;

    glBegin(GL_LINES);
    glColor3f(0.9f, 0.0f, 0.0);

    // Back Face
    glVertex3f(BLC[0], BLC[1], BLC[2]);
    glVertex3f(BLC[0], URC[1], BLC[2]);

    glVertex3f(BLC[0], URC[1], BLC[2]);
    glVertex3f(URC[0], URC[1], BLC[2]);

    glVertex3f(URC[0], URC[1], BLC[2]);
    glVertex3f(URC[0], BLC[1], BLC[2]);

    glVertex3f(URC[0], BLC[1], BLC[2]);
    glVertex3f(BLC[0], BLC[1], BLC[2]);


    //Left Face
    glVertex3f(BLC[0], BLC[1], URC[2]);
    glVertex3f(BLC[0], URC[1], URC[2]);

    glVertex3f(BLC[0], URC[1], URC[2]);
    glVertex3f(BLC[0], URC[1], BLC[2]);

    glVertex3f(BLC[0], URC[1], BLC[2]);
    glVertex3f(BLC[0], BLC[1], BLC[2]);

    glVertex3f(BLC[0], BLC[1], BLC[2]);
    glVertex3f(BLC[0], BLC[1], URC[2]);

    // Front Face
    glVertex3f(URC[0], BLC[1], URC[2]);
    glVertex3f(URC[0], URC[1], URC[2]);

    glVertex3f(URC[0], URC[1], URC[2]);
    glVertex3f(BLC[0], URC[1], URC[2]);

    glVertex3f(BLC[0], URC[1], URC[2]);
    glVertex3f(BLC[0], BLC[1], URC[2]);

    glVertex3f(BLC[0], BLC[1], URC[2]);
    glVertex3f(URC[0], BLC[1], URC[2]);

    //Right Face
    glVertex3f(URC[0], BLC[1], BLC[2]);
    glVertex3f(URC[0], URC[1], BLC[2]);

    glVertex3f(URC[0], URC[1], BLC[2]);
    glVertex3f(URC[0], URC[1], URC[2]);

    glVertex3f(URC[0], URC[1], URC[2]);
    glVertex3f(URC[0], BLC[1], URC[2]);

    glVertex3f(URC[0], BLC[1], URC[2]);
    glVertex3f(URC[0], BLC[1], BLC[2]);

    glEnd();
  }
}

Render::drawPath(const std::vector<Vector>& path) {
  for(size_t i = 0; i < path.size() - 1; i++) {
    // Starts drawing line method and sets color
    glBegin(GL_LINES);
    glColor3f(0.0f, 1.0f, 0.0f);

    // Draws the path between correctly placed nodes
    glVertex3f(path[i][0], path[i][1], path[i][2]);
    glVertex3f(path[i + 1][0], path[i + 1][1], path[i + 1][2]);

    // Ends drawn section
    glEnd();
  }
}

Render::draw