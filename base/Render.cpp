#include "Render.h"
#include <GL/glut.h>
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
    glPopMatrix();
}

void Render::swapBuffers() {
    glutSwapBuffers();
}

void Render::drawVoxels(const std::vector<Voxels>& cubes) {
  for(size_t i = 0; i < cubes.size(); i++) {
    drawCube(cubes[i].upperRightCorner, cubes[i].bottomLeftCorner, 0.9, 0.0, 0.0);
  }
}

void Render::drawPath(const std::vector<Vector>& path) {
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

void Render::drawStartGoalNodes(const Voxels &start, const Voxels &goal, bool showEdges)
{
  float theta = (2 * M_PI) / 50;

  // Draw start node (green)
  drawSphere(start.center, 1, 0.2, 0.8, 0.1);

  // Draw goal node (blue)
  drawSphere(goal.center, 1, 0.1, 0.1, 0.8);

  // // Draw edges if path not found
  // if(path.empty()) {
  //   for(size_t i = 0; i < start.edges.size(); i++) {
  //     glBegin(GL_LINES);
  //       glColor3f(1.0f, 1.0f, 0.0f);
  //       Vector pointOne = start.center;
  //       Vector pointTwo = start.edges[i].center;
  //       glVertex3f(pointOne[0], pointOne[1], pointOne[2]);
  //       glVertex3f(pointTwo[0], pointTwo[1], pointTwo[2]);
  //       glEnd();
  //   }

  //   for(size_t i = 0; i < goal.edges.size(); i++) {
  //     glBegin(GL_LINES);
  //       glColor3f(1.0f, 1.0f, 0.0f);
  //         Vector pointOne = goal.center;
  //         Vector pointTwo = goal.edges[i].center;
  //         glVertex3f(pointOne[0], pointOne[1], pointOne[2]);
  //         glVertex3f(pointTwo[0], pointTwo[1], pointTwo[2]);
  //         glEnd();
  //     }
  // }

}

void Render::drawEdges(const std::vector<Voxels>& cubes) {
  for(size_t i = 0; i < cubes.size(); i++) {
    for(size_t j = 0; j < cubes[i].edges.size(); j++) {
      glBegin(GL_LINES);
      glColor3f(1.0f, 1.0f, 0.0f);

      Vector pointOne = cubes[i].center;
      Vector pointTwo = cubes[i].edges[j].center;

      glVertex3f(pointOne[0], pointOne[1], pointOne[2]);
      glVertex3f(pointTwo[0], pointTwo[1], pointTwo[2]);

      glEnd();
    }
  }
}

void Render::drawSphere(const Vector& center, float radius, float r, float g, float b) {
  float theta = (2 * M_PI) / 50;

  for (float v = 0.0; v < M_PI; v+= theta) {
    glBegin(GL_TRIANGLE_STRIP);
    for (float u = 0.0; u < 2 * M_PI; u += theta) {
      glColor3f(r, g, b);
      glVertex3f((cos(u) * sin(v) * radius) + center[0], (sin(u) * sin(v) * radius) + center[1], (cos(v) * radius) + center[2]);
      glVertex3f((cos(u) * sin(v + theta) * radius) + center[0], (sin(u) * sin(v + theta) * radius) + center[1], (cos(v + theta) * radius) + center[2]);
    }
  }
  glEnd();
}

void Render::drawCube(const Vector& upperRight, const Vector& bottomLeft, float r, float g, float b) {
  glBegin(GL_LINES);
  glColor3f(r, g, b);

  // Back Face
  glVertex3f(bottomLeft[0], bottomLeft[1], bottomLeft[2]);
  glVertex3f(bottomLeft[0], upperRight[1], bottomLeft[2]);

  glVertex3f(bottomLeft[0], upperRight[1], bottomLeft[2]);
  glVertex3f(upperRight[0], upperRight[1], bottomLeft[2]);

  glVertex3f(upperRight[0], upperRight[1], bottomLeft[2]);
  glVertex3f(upperRight[0], bottomLeft[1], bottomLeft[2]);

  glVertex3f(upperRight[0], bottomLeft[1], bottomLeft[2]);
  glVertex3f(bottomLeft[0], bottomLeft[1], bottomLeft[2]);


  //Left Face
  glVertex3f(bottomLeft[0], bottomLeft[1], upperRight[2]);
  glVertex3f(bottomLeft[0], upperRight[1], upperRight[2]);

  glVertex3f(bottomLeft[0], upperRight[1], upperRight[2]);
  glVertex3f(bottomLeft[0], upperRight[1], bottomLeft[2]);

  glVertex3f(bottomLeft[0], upperRight[1], bottomLeft[2]);
  glVertex3f(bottomLeft[0], bottomLeft[1], bottomLeft[2]);

  glVertex3f(bottomLeft[0], bottomLeft[1], bottomLeft[2]);
  glVertex3f(bottomLeft[0], bottomLeft[1], upperRight[2]);

  // Front Face
  glVertex3f(upperRight[0], bottomLeft[1], upperRight[2]);
  glVertex3f(upperRight[0], upperRight[1], upperRight[2]);

  glVertex3f(upperRight[0], upperRight[1], upperRight[2]);
  glVertex3f(bottomLeft[0], upperRight[1], upperRight[2]);

  glVertex3f(bottomLeft[0], upperRight[1], upperRight[2]);
  glVertex3f(bottomLeft[0], bottomLeft[1], upperRight[2]);

  glVertex3f(bottomLeft[0], bottomLeft[1], upperRight[2]);
  glVertex3f(upperRight[0], bottomLeft[1], upperRight[2]);

  //Right Face
  glVertex3f(upperRight[0], bottomLeft[1], bottomLeft[2]);
  glVertex3f(upperRight[0], upperRight[1], bottomLeft[2]);

  glVertex3f(upperRight[0], upperRight[1], bottomLeft[2]);
  glVertex3f(upperRight[0], upperRight[1], upperRight[2]);

  glVertex3f(upperRight[0], upperRight[1], upperRight[2]);
  glVertex3f(upperRight[0], bottomLeft[1], upperRight[2]);

  glVertex3f(upperRight[0], bottomLeft[1], upperRight[2]);
  glVertex3f(upperRight[0], bottomLeft[1], bottomLeft[2]);

  glEnd();
}