#include "Scene.h"

#include <GL/glut.h>
#include <iostream>

Scene::Scene() {
    baseCube.upperRightCorner = Vector(15, 15, 15);
    baseCube.bottomLeftCorner = Vector(-15, -15, -15);
    baseCube.center = Vector(0, 0, 0);
}

/** Buids the base cube for the structure that we initially see in the window*/
void Scene::buildVolume() {
    glBegin( GL_LINES);
    glColor3f(1.0, 1.0, 1.0);

    //Back Face
    glVertex3f(-15.0f, -15.0f, -15.0f);
    glVertex3f(-15.0f, 15.0f, -15.0f);

    glVertex3f(-15.0f, 15.0f, -15.0f);
    glVertex3f(15.0f, 15.0f, -15.0f);

    glVertex3f(15.0f, 15.0f, -15.0f);
    glVertex3f(15.0f, -15.0f, -15.0f);

    glVertex3f(15.0f, -15.0f, -15.0f);
    glVertex3f(-15.0f, -15.0f, -15.0f);

    //Left Face
    glVertex3f(-15.0f, -15.0f, 15.0f);
    glVertex3f(-15.0f, 15.0f, 15.0f);

    glVertex3f(-15.0f, 15.0f, 15.0f);
    glVertex3f(-15.0f, 15.0f, -15.0f);

    glVertex3f(-15.0f, 15.0f, -15.0f);
    glVertex3f(-15.0f, -15.0f, -15.0f);

    glVertex3f(-15.0f, -15.0f, -15.0f);
    glVertex3f(-15.0f, -15.0f, 15.0f);

    //Front Face
    glVertex3f(15.0f, -15.0f, 15.0f);
    glVertex3f(15.0f, 15.0f, 15.0f);

    glVertex3f(15.0f, 15.0f, 15.0f);
    glVertex3f(-15.0f, 15.0f, 15.0f);

    glVertex3f(-15.0f, 15.0f, 15.0f);
    glVertex3f(-15.0f, -15.0f, 15.0f);

    glVertex3f(-15.0f, -15.0f, 15.0f);
    glVertex3f(15.0f, -15.0f, 15.0f);

    //Right Face
    glVertex3f(15.0f, -15.0f, -15.0f);
    glVertex3f(15.0f, 15.0f, -15.0f);

    glVertex3f(15.0f, 15.0f, -15.0f);
    glVertex3f(15.0f, 15.0f, 15.0f);

    glVertex3f(15.0f, 15.0f, 15.0f);
    glVertex3f(15.0f, -15.0f, 15.0f);

    glVertex3f(15.0f, -15.0f, 15.0f);
    glVertex3f(15.0f, -15.0f, -15.0f);

    glEnd();

    // Indicates the upper and lower corner, as well as the center of the voxel
    baseCube.upperRightCorner = Vector(15, 15, 15);
    baseCube.bottomLeftCorner = Vector(-15, -15, -15);
    baseCube.center = Vector(0, 0, 0);
}

/** Builds the obstacles that the user defines for the area */
void Scene::buildObstacle() {
  std::cout << "Obstacles reached" << std::endl;
  for(size_t i = 0; i < spheres.size(); i++) {

    // Uses a float variable named theta to calculate the different vertices needed to be drawn
    float theta = (2 * M_PI) / 50;

    for (float v = 0.0; v < M_PI; v+= theta) {
      glBegin(GL_TRIANGLE_STRIP);
      for (float u = 0.0; u < 2 * M_PI; u+= theta) {
        glColor3f(0.62, 0.05, 0.05);
        glVertex3f((cos(u) * sin(v) * spheres[i].radius) + spheres[i].position[0],(sin(u) * sin(v) * spheres[i].radius) + spheres[i].position[1], (cos(v) * spheres[i].radius) + spheres[i].position[2]);
        glVertex3f((cos(u) * sin(v + theta) * spheres[i].radius) + spheres[i].position[0], (sin(u) * sin(v + theta) * spheres[i].radius) + spheres[i].position[1], (cos(v + theta) * spheres[i].radius) + spheres[i].position[2]);
      }
    }
    glEnd();

  }
}