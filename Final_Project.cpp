//  Christopher Jacobs
//  Three Dimensional A* Pathfinding
//
//
/***************************************************************************/
/* Include needed files */
#include "Final_Project.h"
#include <GL/glut.h>
#include <iostream>
#include <vector>
#include <set>
#include <math.h>
/***************************************************************************/
// Forward declarations
typedef std::pair<float, Vector> PathNode;

// Holds the values of the window dimensions
int WinWidth = 800;
int WinHeight = 600;

// A structure that creates Voxel objects
struct Voxels{
  Vector upperRightCorner;
  Vector bottomLeftCorner;
  Vector center;
  std::vector<Voxels> edges;
  PathNode parent;
};

// A structure that creates Obstacle objects
struct Obstacles{
  Vector position;
  float radius;
};

// Global vectors that hold the values of voxels, obstacles, and the A* path
std::vector<Voxels> cubes;
std::vector<Obstacles> spheres;
std::vector<Vector> path;

// Defines a pair object for float values for distance and Voxels for open and closed lists
typedef std::pair<float, Voxels> Pair;

// Indicates the base cube of the structure as well as the depth of the voxel search
Voxels baseCube;
int depth = 0;

// Creates global values for the start and ending goal
Voxels start;
Voxels goal;

// Holds the values of the rotation matrix
double rotate_x = 0;
double rotate_y = 0;
double rotate_z = 0;

// Holds the boolean value for toggling Voxel graphics
bool voxelToggle = true;

/** Buids the base cube for the structure that we initially see in the window*/
void buildVolume() {
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
void buildObstacle() {
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

/** A method that detects if there is a collision while interploating from one point to another */
bool collisionInt(Vector &point) {
  for(size_t i = 0; i < spheres.size(); i++) {
    float mag = (spheres[i].position - point).magnitude();
    if(mag < spheres[i].radius) {
      return true;
    }
  }
  return false;
}


/** A method that will interpolate from one point to the next to determine */
bool interpolate(Voxels &q1, Voxels &q2, float stepsize) {
  Vector step = (q2.center - q1.center) / stepsize;

  Vector current = q1.center;

  for(int i = 0; i < stepsize; i++) {
    current += step;
    bool decision = collisionInt(current);
    if(decision == true) {
      return true;
    }
  }
  return false;
}

/** A method that will calculate the distance between two points */
float distance(Voxels &q1, Voxels &q2) {
  float dist = (q1.center - q2.center).magnitude();
  return dist;
}

/** A method that will check for any duplicate edges and remove them */
bool duplicate(Voxels &q1) {
  for (size_t i = 0; i < q1.edges.size(); i++) {
    if(q1.center == q1.edges[i].center) {
      return true;
    }
  }
  return false;
}

/** A method that will check for the nearest nearest neighbors around a point and draw edges between them */
void nearestNeighbors(Voxels &q1) {

  // For each cube, calculate the distance from the center to the node in question
  for(size_t i = 0; i < cubes.size(); i++) {
    float len = distance(q1, cubes[i]);

    // If this distance is not greater than twenty units (and not analyzing itself), interpolate to find collisions
    if(len < 20.0 && len != 0.0) {
      bool condition = interpolate(q1, cubes[i], 50.0);

      // If a collision is not found when drawing the path, the path is now plausible
      if(condition == false && duplicate(cubes[i]) == false) {
          q1.edges.push_back(cubes[i]);
      }
    }
  }
}

/** A method that builds a roadmap with the voxels created */
void buildRoadmap() {

  // For each cube, find the nearest neighbors
  for(size_t i = 0; i < cubes.size(); i++) {
    nearestNeighbors(cubes[i]);
  }
}

/** A method that will clamp values when it comes to finding the nearest point to collisions */
float clamp(float value, float minimum, float maximum) {
  return(std::max(minimum, std::min(maximum, value)));
}

/** A method that checks to see if there is a collision active or not */
bool collision(Vector URC, Vector BLC, Vector centerPosition) {
  for(size_t i = 0; i < spheres.size(); i++) {
    Vector difference = (spheres[i].position - centerPosition);

    float halfWidth = abs(URC[0] - centerPosition[0]);
    float halfHeight = abs(URC[1] - centerPosition[1]);
    float halfDepth = abs(URC[2] - centerPosition[2]);


    float clampX = clamp(difference[0], -halfWidth, halfWidth);
    float clampY = clamp(difference[1], -halfHeight, halfHeight);
    float clampZ = clamp(difference[2], -halfDepth, halfDepth);

    Vector closest = Vector(centerPosition[0] + clampX, centerPosition[1] + clampY, centerPosition[2] + clampZ);

    Vector newDifference = (closest - spheres[i].position);

    float length = sqrt(pow(newDifference[0], 2) + pow(newDifference[1], 2) + pow(newDifference[2], 2));

    if(length < spheres[i].radius) {
      return true;
    }
  }
  return false;
}

/** A method that will draw voxels using the previous upper right corner and bottom left corner */
void drawVoxels() {

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

/** A method that will draw the edges on the window */
void drawEdges() {
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

/** A recursive method that calculates the next voxel octree in the set using depth */
int voxelOctrees(Vector URC, Vector BLC, Vector centerPosition) {

  // Creates the new voxel cube primitives along with children identifiers
  Voxels cubeOne, cubeTwo, cubeThree, cubeFour, cubeFive, cubeSix, cubeSeven, cubeEight;
  std::vector<Voxels> children;

  // Uses the previous cubes positions to construct new coordinates for all eight cubes

  // Top Left Front Quadrant
  cubeOne.upperRightCorner = Vector(centerPosition[0], URC[1], URC[2]);
  cubeOne.bottomLeftCorner = Vector(BLC[0], centerPosition[1], centerPosition[2]);
  cubeOne.center = (cubeOne.upperRightCorner + cubeOne.bottomLeftCorner) / 2;

  // Top Right Front Quadrant
  cubeTwo.upperRightCorner = Vector(centerPosition[0], centerPosition[1], centerPosition[2]);
  cubeTwo.bottomLeftCorner = Vector(BLC[0], BLC[1], URC[2]);
  cubeTwo.center = (cubeTwo.upperRightCorner + cubeTwo.bottomLeftCorner) / 2;

  // Top Left Back Quadrant
  cubeThree.upperRightCorner = Vector(URC[0], URC[1], centerPosition[2]);
  cubeThree.bottomLeftCorner = Vector(centerPosition[0], centerPosition[1], URC[2]);
  cubeThree.center = (cubeThree.upperRightCorner + cubeThree.bottomLeftCorner) / 2;

  cubeFour.upperRightCorner = Vector(URC[0], centerPosition[1], centerPosition[2]);
  cubeFour.bottomLeftCorner = Vector(centerPosition[0], BLC[1], URC[2]);
  cubeFour.center = (cubeFour.upperRightCorner + cubeFour.bottomLeftCorner) / 2;

  cubeFive.upperRightCorner = Vector(centerPosition[0], URC[1], BLC[2]);
  cubeFive.bottomLeftCorner = Vector(BLC[0], centerPosition[1], centerPosition[2]);
  cubeFive.center = (cubeFive.upperRightCorner + cubeFive.bottomLeftCorner) / 2;

  cubeSix.upperRightCorner = Vector(centerPosition[0], centerPosition[1], centerPosition[2]);
  cubeSix.bottomLeftCorner = Vector(BLC[0], BLC[1], BLC[2]);
  cubeSix.center = (cubeSix.upperRightCorner + cubeSix.bottomLeftCorner) / 2;

  cubeSeven.upperRightCorner = Vector(URC[0], URC[1], centerPosition[2]);
  cubeSeven.bottomLeftCorner = Vector(centerPosition[0], centerPosition[1], BLC[2]);
  cubeSeven.center = (cubeSeven.upperRightCorner + cubeSeven.bottomLeftCorner) / 2;

  cubeEight.upperRightCorner = Vector(URC[0], centerPosition[1], centerPosition[2]);
  cubeEight.bottomLeftCorner = Vector(centerPosition[0], BLC[1], BLC[2]);
  cubeEight.center = (cubeEight.upperRightCorner + cubeEight.bottomLeftCorner) / 2;

  // Push voxels into vector
  children.push_back(cubeOne);
  children.push_back(cubeTwo);
  children.push_back(cubeThree);
  children.push_back(cubeFour);
  children.push_back(cubeFive);
  children.push_back(cubeSix);
  children.push_back(cubeSeven);
  children.push_back(cubeEight);


  depth++;

  // If the depth is still traversable, search for collisions and keep going more definitive
  for(size_t i = 0; i < children.size(); i++) {
      if(collision(children[i].upperRightCorner, children[i].bottomLeftCorner, children[i].center) == true) {
        if(depth < 3 && depth > 0) {
          cubes.push_back(children[i]);
          depth = voxelOctrees(children[i].upperRightCorner, children[i].bottomLeftCorner, children[i].center);
        }
        else if(depth >= 3) {
          return depth - 1;
        }
      }
  }
  return depth - 1;
}

/** A method that will draw the starting and ending nodes */
void drawNodes() {
    // Uses a float variable named theta to calculate the different vertices needed to be drawn
    float theta = (2 * M_PI) / 50;

    for (float v = 0.0; v < M_PI; v+= theta) {
      glBegin(GL_TRIANGLE_STRIP);
      for (float u = 0.0; u < 2 * M_PI; u+= theta) {
        glColor3f(0.2, 0.8, 0.1);
        glVertex3f((cos(u) * sin(v) * 1) + start.center[0],(sin(u) * sin(v) * 1) + start.center[1], (cos(v) * 1) + start.center[2]);
        glVertex3f((cos(u) * sin(v + theta) * 1) + start.center[0], (sin(u) * sin(v + theta) * 1) + start.center[1], (cos(v + theta) * 1) + start.center[2]);
      }
    }
    glEnd();

    for (float v = 0.0; v < M_PI; v+= theta) {
      glBegin(GL_TRIANGLE_STRIP);
      for (float u = 0.0; u < 2 * M_PI; u+= theta) {
        glColor3f(0.1, 0.1, 0.8);
        glVertex3f((cos(u) * sin(v) * 1) + goal.center[0],(sin(u) * sin(v) * 1) + goal.center[1], (cos(v) * 1) + goal.center[2]);
        glVertex3f((cos(u) * sin(v + theta) * 1) + goal.center[0], (sin(u) * sin(v + theta) * 1) + goal.center[1], (cos(v + theta) * 1) + goal.center[2]);
      }
    }
    glEnd();

    // Draws all edges if the path is not known
    if(path.empty()) {
      for(size_t i = 0; i < start.edges.size(); i++) {
        glBegin(GL_LINES);
        glColor3f(1.0f, 1.0f, 0.0f);

        Vector pointOne = start.center;
        Vector pointTwo = start.edges[i].center;

        glVertex3f(pointOne[0], pointOne[1], pointOne[2]);
        glVertex3f(pointTwo[0], pointTwo[1], pointTwo[2]);

        glEnd();
      }

      for(size_t i = 0; i < goal.edges.size(); i++) {
        glBegin(GL_LINES);
        glColor3f(1.0f, 1.0f, 0.0f);

        Vector pointOne = goal.center;
        Vector pointTwo = goal.edges[i].center;

        glVertex3f(pointOne[0], pointOne[1], pointOne[2]);
        glVertex3f(pointTwo[0], pointTwo[1], pointTwo[2]);

        glEnd();
      }
    }
}

/** A method that finds the minimum distance node */
size_t minimumNode(std::vector<Pair> &list) {
  Pair minimum;
  size_t index;
  minimum.first = 99.0;

  for(size_t i = 0; i < list.size(); i++) {
    if(list[i].first < minimum.first) {
      minimum = list[i];
      index = i;
    }
  }
  return index;
}

/** A method that calculates the heuristic value for A* */
float calculateHeuristic(Voxels &q1) {
  return (start.center - q1.center).magnitude();
}

/** A method that searchs through the to find a pair */
bool searchList(Pair &node, std::vector<Pair> &list) {
  for(size_t i = 0; i < list.size(); i++) {
    if(list[i].second.center == node.second.center) {
      return true;
    }
  }
  return false;
}

/** A method that implements A* by finding the path and also traversing the path */
void findPath() {

  // Calculate the nearest neighbors for our start and end goals
  nearestNeighbors(start);
  nearestNeighbors(goal);

  // Create two vectors of Pairs to house the open list and the closed list
  std::vector<Pair> openList;
  std::vector<Pair> closedList

  // Create a new Pair object to hold the 'f' value and node data
  Pair end = std::make_pair(0.0, goal);
  openList.push_back(end);

  // While the openlist is not empty and the start is not found, keep searching using A*
  while(!openList.empty()) {

    //Find the node in the open list that has the lowest 'f' value and select that node next
    size_t offset = minimumNode(openList);
    Pair node = openList[offset];

    // Erase this node so it will no longer be used
    openList.erase(openList.begin() + offset);


    if(node.second.center == start.center) {
      break;
    }

    closedList.push_back(node);


    for(size_t i = 0; i < node.second.edges.size(); i++) {

      Voxels nextNode = node.second.edges[i];
      Vector nextLoc = nextNode.center;
      float g = (nextLoc - node.second.center).magnitude();
      float h = calculateHeuristic(nextNode);
      float f = g + h;
      Pair newNode = std::make_pair(f, nextNode);
      if(searchList(newNode, openList) == false && searchList(newNode, closedList) == false) {
        openList.push_back(newNode);

        for(size_t i =0; i < cubes.size(); i++) {
          if(newNode.second.center == cubes[i].center) {
            if(cubes[i].parent.first == 0 || cubes[i].parent.first > newNode.first) {
              cubes[i].parent = std::make_pair(f, node.second.center);
            }
          }
        }
      }
    }
  }

  Vector traverse = start.center;
  path.push_back(traverse);

  // Draws the A* path
  while(traverse != goal.center) {
    for(size_t i = 0; i < cubes.size(); i++) {
      if(traverse == cubes[i].center) {
        traverse = cubes[i].parent.second;
        if(traverse == Vector(0,0,0)) {
          std::cout << "Path could not be found" << '\n';
          std::cout << "Please re-run program and try again" << '\n';
          std::cout << "Exiting program..." << '\n';
          exit(1);
        }
        path.push_back(traverse);
        break;
      }
    }
  }
}

/** A method that draws the path in the window */
void drawPath() {

  // Looping through iterations of the path
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

//
//   Display Callback Routine:
//
void display(){
  // Clears the buffer and sets mode to Projection
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_PROJECTION);

  // Resets to Identity Matrix
  glLoadIdentity();

  // Sets up perspective view parameters
  gluPerspective(60.0, 4.0/3.0, 0.01, 100.0);

  // Switches to Model view
  glMatrixMode(GL_MODELVIEW);

  // Resets to Identity Matrix
  glLoadIdentity();

  // Camera parameters to always look at origin
  gluLookAt(40, 0, 40, 0, 0, 0, 0, 1, 0);

  // Pushes new matrix so that we are able to rotate view
  glPushMatrix();
  glRotatef(rotate_x, 1.0, 0.0, 0.0);
  glRotatef(rotate_y, 0.0, 1.0, 0.0);
  glRotatef(rotate_z, 0.0, 0.0, 1.0);

  buildVolume();
  buildObstacle();

  if(voxelToggle == true) {
    drawVoxels();
  }

  if(path.empty()) {
    drawEdges();
  }


  drawNodes();

  if(!path.empty()) {
    drawPath();
  }


  glPopMatrix();




  // swaps the OpenGL pipeline to the viewport
  glutSwapBuffers();
}


//
//  Keyboard Callback Routine
//
void handleKey(unsigned char key, int x, int y){

  switch(key){

    case 'o':
    case 'O':
    {
      float userRadius, userXCoordinate, userYCoordinate, userZCoordinate;
      Obstacles sphere;

      system("clear");
      std::cout << "Placing Obstacle...\n\n";
      std::cout << "Choose the radius of your obstacle (1.0-5.0)" << '\n';
      while(!(std::cin >> userRadius) || userRadius > 5.0 || userRadius < 1.0)
      {
        std::cout << "Error: Enter a valid number: ";

        std::cin.clear();

        std::cin.ignore(123, '\n');
      }

      std::cout << "Choose the center x-location for you obstacle (-15.0-15.0):" << '\n';
      while(!(std::cin >> userXCoordinate) || userXCoordinate > 15.0 || userXCoordinate < -15.0)
      {
        std::cout << "Error: Enter a valid number: ";

        std::cin.clear();

        std::cin.ignore(123, '\n');
      }

      std::cout << "Choose the center y-location for you obstacle (-15.0-15.0):" << '\n';
      while(!(std::cin >> userYCoordinate) || userYCoordinate > 15.0 || userYCoordinate < -15.0)
      {
        std::cout << "Error: Enter a valid number: ";

        std::cin.clear();

        std::cin.ignore(123, '\n');
      }

      std::cout << "Choose the center z-location for you obstacle (-15.0-15.0):" << '\n';
      while(!(std::cin >> userZCoordinate) || userZCoordinate > 15.0 || userZCoordinate < -15.0)
      {
        std::cout << "Error: Enter a valid number: ";

        std::cin.clear();

        std::cin.ignore(123, '\n');
      }
      sphere.position = Vector(userXCoordinate, userYCoordinate, userZCoordinate);
      sphere.radius = userRadius;
      spheres.push_back(sphere);

      std::cout << "Obstacle has been placed...: " << '\n';


      glutPostRedisplay();
    }
      break;

    case 'v':
    case 'V':
      if(spheres.empty()) {
        std::cout << "You must put at least one obstacle on the field" << '\n';
        break;
      }
      system("clear");
      voxelOctrees(baseCube.upperRightCorner, baseCube.bottomLeftCorner, baseCube.center);
      std::cout << "Voxels Built" << '\n';


      glutPostRedisplay();
      break;


    case 'R':
    case 'r':
      buildRoadmap();
      std::cout << "Roadmap Built" << '\n';
      glutPostRedisplay();
      break;

    case 't':
    case 'T':
      if(cubes.size() == 0) {
        std::cout << "There are no Voxels to toggle." << '\n';
        break;
      }
      if(voxelToggle == true) {
        voxelToggle = false;
        std::cout << "Voxels Visible..." << '\n';
        glutPostRedisplay();
        break;
      }
      if(voxelToggle == false) {
        voxelToggle = true;
        std::cout << "Voxels Hidden..." << '\n';
        glutPostRedisplay();
        break;
      }

    case 'a':
    case 'A':
      findPath();
      std::cout << "Path Found" << '\n';
      glutPostRedisplay();
      break;


	  case 'q':		// q or ESC - quit
    case 'Q':
    case 27:
      std::cout << "Closing program..." << '\n';
      exit(0);
      break;


    case 'c':
    case 'C':
      system("clear");
      spheres.clear();
      cubes.clear();
      start.edges.clear();
      goal.edges.clear();
      path.clear();
      cubes.push_back(start);
      cubes.push_back(goal);
      depth = 0;
      std::cout << "Resetting simulation..." << '\n';
      glutPostRedisplay();
      break;

    default:		// not a valid key -- just ignore it
      return;
  }
}

//
//  Keyboard Callback Routine for Arrow Keys
//
void specialInput(int key, int x, int y) {
	switch(key) {

    case GLUT_KEY_RIGHT:
      rotate_y += 5;
      glutPostRedisplay();
      break;

    case GLUT_KEY_LEFT:
      rotate_y -= 5;
      glutPostRedisplay();
      break;

		default:	// not a valid key -- just ignore it
			return;
	}
}

//
// Main program
//
int main(int argc, char* argv[]){
// start up GLUT
glutInit(&argc, argv);

// create the graphics window, giving width, height, and title text
glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
glutInitWindowPosition(80,80);
glutInitWindowSize(WinWidth, WinHeight);
glutCreateWindow("Final Project - Motion Planning");

// specify window clear (background) color to be opaque black
glEnable(GL_DEPTH_TEST);

// set up the callback routines
glutDisplayFunc(display);
glutKeyboardFunc(handleKey);
glutSpecialFunc(specialInput);

// clear the background color
glClearColor(0,0,0,0);

// int *ptr;
// int arr[5];
// ptr = arr;
// std::cout << *(ptr + 1) << '\n';

// set the current view mode to ORTHOGRAPHIC
glMatrixMode(GL_PROJECTION);
glLoadIdentity();

system("clear");
std::cout << "Starting program..." << '\n';
std::cout << "Welcome." << '\n';

start.center = Vector(9, 9, 9);
goal.center = Vector(-9, -9, -9);

cubes.push_back(start);
cubes.push_back(goal);


// Enter GLUT's event loop
glutMainLoop();
return 0;
}
