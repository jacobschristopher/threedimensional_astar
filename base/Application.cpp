#include "Application.h"
#include <GL/glut.h>
#include <iostream>
#include <cmath>
#include <set>

Application* Application::instance = nullptr;

Application::Application(int width, int height)
    : winWidth(width), winHeight(height), depth(0),
      rotate_x(0), rotate_y(0), rotate_z(0), voxelToggle(true) {
    
    instance = this;
    scene = std::make_shared<Scene>();
    
    // starting and end positions
    start.center = Vector(9, 9, 9);
    goal.center = Vector(-9, -9, -9);
    
    // initialize positions
    scene->getCubes().push_back(start);
    scene->getCubes().push_back(goal);
}

Application::~Application() {
    instance = nullptr;
}

void Application::run() {
    glutDisplayFunc(displayCallback);
    glutKeyboardFunc(keyboardCallback);
    glutSpecialFunc(specialCallback);
   
    glEnable(GL_DEPTH_TEST);
    glClearColor(0, 0, 0, 0);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
   
    std::cout << "Starting program..." << '\n';
    std::cout << "Welcome." << '\n';
   
    glutMainLoop();
}

// Static callbacks
void Application::displayCallback() {
    if (instance) instance->display();
}

void Application::keyboardCallback(unsigned char key, int x, int y) {
    if (instance) instance->handleKey(key, x, y);
}

void Application::specialCallback(int key, int x, int y) {
    if (instance) instance->specialInput(key, x, y);
}

// Display
void Application::display() {
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
    
    scene->buildVolume();
    scene->buildObstacle();

    if(voxelToggle) {
        drawVoxels();
    }

    if(scene->getPath().empty()) {  // FIX: Access path through scene
        drawEdges();
    }

    drawNodes();

    if(!scene->getPath().empty()) {  // FIX: Access path through scene
        drawPath();
    }

    glPopMatrix();
    glutSwapBuffers();
}

// Input handlers
void Application::handleKey(unsigned char key, int x, int y) {
    auto& spheres = scene->getObstacles();
    auto& cubes = scene->getCubes();
    auto& path = scene->getPath();
    auto& baseCube = scene->getBaseCube();

    switch(key) {
        case 'o':
        case 'O': {
            float userRadius, userXCoordinate, userYCoordinate, userZCoordinate;
            Obstacles sphere;

            system("cls");
            std::cout << "Placing Obstacle...\n\n";
            std::cout << "Choose the radius of your obstacle (1.0-5.0)" << '\n';
            while(!(std::cin >> userRadius) || userRadius > 5.0 || userRadius < 1.0) {
                std::cout << "Error: Enter a valid number: ";
                std::cin.clear();
                std::cin.ignore(123, '\n');
            }

            std::cout << "Choose the center x-location for you obstacle (-15.0-15.0):" << '\n';
            while(!(std::cin >> userXCoordinate) || userXCoordinate > 15.0 || userXCoordinate < -15.0) {
                std::cout << "Error: Enter a valid number: ";
                std::cin.clear();
                std::cin.ignore(123, '\n');
            }

            std::cout << "Choose the center y-location for you obstacle (-15.0-15.0):" << '\n';
            while(!(std::cin >> userYCoordinate) || userYCoordinate > 15.0 || userYCoordinate < -15.0) {
                std::cout << "Error: Enter a valid number: ";
                std::cin.clear();
                std::cin.ignore(123, '\n');
            }

            std::cout << "Choose the center z-location for you obstacle (-15.0-15.0):" << '\n';
            while(!(std::cin >> userZCoordinate) || userZCoordinate > 15.0 || userZCoordinate < -15.0) {
                std::cout << "Error: Enter a valid number: ";
                std::cin.clear();
                std::cin.ignore(123, '\n');
            }
            
            sphere.position = Vector(userXCoordinate, userYCoordinate, userZCoordinate);
            sphere.radius = userRadius;
            spheres.push_back(sphere);
            std::cout << "Obstacle has been placed..." << '\n';
            glutPostRedisplay();
        }
        break;

        case 'v':
        case 'V':
            if(spheres.empty()) {
                std::cout << "You must put at least one obstacle on the field" << '\n';
                break;
            }
            system("cls");
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
            voxelToggle = !voxelToggle;
            std::cout << (voxelToggle ? "Voxels Visible..." : "Voxels Hidden...") << '\n';
            glutPostRedisplay();
            break;

        case 'a':
        case 'A':
            findPath();
            std::cout << "Path Found" << '\n';
            glutPostRedisplay();
            break;

        case 'q':
        case 'Q':
        case 27:
            std::cout << "Closing program..." << '\n';
            exit(0);
            break;

        case 'c':
        case 'C':
            system("cls");
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

        default:
            return;
    }
}

void Application::specialInput(int key, int x, int y) {
    switch(key) {
        case GLUT_KEY_RIGHT:
            rotate_y += 5;
            glutPostRedisplay();
            break;

        case GLUT_KEY_LEFT:
            rotate_y -= 5;
            glutPostRedisplay();
            break;

        default:
            return;
    }
}

// Helper methods - COPY THESE FROM YOUR Final_Project.cpp
bool Application::collisionInt(Vector &point) {
    auto& spheres = scene->getObstacles();
    for(size_t i = 0; i < spheres.size(); i++) {
        float mag = (spheres[i].position - point).magnitude();
        if(mag < spheres[i].radius) {
            return true;
        }
    }
    return false;
}

bool Application::interpolate(Voxels &q1, Voxels &q2, float stepsize) {
    Vector step = (q2.center - q1.center) / stepsize;
    Vector current = q1.center;

    for(int i = 0; i < stepsize; i++) {
        current += step;
        if(collisionInt(current)) {
            return true;
        }
    }
    return false;
}

float Application::distance(Voxels &q1, Voxels &q2) {
    return (q1.center - q2.center).magnitude();
}

bool Application::duplicate(Voxels &q1) {
    for (size_t i = 0; i < q1.edges.size(); i++) {
        if(q1.center == q1.edges[i].center) {
            return true;
        }
    }
    return false;
}

void Application::nearestNeighbors(Voxels &q1) {
    auto& cubes = scene->getCubes();
    for(size_t i = 0; i < cubes.size(); i++) {
        float len = distance(q1, cubes[i]);
        if(len < 20.0 && len != 0.0) {
            if(!interpolate(q1, cubes[i], 50.0) && !duplicate(cubes[i])) {
                q1.edges.push_back(cubes[i]);
            }
        }
    }
}

void Application::buildRoadmap() {
    auto& cubes = scene->getCubes();
    for(size_t i = 0; i < cubes.size(); i++) {
        nearestNeighbors(cubes[i]);
    }
}

float Application::clamp(float value, float minimum, float maximum) {
    return std::max(minimum, std::min(maximum, value));
}

bool Application::collision(Vector URC, Vector BLC, Vector centerPosition) {
    auto& spheres = scene->getObstacles();
    for(size_t i = 0; i < spheres.size(); i++) {
        Vector difference = (spheres[i].position - centerPosition);

        float halfWidth = abs(URC[0] - centerPosition[0]);
        float halfHeight = abs(URC[1] - centerPosition[1]);
        float halfDepth = abs(URC[2] - centerPosition[2]);

        float clampX = clamp(difference[0], -halfWidth, halfWidth);
        float clampY = clamp(difference[1], -halfHeight, halfHeight);
        float clampZ = clamp(difference[2], -halfDepth, halfDepth);

        Vector closest = Vector(centerPosition[0] + clampX, 
                               centerPosition[1] + clampY, 
                               centerPosition[2] + clampZ);
        Vector newDifference = (closest - spheres[i].position);
        float length = sqrt(pow(newDifference[0], 2) + 
                          pow(newDifference[1], 2) + 
                          pow(newDifference[2], 2));

        if(length < spheres[i].radius) {
            return true;
        }
    }
    return false;
}

void Application::drawVoxels() {
    auto& cubes = scene->getCubes();
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

void Application::drawEdges() {
    auto& cubes = scene->getCubes();
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

int Application::voxelOctrees(Vector URC, Vector BLC, Vector centerPosition) {
    auto& cubes = scene->getCubes();
    
    Voxels cubeOne, cubeTwo, cubeThree, cubeFour, cubeFive, cubeSix, cubeSeven, cubeEight;
    std::vector<Voxels> children;

    // Top Left Front Quadrant
    cubeOne.upperRightCorner = Vector(centerPosition[0], URC[1], URC[2]);
    cubeOne.bottomLeftCorner = Vector(BLC[0], centerPosition[1], centerPosition[2]);
    cubeOne.center = (cubeOne.upperRightCorner + cubeOne.bottomLeftCorner) / 2;

    cubeTwo.upperRightCorner = Vector(centerPosition[0], centerPosition[1], centerPosition[2]);
    cubeTwo.bottomLeftCorner = Vector(BLC[0], BLC[1], URC[2]);
    cubeTwo.center = (cubeTwo.upperRightCorner + cubeTwo.bottomLeftCorner) / 2;

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

    children.push_back(cubeOne);
    children.push_back(cubeTwo);
    children.push_back(cubeThree);
    children.push_back(cubeFour);
    children.push_back(cubeFive);
    children.push_back(cubeSix);
    children.push_back(cubeSeven);
    children.push_back(cubeEight);

    depth++;

    for(size_t i = 0; i < children.size(); i++) {
        if(collision(children[i].upperRightCorner, children[i].bottomLeftCorner, children[i].center)) {
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

void Application::drawNodes() {
    auto& path = scene->getPath();
    float theta = (2 * M_PI) / 50;

    // Draw start node (green)
    for (float v = 0.0; v < M_PI; v+= theta) {
        glBegin(GL_TRIANGLE_STRIP);
        for (float u = 0.0; u < 2 * M_PI; u+= theta) {
            glColor3f(0.2, 0.8, 0.1);
            glVertex3f((cos(u) * sin(v) * 1) + start.center[0],
                      (sin(u) * sin(v) * 1) + start.center[1], 
                      (cos(v) * 1) + start.center[2]);
            glVertex3f((cos(u) * sin(v + theta) * 1) + start.center[0], 
                      (sin(u) * sin(v + theta) * 1) + start.center[1], 
                      (cos(v + theta) * 1) + start.center[2]);
        }
        glEnd();
    }

    // Draw goal node (blue)
    for (float v = 0.0; v < M_PI; v+= theta) {
        glBegin(GL_TRIANGLE_STRIP);
        for (float u = 0.0; u < 2 * M_PI; u+= theta) {
            glColor3f(0.1, 0.1, 0.8);
            glVertex3f((cos(u) * sin(v) * 1) + goal.center[0],
                      (sin(u) * sin(v) * 1) + goal.center[1], 
                      (cos(v) * 1) + goal.center[2]);
            glVertex3f((cos(u) * sin(v + theta) * 1) + goal.center[0], 
                      (sin(u) * sin(v + theta) * 1) + goal.center[1], 
                      (cos(v + theta) * 1) + goal.center[2]);
        }
        glEnd();
    }

    // Draw edges if path not found
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

size_t Application::minimumNode(std::vector<Pair> &list) {
    Pair minimum;
    size_t index = 0;
    minimum.first = 99.0;

    for(size_t i = 0; i < list.size(); i++) {
        if(list[i].first < minimum.first) {
            minimum = list[i];
            index = i;
        }
    }
    return index;
}

float Application::calculateHeuristic(Voxels &q1) {
    return (start.center - q1.center).magnitude();
}

bool Application::searchList(Pair &node, std::vector<Pair> &list) {
    for(size_t i = 0; i < list.size(); i++) {
        if(list[i].second.center == node.second.center) {
            return true;
        }
    }
    return false;
}

void Application::findPath() {
    auto& cubes = scene->getCubes();
    auto& path = scene->getPath();
    
    nearestNeighbors(start);
    nearestNeighbors(goal);

    std::vector<Pair> openList;
    std::vector<Pair> closedList;

    Pair end = std::make_pair(0.0, goal);
    openList.push_back(end);

    while(!openList.empty()) {
        size_t offset = minimumNode(openList);
        Pair node = openList[offset];
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
            
            if(!searchList(newNode, openList) && !searchList(newNode, closedList)) {
                openList.push_back(newNode);

                for(size_t j = 0; j < cubes.size(); j++) {
                    if(newNode.second.center == cubes[j].center) {
                        if(cubes[j].parent.first == 0 || cubes[j].parent.first > newNode.first) {
                            cubes[j].parent = std::make_pair(f, node.second.center);
                        }
                    }
                }
            }
        }
    }

    Vector traverse = start.center;
    path.push_back(traverse);

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

void Application::drawPath() {
    auto& path = scene->getPath();
    
    for(size_t i = 0; i < path.size() - 1; i++) {
        glBegin(GL_LINES);
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(path[i][0], path[i][1], path[i][2]);
        glVertex3f(path[i + 1][0], path[i + 1][1], path[i + 1][2]);
        glEnd();
    }
}