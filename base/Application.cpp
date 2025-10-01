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
    renderer = std::make_shared<Render>();
    io = std::make_shared<IOHandler>();
    
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

    renderer->setupCamera(rotate_x, rotate_y, rotate_z);
    
    scene->buildVolume();
    scene->buildObstacle();

    if(voxelToggle) {
        renderer->drawVoxels(scene->getCubes());
    }

    if(scene->getPath().empty()) {  // FIX: Access path through scene
        renderer->drawEdges(scene->getCubes());
    }

    renderer->drawStartGoalNodes(start, goal, true);

    if(!scene->getPath().empty()) {  // FIX: Access path through scene
        renderer->drawPath(scene->getPath());
    }

    renderer->clearScreen();
    renderer->swapBuffers();
}

// Input handlers
void Application::handleKey(unsigned char key, int x, int y) {
    io->handleKey(key, x, y, *scene);

}

void Application::specialInput(int key, int x, int y) {
    io->specialInput(key, x, y);
}

// Helper methods 
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