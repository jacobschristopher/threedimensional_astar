#include "IOHandler.h"
#include <GL/glut.h>

IOHandler::IOHandler() {}

void IOHandler::handleKey(unsigned char key, int x, int y, Scene& scene) {
    auto& spheres = scene.getObstacles();
    auto& cubes = scene.getCubes();
    auto& path = scene.getPath();
    auto& baseCube = scene.getBaseCube();

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
            //voxelOctrees(baseCube.upperRightCorner, baseCube.bottomLeftCorner, baseCube.center);
            //std::cout << "Voxels Built" << '\n';
            glutPostRedisplay();
            break;

        case 'R':
        case 'r':
            //buildRoadmap();
            //std::cout << "Roadmap Built" << '\n';
            glutPostRedisplay();
            break;

        case 't':
        case 'T':
            if(cubes.size() == 0) {
                std::cout << "There are no Voxels to toggle." << '\n';
                break;
            }
            //voxelToggle = !voxelToggle;
            //std::cout << (voxelToggle ? "Voxels Visible..." : "Voxels Hidden...") << '\n';
            glutPostRedisplay();
            break;

        case 'a':
        case 'A':
            //findPath();
            //std::cout << "Path Found" << '\n';
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
            //start.edges.clear();
            //goal.edges.clear();
            path.clear();
            //cubes.push_back(start);
            //cubes.push_back(goal);
            //depth = 0;
            std::cout << "Resetting simulation..." << '\n';
            glutPostRedisplay();
            break;

        default:
            return;
    }
}

void IOHandler::specialInput(int key, int x, int y) {
     switch(key) {
        case GLUT_KEY_RIGHT:
            //rotate_y += 5;
            glutPostRedisplay();
            break;

        case GLUT_KEY_LEFT:
            //rotate_y -= 5;
            glutPostRedisplay();
            break;

        default:
            return;
    }
}