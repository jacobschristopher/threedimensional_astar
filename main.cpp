#include "main.h"
#include <GL/glut.h>

int main(int argc, char* argv[]){
// start up GLUT
glutInit(&argc, argv);

// create the graphics window attribute
glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
glutInitWindowPosition(80,80);
glutInitWindowSize(800, 600);
glutCreateWindow("Three Dimensional Pathfinder");

// Application object
Application app(800, 600);

app.run();

return 0;
}
