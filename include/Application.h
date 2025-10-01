#ifndef __APPLICATION_H__
#define __APPLICATION_H__

#include "Scene.h"
#include "Render.h"
#include <memory>

class Application {
    public:
        Application(int width, int height);
        ~Application();

        void run();

    
    private:
        static Application* instance;

        int winWidth;
        int winHeight;
        std::shared_ptr<Scene> scene;
        std::shared_ptr<Render> renderer;
        Voxels start;
        Voxels goal;
        int depth;
        double rotate_x, rotate_y, rotate_z;
        bool voxelToggle;

        // static GLUT calls
        static void displayCallback();
        static void keyboardCallback(unsigned char key, int x, int y);
        static void specialCallback(int key, int x, int y);

        // instance methods
        void display();
        void handleKey(unsigned char key, int x, int y);
        void specialInput(int key, int x, int y);

        // helper functions
        bool collisionInt(Vector &point);
        bool interpolate(Voxels &q1, Voxels &q2, float stepsize);
        float distance(Voxels &q1, Voxels &q2);
        bool duplicate(Voxels &q1);
        void nearestNeighbors(Voxels &q1);
        void buildRoadmap();
        float clamp(float value, float minimum, float maximum);
        bool collision(Vector URC, Vector BLC, Vector centerPosition);
        void drawVoxels();
        void drawEdges();
        int voxelOctrees(Vector URC, Vector BLC, Vector centerPosition);
        void drawNodes();
        size_t minimumNode(std::vector<Pair> &list);
        float calculateHeuristic(Voxels &q1);
        bool searchList(Pair &node, std::vector<Pair> &list);
        void findPath();
        void drawPath(); 
};

#endif