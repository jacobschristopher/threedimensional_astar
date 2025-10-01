#ifndef __RENDER_H__
#define __RENDER_H__

#include "Scene.h"

class Render {
    public:
        Render();

        void drawVoxels(const std::vector<Voxels>& cubes);
        void drawEdges(const std::vector<Voxels>& cubes);
        void drawPath(const std::vector<Vector>& path);
        void drawStartGoalNodes(const Voxels& start, const Voxels& goal, bool showEdges);

        void setupCamera(double rotate_x, double rotate_y, double rotate_z);
        void clearScreen();
        void swapBuffers();

    private:
        void drawSphere(const Vector& center, float radius, float r, float g, float b);
        void drawCube(const Vector& upperRight, const Vector& bottomLeft, float r, float g, float b);
};

#endif