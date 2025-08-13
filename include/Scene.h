#ifndef __SCENE_H__
#define __SCENE_H__

//#include "Vector.h"
#include "DataStructures.h"

#include <vector>

class Scene {

    public:
        // std::vector<Voxels> cubes;
        // std::vector<Obstacles> spheres;
        // std::vector<Vector> path;

        void buildVolume();
        void buildObstacle();

    private:
        Voxels baseCube;
        std::vector<Voxels> cubes;
        std::vector<Obstacles> spheres;
        std::vector<Vector> path;


};

#endif