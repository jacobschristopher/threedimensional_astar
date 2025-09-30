#ifndef __SCENE_H__
#define __SCENE_H__

//#include "Vector.h"
#include "DataStructures.h"

#include <vector>

class Scene {

    public:
        Scene();

        void buildVolume();
        void buildObstacle();

        std::vector<Voxels>& getCubes() {return cubes; }
        std::vector<Obstacles>& getObstacles() {return spheres; }
        std::vector<Vector>& getPath() {return path; }
        Voxels& getBaseCube() {return baseCube; }

        const std::vector<Voxels>& getCubes() const { return cubes; }
        const std::vector<Obstacles>& getObstacles() const { return spheres; }
        const std::vector<Vector>& getPath() const { return path; }
        const Voxels& getBaseCube() const { return baseCube; }

    private:
        Voxels baseCube;
        std::vector<Voxels> cubes;
        std::vector<Obstacles> spheres;
        std::vector<Vector> path;

};

#endif