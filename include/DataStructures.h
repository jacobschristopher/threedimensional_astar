#include "Vector.h"

#include <vector>

typedef std::pair<float, Vector> PathNode;

// A structure that creates Obstacle objects
struct Obstacles{
  Vector position;
  float radius;
};

// A structure that creates Voxel objects
struct Voxels{
  Vector upperRightCorner;
  Vector bottomLeftCorner;
  Vector center;
  std::vector<Voxels> edges;
  PathNode parent;
};

// Defines a pair object for float values for distance and Voxels for open and closed lists
typedef std::pair<float, Voxels> Pair;