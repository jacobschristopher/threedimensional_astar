#include "VoxelOctree.h"
#include "Vector.h"

#include <GL/glut.h>



VoxelOctree::VoxelOctree(int size, int maxDepth) {
    m_Size = size;
    m_MaxDepth = maxDepth;
}


VoxelOctree::InsertImpl(Node **node, Vector point, Vector position, int depth) {
    if (*node == nullptr) {
        *node = new Node;
    }

    if (depth == m_MaxDepth) {
        (*node)->IsLeaf = true;
        return;
    }

    float size = m_Size / std::exp2(depth);

    Vector childPos;
}