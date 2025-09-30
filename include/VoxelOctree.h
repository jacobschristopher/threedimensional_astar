#ifndef __VOXELOCTREE_H__
#define __VOXELOCTREE_H__

//#include "Vector.h"

struct VoxelInfo {

};

struct VoxelData {

};

struct Node {
    bool IsLeaf;
    Node *children[8];
    VoxelData data;
};


class VoxelOctree {
    
    public:

        VoxelOctree(int size, int maxDepth);

        void Insert(Vector point);

    private:
        void InsertImpl(Node **node, Vector point, Vector position, int depth);

        int m_Size;
        int m_MaxDepth;
        Node *m_Root;


};


class Octree {

    public:

    

};

#endif
