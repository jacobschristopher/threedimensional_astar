#ifndef __IOHANDLER_H__
#define __IOHANDLER_H__

#include "Scene.h"
#include <iostream>

class IOHandler {

    public:
        IOHandler();

        void handleKey(unsigned char key, int x, int y, Scene& scene);
        void specialInput(int key, int x, int y);

    private:

};

#endif