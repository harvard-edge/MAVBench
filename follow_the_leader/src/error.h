#ifndef ERROR_H
#define ERROR_H
#include "bounding_box.h"

class error {
    public:
        double x, y, z, full;
        error();
        error(bounding_box, int, int , double);

};
#endif
