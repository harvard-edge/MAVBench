#ifndef ERROR_H
#define ERROR_H
struct error {
 double x, y,z, full;
 //full is (x + y)/2
};

error calculate_error(bounding_box bb, int img_cols, double height_ratio);
#endif
