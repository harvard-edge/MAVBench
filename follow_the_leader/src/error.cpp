#include <iostream>
#include <algorithm>
#include <vector>
#include <chrono>
#include <limits>
#include <stdint.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Drone.h"
#include "objdetect.h"
#include "track.h"
#include "pid.h"
#include "string"
#include <fstream>
#include <iostream>
//#include "configs.h"
//#include "log__class.h"
#include <fstream>
#include <cmath>
#include "error.h"
using namespace std;

/*
double target_y(double center, double target)
{
	return (target-center) + center;
    //cos(roll * M_PI / 180) + center;
}
*/
error calculate_error(bounding_box bb, int img_cols, double height_ratio){
    error my_error = {-1, -1, -1, -1};
    /*    
    double bb_cntr =  target_y(img_cols/2, bb.x + bb.w/2);
    double img_col_cntr =  img_cols / 2;
    my_error.x = abs(((height_ratio*img_cols)/bb.h)  - 1) ; //into the screen
    my_error.y = abs(img_col_cntr -  bb_cntr)/(img_col_cntr); 
    my_error.full = (my_error.y + my_error.x)/2;

    my_error.z = abs(0);
    */ 
    return my_error;

}


