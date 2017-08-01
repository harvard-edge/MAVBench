#include <iostream>
#include <algorithm>
#include <vector>
#include <chrono>
#include <limits>
#include <stdint.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "drone.h"
#include "objdetect.h"
#include "track.h"
#include "pid.h"
#include "string"
#include <fstream>
#include <iostream>
#include "misc.h"
#include "configs.h"
#include "log__class.h"
#include <fstream>
#include <cmath>
using namespace std;

// *** F:DN calculate the error associated with follow the leader
// they are all normalized

error calculate_error(bounding_box bb, double roll, int img_cols, double height_ratio){
    double bb__cntr =  target_y(img_cols/2, bb.x + bb.w/2, roll);
    double img__col__cntr =  img_cols / 2;
    error my_error = {-1, -1, -1, -1};

    // my_error.x = abs(height_ratio*img_cols - bb.h)/(height_ratio*img_cols)  ;
    my_error.x = abs(((height_ratio*img_cols)/bb.h)  - 1) ; //into the screen
    
    //double vx = 0; 
    my_error.y = abs(img__col__cntr -  bb__cntr)/(img__col__cntr); 
    my_error.full = (my_error.y + my_error.x)/2;

    my_error.z = abs(0);
    return my_error;

}


