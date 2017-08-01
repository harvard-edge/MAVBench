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
#include "configs.h"
#include "log__class.h"
#include "error.h"

double target_y(double center, double target, double roll);
error calculate_error(bounding_box bb, double roll, int img_cols,  double height_ratio);
int print_results(vector <bounding_box> bb__vec, vector <double> roll__vec, int img_cols, double height_ratio);
