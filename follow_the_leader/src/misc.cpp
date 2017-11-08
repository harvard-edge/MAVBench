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
#include "misc.h"
#include "configs.h"
#include "log__class.h"
#include <fstream>
#include <cmath>
using namespace std;
double target_y(double center, double target, double roll)
{
	return (target-center) + center;
    //cos(roll * M_PI / 180) + center;
}




// *** F:DN manipulating the results
int print_results(vector <bounding_box> bb__vec, vector <double> roll__vec, int img_cols, double height_ratio){
	std::vector<error> error__vec; //collecting bounding box info in this vector
    /* 
       for ( auto &i :bb ) {
       calculate_error(i, img_cols, roll, height_ratio){
       std::cout << i.x << std::endl;
       }
       */
    
    // *** F:DN vars 
    ofstream y__error__handle; //log file
    ofstream x__error__handle; //log file
    ofstream full__error__handle; //log file
    y__error__handle.open("y__error.txt");
    x__error__handle.open("x__error.txt");   
    full__error__handle.open("full__error.txt");   
    
    //*** F:DN calculate the errors 
    cout<<"length"<<bb__vec.size(); 
    for(unsigned i = 0; i < bb__vec.size(); ++i) {
        error__vec.push_back(calculate_error(bb__vec[i], roll__vec[i], img_cols, height_ratio));
        //cout<<bb<<endl;
    }
    
     
    //*** print the errors
    
    for(unsigned i = 0; i < error__vec.size(); ++i) {
        y__error__handle << error__vec[i].y<< " "; 
        x__error__handle << error__vec[i].x<< " "; 
        full__error__handle << error__vec[i].full<< " "; 
        
        //cout<<error__vec[i].y<<endl;
        //cout<<roll__vec[i]<<endl;
        //cout<<bb<<endl;
    }

    cout << endl;
    
    return 0;
}



