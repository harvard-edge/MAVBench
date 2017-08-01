#include <iostream>
#include <algorithm>
#include <vector>
#include <chrono>
#include <limits>
#include <stdint.h>
#include <math.h>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "drone.h"
#include "objdetect.h"
#include "track.h"
#include "pid.h"
#include "string"
#include "configs.h"
#include "misc.h"
#include "log__class.h"

using namespace std;
using namespace std::chrono;
log__class log__f;
//ofstream log__f__handle; //log file

void init_drone(Drone& drone);
void fly_towards_target(Drone& drone, bounding_box bb, cv::Mat depth, int img_rows, int img_cols, PID& pid_vx, PID& pid_vy, PID& pid_vz, double dt, float desired__bb__h);

int main_helper();

int main() {
    while(1) { 
        main_helper();
    }
    return 0;
}


int main_helper()
{
    //----------------------------------------------------------------- 
    // *** F:DN  parameters	
    //----------------------------------------------------------------- 
    string log__f__addr = "log.txt";
    log__f.set__addr(log__f__addr); 
    const double takeoff_h = 1.0;
	const double detect_thresh = 0.8;
	const double track_thresh = 0.85;
	const int max_track = 30;
    const int image_w = 640, image_h = 360;
    float vx__K = 2.0/(image_h/2); //0.0435;//0.0235;
    float vy__K = 1.7/(image_w/2); //0.022;//;0.012;
    float vz__K = 1.0;
    
    //----------------------------------------------------------------- 
    // *** F:DN  vars
    //----------------------------------------------------------------- 
    int total_number_of_kernel_exeuctions= 0;
    int tracking_number_of_invocation = 0;
    int feeding_number_of_invocation = 0;
    int detection_number_of_invocation = 0;
	const string display_name = "Drone View";
    bool is_drone_buffer_empty; //if true, we jumpt to tracking
    bool skip_tracking; //if true, we skip buffering and tracking and go back to detection
    bool collect__height_ratio  = true;
    int track_id;
	std::vector<bounding_box> bounding_box__vector; //collecting bounding box info in this vector
	std::vector<double> roll__vec; //collecting bounding box info in this vector
	std::vector<double> depth__vec; //collecting bounding box info in this vector
    bool collect__data = false; //if true, we start colleclting data
    char key;
    int img__cols__ref; //number of columns in an image (it'll be fixed throughout the program)
    double bb__cntr__x; 
    double bb__cntr__y;

    PID pid_vx(vx__K, 0, 0, 2.5, -2.5);
    PID pid_vy(vy__K, 0, 0, 2, -2);
	PID pid_vz(vz__K, 0, 0, 0.5, -0.5); //at the moment only for keeping drone stable

	steady_clock::time_point last_frame_time;

    //-----------------------------------------------------------------	
    // *** F:DN Body
    //----------------------------------------------------------------- 
    //log__f__handle.open("log.txt");
    //log__f__handle << "Writing into the log file";
    
    cout << "Press enter to connect to drone";
	cin.get();
	//Drone drone("10.157.34.101", 41451);
    Drone drone("10.157.34.208", 41451);
    init_drone(drone);
    YOLODetector detector;
    int pic_id = 0;
    float desired__bb__h; //desired bounding box (desired depth basically)
    
    cv::Mat img;
    cv::Mat depth;
    
    while (1) { 

        //*** F:DN follow the reader
        bounding_box bb = {-1, -1, -1, -1, -1};
        

        // *** F:DN make an attempt to detect
        while (bb.conf < detect_thresh) {
            cout << "Waiting to detect person..." << endl;
            drone.stop_buffering();
            drone.clear_buffer();
            img = drone.read_frame();
            img__cols__ref = img.cols; 
            drone.start_buffering();
            cv::imshow(display_name, img);
            
            key = (char) cv::waitKey(1); 
            if ('q' == key) 
                goto end;
            if ('s' == key) 
                collect__data = !collect__data;

            bb = detector.detect_person(img);
            bb__cntr__y =  bb.y + bb.h/2;
            bb__cntr__x =  bb.x + bb.w/2;
            
            if (collect__data) { //collect data
                //cout<<"got in"<<endl; 
                
                bounding_box__vector.push_back(bb);
                roll__vec.push_back(drone.get_roll());
                //depth__vec.depth(bb__cntr__x, bb__cntr__y)
            }
            
            total_number_of_kernel_exeuctions +=1;	
            detection_number_of_invocation +=1;
        }

        // *** F:DN collect some data
        if (collect__height_ratio) { 
            collect__height_ratio  = false;
            desired__bb__h =  float(bb.h)/img.cols;
        } 
        last_frame_time = steady_clock::now();
        cout << "Detected person!" << endl;
        cv::rectangle(img, cv::Point(bb.x, bb.y), cv::Point(bb.x+bb.w, bb.y+bb.h), cv::Scalar(0,255,255)); //yellow
        cv::imshow(display_name, img);
        key = (char) cv::waitKey(1); 
        if ('q' == key) 
            goto end;
        if ('s' == key) 
            collect__data = !collect__data;
        
        // *** F:DN make an attempt to track	
        KCFtracker tracker(bb, img);
        pid_vx.reset();
        pid_vy.reset();
        pid_vz.reset();

        // *** F:DN track the buffered images	
        cout << "Feeding buffer into tracker" << endl;
        is_drone_buffer_empty = drone.buffer_empty();
        skip_tracking = false; 

        /*
        while (!is_drone_buffer_empty) {
            
            total_number_of_kernel_exeuctions +=1;	
            feeding_number_of_invocation +=1; 
            img = drone.read_frame_buf(&is_drone_buffer_empty);
            bb = tracker.track(img);
            bb__cntr__y =  bb.y + bb.h/2;
            bb__cntr__x =  bb.x + bb.w/2;
            
            if (bb.conf < track_thresh) { //can not track
                skip_tracking = true; 
                break; 
            }

            if (collect__data) { //collect data
                //cout<<"got in"<<endl; 
                bounding_box__vector.push_back(bb);
                roll__vec.push_back(drone.get_roll());
                //depth__vec.depth(bb__cntr__x, bb__cntr__y)
            }
            //fly_towards_target(drone, bb, depth, img.rows, img.cols, pid_vx, pid_vy, pid_vz, 1, desired__bb__h);

            cv::rectangle(img, cv::Point(bb.x, bb.y), cv::Point(bb.x+bb.w, bb.y+bb.h), cv::Scalar(255,0,255)); //purple
            cv::imshow(display_name, img);
            
            key = (char) cv::waitKey(1); 
            if ('q' == key) 
                goto end;
            if ('s' == key) 
                collect__data = !collect__data;
            //drone.read_frame_buf();
        }
        drone.stop_buffering();
        */

        drone.stop_buffering();
        auto buf_images = drone.buffered_images();
        for (int i = 0; i < buf_images.size(); i += std::max(1, (int)buf_images.size() / 10)) {
            img = buf_images[i];
            bb = tracker.track(img);

            bb__cntr__y =  bb.y + bb.h/2;
            bb__cntr__x =  bb.x + bb.w/2;
            
            if (bb.conf < track_thresh) { //can not track
                skip_tracking = true; 
                break; 
            }

            if (collect__data) { //collect data
                //cout<<"got in"<<endl; 
                bounding_box__vector.push_back(bb);
                roll__vec.push_back(drone.get_roll());
                //depth__vec.depth(bb__cntr__x, bb__cntr__y)
            }
            //fly_towards_target(drone, bb, depth, img.rows, img.cols, pid_vx, pid_vy, pid_vz, 1, desired__bb__h);

            cv::rectangle(img, cv::Point(bb.x, bb.y), cv::Point(bb.x+bb.w, bb.y+bb.h), cv::Scalar(255,0,255)); //purple
            cv::imshow(display_name, img);
            
            key = (char) cv::waitKey(1); 
            if ('q' == key) 
                goto end;
            if ('s' == key) 
                collect__data = !collect__data;
        }

        if (skip_tracking) { 
            cout<<"SKIP TRACKING"<<endl; 
            continue;
        }


        // *** F:DN track real time now
        cout << "Done. Now tracking..." << endl;
        img = drone.read_frame();
        //depth = drone.read_frame_depth();
        track_id = 0;
        //int counter = 0;	
        do {
            total_number_of_kernel_exeuctions +=1;
            tracking_number_of_invocation +=1;
            steady_clock::time_point curr_time = steady_clock::now();
            duration<double> dt = duration_cast<duration<double>>(curr_time - last_frame_time);

            if (collect__data) { //collect data
                //cout<<"got in"<<endl; 
                bounding_box__vector.push_back(bb);
                roll__vec.push_back(drone.get_roll());
                //depth__vector.depth(bb__cntr__x, bb__cntr__y)
            }

            fly_towards_target(drone, bb, depth, img.rows, img.cols, pid_vx, pid_vy, pid_vz, dt.count(), desired__bb__h);

            last_frame_time = curr_time;

            //if (counter > 0) {	
            cv::rectangle(img, cv::Point(bb.x, bb.y), cv::Point(bb.x+bb.w, bb.y+bb.h), cv::Scalar(255,255,0)); //blue
            //}	
            cv::imshow(display_name, img);

            
            key = (char) cv::waitKey(1); 
            if ('q' == key) 
                goto end;
            if ('s' == key) 
                collect__data = !collect__data;
            
            img = drone.read_frame();
            //depth = drone.read_frame_depth();

            bb = tracker.track(img);
            bb__cntr__y =  bb.y + bb.h/2;
            bb__cntr__x =  bb.x + bb.w/2;
            //counter +=1;	
            track_id++;
        } while (bb.conf >= track_thresh && track_id < max_track);

    }

    end:
    drone.stop_buffering();
    cv::destroyAllWindows();
    cout <<"\ndetection_number_of_invocation : "<<float(detection_number_of_invocation)/float(total_number_of_kernel_exeuctions)<<endl;	
    cout <<"feeding_number_of_invocation : "<<float(feeding_number_of_invocation)/float(total_number_of_kernel_exeuctions)<<endl;	
    cout <<"tracking_number_of_invocation : "<<float(tracking_number_of_invocation)/float(total_number_of_kernel_exeuctions)<<endl;	
    cout << "\nFinished\n";

    print_results(bounding_box__vector, roll__vec, img__cols__ref, desired__bb__h);

    //init_drone(drone);
    return 0;
}

void init_drone(Drone& drone)
{
	cout << "Initialize drone:\n";
	cout << "\ta: arm\n";
	cout << "\td: disarm\n";
	cout << "\tt h: takeoff to h m\n";
	cout << "\tl: land\n";
	cout << "\tf x y z d: fly at (x,y,z) m/s for d s\n";
	cout << "\ty x: set yaw to x\n";
	cout << "\tp: print pitch, roll, yaw, height\n";
	cout << "\tc: complete drone setup and continue\n";
	cout << "\tq: quit\n";

	char cmd = '\0';
	while (cmd != 'c') {
		cin >> cmd;

		if (cmd == 'a') {
			drone.arm();
		} else if (cmd == 'd') {
			drone.disarm();
		} else if (cmd == 't') {
			double height;
			cin >> height;
			cin.ignore(numeric_limits<streamsize>::max(), '\n');
			drone.takeoff(height);
		} else if (cmd == 'l') {
			drone.land();
		} else if (cmd == 'f') {
			double x,y,z,d;
			cin >> x >> y >> z >> d;
			drone.fly_velocity(x, y, z, d);
		} else if (cmd == 'y') {
			double x;
			cin >> x;
			drone.set_yaw(x);
		} else if (cmd == 'p') {
			cout << "pitch: " << drone.get_pitch() << " roll: " << drone.get_roll() << " yaw: " << drone.get_yaw() << " height: " << drone.gps().z << endl;
		} else if (cmd == 'q') {
			exit(0);
		} else if (cmd != 'c') {
			cout << "Unknown command" << endl;
		}
	}
	cin.ignore(numeric_limits<streamsize>::max(), '\n');
}

void fly_towards_target(Drone& drone, bounding_box bb, cv::Mat depth, int img_rows, int img_cols, PID& pid_vx, PID& pid_vy, PID& pid_vz, double dt, float desired__bb__h)
{
	static float hover_height = drone.gps().z;
	const float target_distance = 0.6;
	//const float height_ratio = 0.01;
	float height_ratio = desired__bb__h;

	/*double total_depth = 0;
	//double depth_samples = 0;
	std::vector<double> depths;
	const int step = 1;
	for (int r = bb.y; r < bb.y + bb.h; r += step) {
		for (int c = bb.x; c < bb.x + bb.w; c += step) {
			//total_depth += depth.data[256 - (depth.cols * r + c)];
			//depth_samples++;
			depths.push_back(depth.data[256 - (depth.cols * r + c)]);
		}
	}
	//double target_depth = total_depth / depth_samples;
	nth_element(depths.begin(), depths.begin() + depths.size()/2, depths.end());
	double target_depth = double(depths[depths.size()/2]) / 100;*/

	auto yaw = drone.get_yaw();
	if (yaw > 15 || yaw < -15) {
		cout << "Correcting yaw\n";
		drone.set_yaw(0);
	}

	auto roll = drone.get_roll();

	
    double bb__cntr__x =  target_y(img_cols/2, bb.x + bb.w/2, roll);
    double bb__cntr__y =  bb.y + bb.h/2;
    //double bb__cntr__x =  target_x(img_cols/2, bb.x + bb.w/2, roll);
    
    double img__cntr =  img_cols / 2;
    // double vx = pid_vx.calculate(target_depth, target_distance, dt);
	double vx = pid_vx.calculate(bb.h, height_ratio*img_cols,  dt); //get closer to the person
    //double vx = 0; 
    double vy = pid_vy.calculate(img__cntr, bb__cntr__x, dt); //keep the person in the center of the img 
	double vz = pid_vz.calculate(drone.gps().z, hover_height, dt); //for hovering at the same point

	drone.fly_velocity(vx, vy, vz);
	//cout << target_depth << ' ' << vx << endl;
	//cout << img_cols*height_ratio - bb.h  << ' ' << vx << endl;
	//cout << vx << ' ' << vy << ' ' << vz << endl;
}

