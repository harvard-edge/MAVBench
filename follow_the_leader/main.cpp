#include <iostream>
#include <algorithm>
#include <vector>
#include <chrono>
#include <limits>
#include <fstream>
#include <mutex>
#include <atomic>
#include <stdint.h>
#include <math.h>
#include <signal.h> // To catch sigint

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
log__class log__f;

typedef YOLODetector detector_t;
typedef KCFtracker tracker_t;

// Global variables for thread management
detector_t * detector;
tracker_t * tracker = nullptr;
bounding_box target_bb = {-1, -1, -1, -1};
std::atomic<bool> end_threads;
std::mutex tracker_mutex;
std::mutex target_bb_mutex;
std::atomic<int> img_id(0);

// Gives user manual keyboard control of the drone
void init_drone(Drone& drone);

// Directs the drone to fly towards the target in the bounding box
// TODO: this function currently requires that the drone stabilize its camera
// with a gimbal. We should remove that requirement.
void fly_towards_target(Drone& drone, const bounding_box& bb,
        int img_rows, int img_cols, PID& pid_vx, PID& pid_vy, PID& pid_vz,
        double dt);

// Thread loop that detects targets in drone's camera feed
void detect_thread_loop(Drone& drone);

// Thread loop that tracks targets in drone's camera feed while waiting for
// detection algorithms to complete
void track_thread_loop(Drone& drone);

// Signal handler used to cleanup threads when Ctrl-c is pressed
void sig_handler(int signo);

int main()
{
    //----------------------------------------------------------------- 
    // *** F:DN  parameters	
    //----------------------------------------------------------------- 
    const int image_w = 640, image_h = 360;
    float vx__K = 1.6/(image_h/2); 
    float vy__K = 1.6/(image_w/2); 
    float vz__K = 1.0;
    
    //----------------------------------------------------------------- 
    // *** F:DN  vars
    //----------------------------------------------------------------- 
	const string display_name = "Drone View";

    PID pid_vx(vx__K, 0, 0, 2, -2);
    PID pid_vy(vy__K, 0, 0, 2, -2);
	PID pid_vz(vz__K, 0, 0, 0.5, -0.5); //at the moment only for keeping drone stable

    //-----------------------------------------------------------------	
    // *** F:DN Body
    //----------------------------------------------------------------- 
    
    cout << "Press enter to connect to drone";
	cin.get();
	//Drone drone("10.157.34.101", 41451);
    Drone drone("10.157.34.208", 41451);
    init_drone(drone);

    // Initialize detector
    detector = new detector_t;

    // Install signal handler
    signal(SIGINT, sig_handler);

    // Spawn threads
    end_threads = false;
    std::thread detect_thread(detect_thread_loop, std::ref(drone));
    std::thread track_thread(track_thread_loop, std::ref(drone));
    drone.start_buffering();

    // Main thread loop
    cout << "Following target..." << endl;
    while(!end_threads) { 
        target_bb_mutex.lock();

        if (target_bb.x != -1)
            fly_towards_target(drone, target_bb, image_h, image_w, pid_vx, pid_vy, pid_vz, 1); // dt is not currently used
        else
            drone.fly_velocity(0, 0 , 0);

        target_bb_mutex.unlock();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    cout << "Waiting for all threads to end..." << endl;
    detect_thread.join();
    track_thread.join();
    drone.stop_buffering();

    return 0;
}

void detect_thread_loop(Drone& drone)
{
	const double detect_thresh = 0.8;
    
    while (!end_threads) {
        cv::Mat img = drone.read_frame_buf_newest();

        if (img.empty())
            continue;

        drone.clear_buffer();

        bounding_box bb = detector->detect_person(img);

        if (bb.conf < detect_thresh || bb.x == -1)
            continue;

        tracker_mutex.lock();

        if (tracker != nullptr)
            delete tracker;
        tracker = new tracker_t(bb, img);

        std::vector<cv::Mat> buffered_imgs = drone.buffered_images();

        const int k = 10;
        int step = std::max(1, (int)buffered_imgs.size() / k);
        for (int i = 0; i < buffered_imgs.size(); i += k) {
            bb = tracker->track(buffered_imgs[i]);
        }

        target_bb_mutex.lock();
        target_bb = bb;
        target_bb_mutex.unlock();

        tracker_mutex.unlock();

        // cv::rectangle(img, cv::Point(bb.x, bb.y), cv::Point(bb.x+bb.w, bb.y+bb.h), cv::Scalar(0,255,255)); //yellow
        // cv::imwrite("imgs/"+std::to_string(img_id++)+".jpg", img);
    }
}

void track_thread_loop(Drone& drone)
{
	const double track_thresh = 0.9;
    
    while (!end_threads) {
        cv::Mat img = drone.read_frame_buf_newest();

        if (img.empty())
            continue;

        tracker_mutex.lock();

        if (tracker == nullptr) {
            tracker_mutex.unlock();
            continue;
        }

        bounding_box bb = tracker->track(img);
        
        tracker_mutex.unlock();

        target_bb_mutex.lock();
        if (bb.conf >= track_thresh) {
            target_bb = bb;
        } else {
            target_bb = {-1, -1, -1, -1};
        }
        target_bb_mutex.unlock();

        // cv::rectangle(img, cv::Point(bb.x, bb.y), cv::Point(bb.x+bb.w, bb.y+bb.h), cv::Scalar(255,255,0)); //blue
        // cv::imwrite("imgs/"+std::to_string(img_id++)+".jpg", img);
    }
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

void fly_towards_target(Drone& drone, const bounding_box& bb,
        int img_rows, int img_cols, PID& pid_vx, PID& pid_vy, PID& pid_vz,
        double dt)
{
	static float hover_height = drone.gps().z;
	const float height_ratio = 0.2;

	auto yaw = drone.get_yaw();
	if (yaw > 15 || yaw < -15) {
		cout << "Correcting yaw\n";
		drone.set_yaw(0);
	}
	
    double bb__cntr__x =  bb.x + bb.w/2;
    double bb__cntr__y =  bb.y + bb.h/2;
    
    double img__cntr =  img_cols / 2;
	double vx = pid_vx.calculate(bb.h, height_ratio*img_cols,  dt); //get closer to the person
    double vy = pid_vy.calculate(img__cntr, bb__cntr__x, dt); //keep the person in the center of the img 
	double vz = pid_vz.calculate(drone.gps().z, hover_height, dt); //for hovering at the same point

	drone.fly_velocity(vx, vy, vz);
}

void sig_handler(int signo)
{
    if (signo == SIGINT) {
        end_threads = true;
    }
}

