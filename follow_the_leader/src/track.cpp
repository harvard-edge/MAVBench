#include "track.h"
#include <chrono>
#include <bits/stdc++.h>
#include <stdio.h>
//std::ofstream file_to_output_2;
KCFtracker::KCFtracker(const bounding_box& bb, cv::Mat frame) : tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB)
{
 	
    tracker.init(cv::Rect(bb.x, bb.y, bb.w, bb.h), frame);
	peak = 0;
    
    //file_to_output_2.close();
    //file_to_output_2.open("/home/nvidia/catkin_ws/src/mav-bench/follow_the_leader/src/tracking_output_2.txt");
}

bounding_box KCFtracker::track(cv::Mat frame) {
	bounding_box result;
	double old_peak = peak;
	
    //auto trk_s  = std::chrono::steady_clock::now();
    cv::Rect rect = tracker.update(frame, &peak);
    //auto trk_e  = std::chrono::steady_clock::now();
    //auto trk__t = std::chrono::duration_cast<std::chrono::milliseconds>(trk_e - trk_s).count();
    //file_to_output_2<<trk__t<<"ms"<<std::endl;

	result.x = rect.x;
	result.y = rect.y;
	result.w = rect.width;
	result.h = rect.height;
	result.conf = peak != 0 ? peak/old_peak : 1.0;

	return result;
}
/*
TLDtracker::TLDtracker(const bounding_box& bb, cv::Mat frame)
{
	cv::Mat grey;

#if CV_MAJOR_VERSION==3
	cv::cvtColor(frame, grey, COLOR_BGR2GRAY);
#else
	cv::cvtColor(frame, grey, CV_BGR2GRAY);
#endif //CV_MAJOR_VERSION==3

	tracker.detectorCascade->imgWidth = grey.cols;
	tracker.detectorCascade->imgHeight = grey.rows;
	tracker.detectorCascade->imgWidthStep = grey.step;

	cv::Rect rect(bb.x, bb.y, bb.w, bb.h);
	tracker.selectObject(grey, &rect);
}

bounding_box TLDtracker::track(cv::Mat frame) {
	bounding_box result = {-1, -1, -1, -1};

	tracker.processImage(frame);

	if(tracker.currBB != 0) {
		cv::Rect rect = *(tracker.currBB);
		result.x = rect.x;
		result.y = rect.y;
		result.w = rect.width;
		result.h = rect.height;
		//result.conf = tracker.currConf;
		result.conf = 1.0;
	}

	return result;
}
*/
